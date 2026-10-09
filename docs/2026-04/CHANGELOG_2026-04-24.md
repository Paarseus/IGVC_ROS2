# 2026-04-24 — Semantic segmentation layer scaffolding (Phases 0–3)

**Scope.** Stand up the plumbing for camera-based perception feeding Nav2 via
`kiwicampus/semantic_segmentation_layer`. ZED X front camera + stub perception
node + kiwicampus costmap plugin wired end-to-end. No real CV yet; the stub
exists specifically to prove the contract before we commit to HSV or ML.

**Jetson access was unavailable.** Everything here is code, config, and docs.
Runtime verification (colcon build against CUDA, topic checks, costmap
inspection) is blocked on the next Jetson session — see "Verification pending"
at the bottom.

---

## Why

- Current Nav2 stack sees obstacles only through the Velodyne (VoxelLayer +
  ObstacleLayer). LiDAR can't see white lane paint, can't distinguish an
  orange barrel from a gray one, can't identify a white-painted pothole.
- IGVC AutoNav needs all three, plus the ability to navigate on the road-
  surface information that only a camera provides.
- We also need a *swappable* perception path: classical HSV for a fast bring-
  up, learned segmentation later without rewriting the Nav2 side.

## The decision

Adopt `kiwicampus/semantic_segmentation_layer` as the integration bridge. It
is a proper pluginlib Nav2 costmap layer — the "right" place for this seam.
The CV algorithm lives *outside* the plugin, in a separate `avros_perception`
node that publishes a 4-topic contract the plugin consumes. Classical HSV
today, ONNX later — same contract, same Nav2 config.

This is the standard and idiomatic ROS2 pattern; it's also what the user
explicitly asked for ("standard and professional, no workarounds").

## Multi-agent research summary

Five agents scoped the problem before planning. Condensed findings:

- **kiwicampus audit.** Apache-2.0, low-traffic but maintained, branches for
  rolling/jazzy/humble. Humble build needs community PR #1 applied (adds
  `#include <deque>` + swaps imported CMake targets for
  `ament_target_dependencies`). Ships no ML code — caller supplies mask,
  cloud, labels, and optional confidence.
- **Alternatives survey.** IGVC AutoNav winners 2019–2025 all use classical
  CV (HSV + warpPerspective); Oklahoma (4 of last 5 wins) uses zero LiDAR.
  `spatio_temporal_voxel_layer` is the right "replace VoxelLayer" option
  eventually, orthogonal to this work.
- **Prior-art code.** RoboJackets shipped the only proper pluginlib costmap
  layer in an IGVC repo (ROS1, archived). Sooner Robotics 2023/2025 ships
  150 LoC HSV → OccupancyGrid. Hosei 2025 dropped cameras entirely for
  LiDAR-intensity lane detection — a strong Phase 5 side-bet, but the layer
  doesn't fit that architecture.
- **AVROS integration surface.** `avros_perception/` existed but was empty.
  Three `zed_*.yaml` configs already present (left/right/back); right had a
  hardware-fault flag. `zed-ros2-wrapper` was pinned to `master` (unsafe).
  RealSense wired into local costmap as `camera_depth`.

Conclusion: use kiwicampus (bridge the problem cleanly), start with a
stub-pipeline perception node (de-risk plumbing), keep RealSense in place
(additive, not replacement), use ZED X front for Phase 1.

## Architecture

```
  ZED X (front)                                    Nav2
  /zed_front/zed_node/rgb/image_rect_color         local_costmap
  /zed_front/zed_node/point_cloud/cloud_registered   ├── voxel_layer (Velodyne + RealSense)
         │                                            ├── semantic_front  (kiwicampus)
         ▼                                            └── inflation_layer
  avros_perception/perception_node                        ▲
   ├─ message_filters ApproximateTime (slop 0.02s)         │
   ├─ Pipeline.run(rgb) → mono8 mask + confidence         │ subscribes
   └─ publish (all share header.stamp):                   │
        /perception/front/semantic_mask       ────────────┤
        /perception/front/semantic_confidence ────────────┤
        /perception/front/semantic_points     ────────────┤  (organized PC2)
        /perception/front/label_info          ────────────┘  (transient_local)
```

The pipeline-selector param (`pipeline: stub|hsv|onnx`) swaps the CV
implementation without changing any subscription, publisher, or Nav2 param.

## Decisions baked in

| Decision | Why |
|---|---|
| One `perception_node` process per camera (not a multiplexer) | Crashing HSV on one cam doesn't take down the others; TRT can schedule three engines across streams without internal locking. |
| Plain `Node()` for the ZED wrapper (not a ComponentContainer) | Matches the rest of AVROS. IPC is a measurable-bottleneck optimization, not a default. Revisit if 3-cam CPU is hot. |
| `zed_back_camera_center` kept in URDF, gated behind `enable_zed_back` xacro arg | Serial 49910017 was repurposed to front; keeping the mount + frame docs is cheaper than reconstructing them later. |
| Kiwicampus Humble patch carried as a `.patch` file + apply script | No fork to maintain. When upstream merges PR #1, delete the patch and advance the `avros.repos` pin. |
| `perception.launch.py` included from `navigation.launch.py`, NOT `sensors.launch.py` | Perception depends on camera topics existing; it belongs at the Nav2 layer. Keeps `sensors.launch.py` pure-driver. |
| RealSense stays in the local costmap as `camera_depth` | Known-good, additive. We can drop it in Phase 5 after measuring whether ZED front covers its role. |
| All `enable_*` args default `false` for new hardware | Don't change startup behavior on `master` until the Jetson says the hardware is live. |

## Data-flow contract

Kiwicampus requires `(mask, pointcloud, labels)` tuples with these properties:

- **Mask**: `sensor_msgs/Image` mono8, pixel value = class ID, same H×W as the
  RGB frame it was produced from.
- **PointCloud2**: **organized** (`height > 1`, row-major), same H×W as the
  mask, so `(u,v) → (x,y,z)` is a direct array lookup.
- **LabelInfo**: `vision_msgs/LabelInfo` with the class ID ↔ name mapping,
  published once with `transient_local` + `reliable` QoS so late-subscribing
  Nav2 plugins get the latched value.
- **header.stamp identical** across mask, cloud, confidence — the plugin
  uses `message_filters::ApproximateTime` and silently drops frames whose
  stamps diverge. `perception_node` copies the incoming image stamp onto
  every outbound message.

Deviating from any of these results in the plugin being silent: no errors,
just no cost written. This is the primary failure mode documented in
`CLAUDE.md` under Known Issues.

---

## Files created

### ZED X front camera
- `src/avros_bringup/config/zed_front.yaml` — camera_model `zedx`, serial
  `49910017` (repurposed from back), HD720 @ 15 fps, PERFORMANCE depth,
  5 Hz point cloud. Comments explicitly flag organized-cloud requirement.

### avros_perception package
- `src/avros_perception/package.xml` — MIT, ament_python. Depends rclpy,
  sensor_msgs, vision_msgs, cv_bridge, message_filters, numpy, opencv, yaml.
- `src/avros_perception/setup.py` — entry point `perception_node`,
  installs `config/` + `launch/` to share.
- `src/avros_perception/setup.cfg`, `resource/avros_perception` — standard.
- `src/avros_perception/avros_perception/__init__.py` — empty.
- `src/avros_perception/avros_perception/perception_node.py` — the main
  node. Subscribes `image_rect_color` + `cloud_registered` via
  `message_filters.ApproximateTimeSynchronizer(queue=10, slop=0.02)`.
  Publishes mask/confidence/cloud/labels with shared `header.stamp`.
  Registers an on-set-parameters callback that updates the pipeline's
  params dict in-place (so `ros2 param set ... inject_stripe_width 100`
  takes effect live). Asserts `mask.shape == (cloud.height, cloud.width)`;
  warns once if they diverge.
- `src/avros_perception/avros_perception/pipelines/__init__.py` — exposes
  `PIPELINES = {'stub': StubPipeline}` and a `build_pipeline()` factory.
- `src/avros_perception/avros_perception/pipelines/base.py` — abstract
  `Pipeline` + `PipelineResult(mask, confidence)` dataclass. `run()` takes
  optional depth so future pipelines (ONNX panoptic) can use it without
  breaking the contract.
- `src/avros_perception/avros_perception/pipelines/stub.py` — zeros mask
  except for a vertical stripe `[start, start+width)` filled with
  `inject_class_id`. Width `0` disables injection.
- `src/avros_perception/avros_perception/utils/__init__.py` — empty.
- `src/avros_perception/avros_perception/utils/class_map.py` — YAML loader
  producing `ClassEntry` records and a `vision_msgs/LabelInfo` message.
- `src/avros_perception/config/class_map.yaml` — single source of truth.
  Stub classes: `free=0`, `obstacle=1`, `unknown=255`. Phase 4 splits
  `obstacle` into `lane_white`, `barrel_orange`, `pothole`.
- `src/avros_perception/config/perception.yaml` — `perception_node`
  params: `camera_name`, topic overrides, `pipeline: stub`, sync slop,
  stub stripe knobs.
- `src/avros_perception/launch/perception.launch.py` — single Node launch
  with `camera_name` + `use_sim_time` args. Phase 5 extends to N cams.

### Kiwicampus integration
- `src/avros_bringup/patches/kiwicampus_pr1.patch` — mbox format, 127
  lines, fetched from `https://github.com/kiwicampus/semantic_segmentation_layer/pull/1.patch`.
  Fixes Humble build by swapping modern imported-target link rules for
  `ament_target_dependencies` and adding `#include <deque>`.
- `scripts/apply_kiwicampus_patches.sh` — idempotent. Uses
  `git apply --reverse --check` to detect already-applied state; otherwise
  runs `git am --keep-non-patch`. Aborts cleanly on failure.

### Documentation
- `docs/CHANGELOG_2026-04-24.md` — this file.

## Files modified

| File | Change |
|---|---|
| `avros.repos` | Pinned `zed-ros2-wrapper` to `v5.2.2` (with `TODO(jetson)` comment); added `semantic_segmentation_layer` on `humble` branch. |
| `.gitignore` | Added `src/semantic_segmentation_layer/`. |
| `src/avros_bringup/urdf/avros.urdf.xacro` | Added `xacro:arg enable_zed_back` (default false); added `zed_front_camera_center` link + joint (xyz `1.0 0 0.9`, rpy `0 0 0`, TODO mount measurement); wrapped existing `zed_back_camera_center` in `xacro:if`. |
| `src/avros_bringup/launch/sensors.launch.py` | Added `enable_zed_front` arg (default false); added `zed_wrapper` Node with `namespace=zed_front`, `name=zed_node`, gated on the arg. |
| `src/avros_bringup/launch/localization.launch.py` | Plumbed `enable_zed_front` arg through to `sensors.launch.py`. |
| `src/avros_bringup/launch/navigation.launch.py` | Added `enable_zed_front` + `enable_perception` args; added `IncludeLaunchDescription` for `avros_perception/launch/perception.launch.py` gated on `enable_perception`; imported `IfCondition`. |
| `src/avros_bringup/config/nav2_params.yaml` | Local costmap plugins list now `[voxel_layer, semantic_front, inflation_layer]`; added `semantic_front` block (kiwicampus class string, topic contract, `class_types: [danger]` with `obstacle` mapped to LETHAL 254, `tile_map_decay_time 1.5`). |
| `src/avros_bringup/config/nav2_params_humble.yaml` | Same change as above (Humble-targeted config). |
| `src/avros_perception/avros_perception/perception_node.py` | Added `semantic_confidence` publisher (was missing; kiwicampus accepts it as optional but we already compute it in `PipelineResult`). |
| `CLAUDE.md` | New "ZED X Front Camera" subsection under Sensors; updated Packages/Launch Files/Config Files tables; 2 new Known Issues rows (kiwicampus Humble patch + silent-drop gotchas); new "Perception / semantic segmentation" TODO section. |

## Local verification (done)

All syntax/schema checks passed on this workstation (no CUDA, no ZED SDK):

- `python3 -m py_compile` on every .py file under `avros_perception/` and
  every modified launch file: OK.
- `yaml.safe_load` on `avros.repos`, `zed_front.yaml`, both `nav2_params*.yaml`,
  `perception.yaml`, `class_map.yaml`: OK.
- `xacro avros.urdf.xacro` with default args: produces valid URDF, 1 `zed_front`
  link, 0 `zed_back` links.
- `xacro avros.urdf.xacro enable_zed_back:=true`: produces valid URDF, 1
  `zed_front`, 1 `zed_back`.
- `kiwicampus_pr1.patch` is mbox format (starts with `From <sha>`), 127
  lines. `scripts/apply_kiwicampus_patches.sh` is executable (mode 0755).

## Hardware verification — complete on Jetson (`~/IGVC`, not `~/AVROS`)

Correction to earlier assumption: the active workspace on the Jetson is
`~/IGVC` (remote `Paarseus/IGVC_ROS2.git`, branch `main`), not `~/AVROS`.
`~/AVROS` and `~/IGVC_ROS2` also exist on the Jetson but are stale/
experimental trees (`~/AVROS` had a `feature/rtabmap-slam` branch with a
parallel semantic-seg effort — untouched this session). Transfer used
rsync of today's uncommitted delta after `git pull` on Jetson.

**Installed state confirmed:** ZED SDK 5.2.0, JetPack 6 L4T R36 (kernel
5.15.148-tegra), CUDA present, `ZED_Explorer` sees the front camera.

### Results

| Phase | Evidence | Pass |
|---|---|---|
| 0 | `colcon build --symlink-install` — 17 packages in 3m34s, kiwicampus (1m5s) + zed_components (3m17s) clean | ✅ |
| 1 | Topic tree `/zed_front/zed_node/rgb/color/rect/image` + `point_cloud/cloud_registered` @ 15 Hz, organized 256×448, `base_link → zed_front_camera_link` resolves to `[1.0, 0, 0.9]` | ✅ |
| 2 | `perception_node` active, LabelInfo latched w/ 3 classes, `semantic_mask` @ 14.97 Hz; `ros2 param set inject_stripe_width 200` → 51,200 non-zero mask pixels (exactly `200 cols × 256 rows`) | ✅ |
| 3 | `/local_costmap/costmap` publishes 250×250; stripe injection → `max_cost=100` (LETHAL-in-OccupancyGrid scale), `cells_gt0=2,476`; baseline returns to 0 | ✅ |

Tile decay after stripe-off takes longer than `tile_map_decay_time` in
the stub configuration — the stub publishes class `free` when the
stripe is off, which kiwicampus ignores rather than using it to refresh
tiles, so the old `obstacle` tiles linger until their individual decay
timer elapses. Not a bug in our pipeline; won't show up in a real CV
pipeline that publishes a diverse class distribution every frame.

### Bugs found during bring-up + fixes

Nine issues surfaced only once real hardware was in the loop. Each is
now fixed in the code; documenting them here so future me / future
teammates don't re-discover them from scratch.

1. **`zed_wrapper` is a metapackage; the camera is a composable component.**
   `Node(package='zed_wrapper', executable='zed_wrapper')` fails with
   *"libexec directory ... does not exist"*. Fix: launch via
   `IncludeLaunchDescription(zed_camera.launch.py)` — the wrapper's own
   launch composes `stereolabs::ZedCamera` into a container.
2. **Namespace + node_name collision.** Passing both `namespace='zed_front'`
   and `node_name='zed_node'` to `zed_camera.launch.py` silently rewrites
   `node_name` to `camera_name` (`zed_camera.launch.py:242-245`),
   producing `/zed_front/zed_front/...`. Fix: pass only `camera_name`,
   let the wrapper default everything else. Canonical tree becomes
   `/zed_front/zed_node/...`.
3. **`zed_macro.urdf.xacro` required, not a hand-rolled link.** The
   wrapper publishes image and cloud `frame_id`s in
   `zed_<name>_camera_link`, `zed_<name>_left_camera_frame_optical`, etc.
   A single hand-rolled `zed_front_camera_center` link leaves the other
   frames missing — downstream `lookupTransform` silently fails. Fix:
   `<xacro:include filename="$(find zed_wrapper)/urdf/zed_macro.urdf.xacro"/>`
   + `<xacro:zed_camera name="zed_front" model="zedx">` + a separate
   joint from `base_link` to `zed_front_camera_link`.
4. **`depth_mode: 'PERFORMANCE'` is not a valid v5.2 enum.** The wrapper
   silently falls back to AUTO and then throws `InvalidParameterValueException`
   during node init. v5.2 enum is `NONE | NEURAL_LIGHT | NEURAL | NEURAL_PLUS`.
   Use `NEURAL_LIGHT` (11% GPU on Orin — right size for multi-cam).
5. **`grab_resolution: 'HD720'` is not valid for ZED X.** ZED X's enum is
   `HD1200 | HD1080 | SVGA | AUTO`. `HD720` triggers a warning and fallback.
   Use `HD1080` as the closest equivalent.
6. **`serial_number` is a launch arg, not a YAML param.** Putting
   `general.serial_number: 49910017` in the override YAML silently does
   nothing (the wrapper logs `* Camera SN: 0`). Fix: pass via launch
   arg `'serial_number': '49910017'` on the `IncludeLaunchDescription`.
7. **`rgb/image_rect_color` is a v4 topic name; v5 publishes at
   `rgb/color/rect/image`.** Affected `perception_node`'s default topic
   path. Fixed in the node's fallback construction.
8. **Image and organized cloud come out at different resolutions.**
   Wrapper respects `pub_downscale_factor` on the image (HD1080 → 540×960)
   but the cloud uses `point_cloud_res: COMPACT` → 256×448. kiwicampus
   indexes `cloud[v,u]` to look up 3D per mask pixel, so the mask **must**
   match the cloud's HxW. Fix: `perception_node` resizes the image to the
   cloud's shape before running the pipeline (`cv2.resize INTER_AREA`).
9. **`class_types` must be nested inside each observation source**, not
   at the plugin top level. The kiwicampus plugin declares them under
   `layer_name.source.class_types`, so a top-level `class_types` produces
   the runtime error *"no class types defined for source front"* and
   kills `controller_server`. README formatting was ambiguous on this.

### File changes this iteration (bug fixes on top of morning edits)

| File | Change |
|---|---|
| `src/avros_bringup/launch/sensors.launch.py` | `IncludeLaunchDescription` of stock `zed_camera.launch.py` with only `camera_model` + `camera_name` + `serial_number` + `publish_tf='false'` + `publish_urdf='false'` + override path. No `namespace`/`node_name`. |
| `src/avros_bringup/urdf/avros.urdf.xacro` | Replaced hand-rolled `zed_front_camera_center` with `<xacro:include .../zed_macro.urdf.xacro>` + `<xacro:zed_camera name="zed_front" model="zedx">` + explicit joint from `base_link` to `zed_front_camera_link`. |
| `src/avros_bringup/config/zed_front.yaml` | Rewritten: `HD1080 @ 15` (not `HD720`), `NEURAL_LIGHT` (not `PERFORMANCE`), `point_cloud_freq: 15.0` to match image, explicit off for `pos_tracking`/`mapping`/`object_detection`/`body_tracking`. Serial moved out (launch arg now). |
| `src/avros_bringup/config/nav2_params.yaml` + `nav2_params_humble.yaml` | `class_types` + per-type blocks moved INSIDE the `front:` observation source block. `max_obstacle_distance: 100.0` and `min_obstacle_distance: 0.0` during bench-test (narrow back down for field work). `visualize_tile_map: true` for debug. |
| `src/avros_perception/avros_perception/perception_node.py` | Default `rgb_topic` path corrected to `rgb/color/rect/image` for v5 wrapper. `cv2.resize(INTER_AREA)` applied when cloud HxW ≠ image HxW (always the case under defaults). Removed the stale shape-mismatch warn-once flag. |
| `docs/CHANGELOG_2026-04-24.md` | This section. |

### Final verified state

```
ros2 launch avros_bringup navigation.launch.py \
    enable_zed_front:=true enable_perception:=true
ros2 param set /perception_node inject_stripe_width 200
# → /local_costmap/costmap shows 2,400+ LETHAL cells in front of vehicle
```

Everything from SSH verification → build → sensor up → perception topics
→ costmap cost injection works end-to-end on the Jetson.

## What's next (out of scope for this session)

- **Phase 4** — replace `StubPipeline` with `HSVPipeline`. White lanes + orange
  barrels. New class IDs in `class_map.yaml`; re-key kiwicampus `class_types`
  accordingly. No Nav2 changes beyond the class cost table.
- **Phase 5** — add `zed_left` + `zed_right` (new unit, serial TBD). Two more
  wrapper Nodes, two more `perception_node` instances with different
  `camera_name`, two more kiwicampus plugin blocks (`semantic_left`,
  `semantic_right`). Decide whether to drop RealSense from VoxelLayer.
- **Phase 6 (later)** — ONNX/TRT pipeline behind the same contract. Requires
  a labeled course dataset; not worth starting until Phase 4 is field-tested.

## References

- Approved plan: `~/.claude/plans/temporal-drifting-shamir.md`
- Plugin repo: https://github.com/kiwicampus/semantic_segmentation_layer
- Humble fix PR: https://github.com/kiwicampus/semantic_segmentation_layer/pull/1
- ZED wrapper: https://github.com/stereolabs/zed-ros2-wrapper (pinned `v5.2.2`)
