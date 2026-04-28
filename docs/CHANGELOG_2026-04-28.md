# 2026-04-28 — Costmap dual-clock decay bug + wheel-odom EKF fusion + NoMachine fix

**Scope.** Long methodical session, three main threads:

1. **NoMachine remote-desktop black-screen fix** — GDM was silently re-enabling itself, NoMachine was attaching to the empty greeter on `:0` instead of spawning a fresh virtual session on `:1001`.
2. **Local costmap stuck at 7 LETHAL cells while internal kiwicampus tile_map carried 280–750 tiles per snapshot** — diagnosed via a 5-phase multi-agent investigation, root-caused to a dual-clock decay bug in the kiwicampus plugin (`bufferSegmentation` purges with cloud header stamp, `updateBounds` purges with `node->now()`), fixed by bumping `tile_map_decay_time: 1.5 → 5.0`.
3. **Robot rotated in costmap but didn't translate** — EKF #1 (`odom → base_link`) had no working translation source. `/wheel_odom` was being published by `actuator_node` but with all-zero covariance (silently rejected by `robot_localization`) and was not even subscribed in `ekf.yaml`. Fixed by patching `actuator_node` to publish proper diagonal covariance, adding `odom0: /wheel_odom` to EKF #1, and removing the broken `twist0: /filter/twist` source. Verified by a 14 m drive (EKF translation tracking works) and a 360° spin calibration test (track width ≈0.7% off, covariance values well-tuned).

Companion to `docs/CHANGELOG_2026-04-27.md` (mutex patch, decay sweep) and `docs/CHANGELOG_2026-04-27_field.md` (slim launch, HSV outdoor calibration).

Files touched:

```
M  CLAUDE.md
M  src/avros_bringup/config/ekf.yaml
M  src/avros_bringup/config/nav2_params_humble.yaml
M  src/avros_bringup/config/perception_test_params.yaml
M  src/avros_control/avros_control/actuator_node.py
?? docs/CHANGELOG_2026-04-28.md                                (this file)
```

Single squashed commit on origin/main: `a750e37` ("Fix wheel-odom EKF fusion + costmap decay").

Out of scope (left for future): tighten HSV `pothole` thresholds (currently false-positive on concrete shadows ≈14 % of frame), 5 m straight-line wheel-odom calibration test, GPS-vs-odom drift comparison.

---

## 1. NoMachine black-screen on local Wi-Fi

**Symptom.** Connecting to the Jetson with NoMachine over the local 192.168.13.0/24 network produced a black screen. The Tailscale IP behaved identically; only the connection IP changed, the failure mode didn't.

**Root cause.** GDM (`gdm.service`) had been re-enabled at some point (possibly by a package upgrade or a JetPack reset). With GDM running but no user logged in graphically, NoMachine attached to the empty `:0` greeter and faithfully mirrored its blank screen instead of spawning a fresh virtual session on `:1001`. Documented in NoMachine KB AR03P00973 and previously called out in `CLAUDE.md`'s Remote Desktop section, but had regressed.

```
$ sudo /usr/NX/bin/nxserver --list
Display Username Remote IP       Session ID                       Node
0       gdm      -               8EA6B460A5E541A01559AF8CA0B35F26 localhost:4000
```

That `display 0 / user gdm / no remote IP` row is the empty greeter NoMachine was happily showing.

**Fix.**

```bash
sudo systemctl stop gdm
sudo systemctl mask gdm                       # `static` units can't be plain-disabled
sudo systemctl set-default multi-user.target  # boot text-mode by default
```

`mask` rather than `disable` because `gdm.service` has `static` install info; only masking actually prevents it from starting via `graphical.target` dependencies. Reversible via `systemctl unmask gdm && systemctl set-default graphical.target && systemctl start gdm` if a real monitor is later attached and auto-login is desired.

**Note on local IP.** The Jetson reports two IPv4 addresses besides Tailscale: `wlP1p1s0 = 10.110.132.84/21` (Wi-Fi to a different SSID it remembers) and `eno1 = 192.168.13.10/24` (sensor LAN). The laptop in the lab was actually on the 192.168.13.0/24 net (`192.168.13.108`, default gw `192.168.13.31`), so `192.168.13.10:4000` is the right NoMachine target, not `10.110.132.84` despite the latter sounding like the "Wi-Fi" IP. CLAUDE.md's network inventory describes 192.168.13.0/24 as a "sensor LAN", but in the current lab it is the Wi-Fi network — verified by `ip route` showing the laptop's default route through `192.168.13.31`.

---

## 2. Local costmap stuck at 7 LETHAL cells

**Symptom.** With the perception_test stack running (front ZED + HSV perception + controller_server + kiwicampus semantic_layer), the master `/local_costmap/costmap` had exactly **7 LETHAL cells + 50 inflation cells** every snapshot, **frozen across multiple seconds of samples**, while the camera was clearly producing detections. Internal kiwicampus tile buffer (`/local_costmap/front/tile_map`) carried 280–750 tile entries per snapshot. The HSV overlay showed ~14 % of the frame painted as "pothole" (concrete misclassification — separate issue, see TODO list).

The compression ratio internal-buffer → master-grid was therefore ≈40–100×, far worse than expected from coordinate downsampling alone.

### 2.1 Multi-phase investigation protocol

A 5-phase methodical investigation. Each phase's gate had to produce a clean signal before moving to the next. Where work was independent within a phase, it was parallelized via subagents.

| Phase | What | Gate |
|---|---|---|
| 0 | Baseline freeze: snapshot every topic + node + param + git state to `/tmp/phase0/` | reproducibility ground truth |
| 1 | Per-topic integrity (4 parallel agents): mask, points, confidence, label_info | all four publishing, hxw match, stamps fresh |
| 2 | ApproximateTime sync feasibility (1 agent, 100 triples) | ≥90 % triples within plausible slop |
| 3 | Internal `tile_map` vs master `costmap` content (2 parallel agents) | does loss happen before or after master-merge? |
| 4 | Source-code trace of `SemanticSegmentationLayer` (3 parallel agents) | named, line-cited hypothesis |
| 5 | Targeted experiments — one variable at a time | cell count tracks input within an order of magnitude |

### 2.2 Key findings per phase

**Phase 1 (per-topic integrity).** All four perception output topics publishing at 14.99 Hz with 256×448 hxw matching the cloud's COMPACT resolution. Two anomalies that turned out to be red herrings:

- Mask + confidence carry `frame_id=zed_front_left_camera_frame_optical`, while the cloud carries `frame_id=zed_front_left_camera_frame` (the non-optical parent). Irrelevant for ApproximateTimeSync (uses stamps) and for index-based association (matches `(u,v)` indices regardless of frame).
- Confidence is binary `mono8` (only values 0 and 1), not a continuous probability. With `mark_confidence: 0` and `LabelInfo.threshold: 0.0` both set to 0, every observation passes — confidence isn't gating anything.

`label_info` topic verified RELIABLE + TRANSIENT_LOCAL with all 5 classes — kiwicampus's required QoS contract is met.

**Phase 2 (sync feasibility).** Stamps on mask, cloud, and confidence are bit-identical (`perception_node` stamps all three with the source image stamp). 100 % of mask messages can sync at any non-zero slop. **Sync is not the bottleneck.**

**Phase 3 (internal vs master).**

| | `/local_costmap/front/tile_map` (internal) | `/local_costmap/costmap` (master) |
|---|---|---|
| Type | `sensor_msgs/PointCloud2` (sparse, fields `x,y,z,confidence,confidence_sum,class`) | `nav_msgs/OccupancyGrid` (200×200 dense) |
| `frame_id` (header) | `map` *(see 2.3)* | `odom` |
| Steady-state count | 305–623 tiles (median 311) at decay 1.5; 856–939 (median 893) post-fix | 7 LETHAL frozen |

The tile_map's spatial bbox `x[0.35, 5.55] × y[-6.85, -0.75]` matched the cloud projection's bbox in odom `x[0.46, 8.33] × y[-6.42, -0.27]` almost exactly — i.e. the data is geometrically correct, the tile_map is just labelled `frame_id="map"` inside kiwicampus while the coordinates are actually in the buffer's `global_frame_` (which is `odom` for a local_costmap). Mislabel, not a real frame conflict.

The 7 master cells fall **inside** the projected pothole bbox, so the spatial location is consistent. The loss is inside the tile→master transfer, not in the projection.

**Phase 4 (source trace).** Three parallel agents read `segmentation_buffer.cpp/.hpp` and `semantic_segmentation_layer.cpp/.hpp` (kiwicampus, with our PR#2-carry mutex patch already applied). Findings, file:line cited:

- **`bufferSegmentation` per-frame survival is one observation per tile** (`segmentation_buffer.cpp:140, 172–195`). 21,500 cloud points → at most ~one survivor per occupied cell per frame.
- **Class filter is at `cpp:212`**, not later. Pixels whose class ID isn't in `segmentation_cost_multimap_` are silently dropped here. Since our config's `class_types: ["danger"]` / `danger.classes: ["lane_white", "barrel_orange", "pothole"]` excludes `free` (id=0) and `unknown` (id=255), those classes never reach the tile_map. This is also where the startup `CRITICAL ERROR: Class 'free' from label_info is not defined in the costmap parameters` warnings originate. Intra-tile "dominance" between free and pothole is therefore **not** the issue.
- **Two purge paths run on different clocks:**
  - `bufferSegmentation` calls `purgeOldObservations(cloud_time_seconds)` (cpp:204) — sensor stamp.
  - `updateBounds` calls `purgeOldObservations(node->now().seconds())` (`semantic_segmentation_layer.cpp:323, 339`) — wall clock.
  - Observations are stored stamped with `cloud_time_seconds` (`segmentation_buffer.cpp:213`).

  The applied `kiwicampus_pr2_mutex.patch` fixes a different race (was: `purgeOldObservations` running without the tile-map's own recursive mutex, allowing `bufferSegmentation` to mutate the queue mid-purge). The patch restores per-purge correctness, but it does **not** address the dual-clock comparison, which is what's actually evicting observations on `updateBounds` cycles.

- **Master-merge is `updateWithMax` over the bounds region** (`updateCosts`, `:389–419`); per-cell logic lives entirely in `updateBounds`. `worldToMap` failures inside `updateBounds` (`:351–355`) drop tiles silently with `RCLCPP_DEBUG`.

**Phase 5 (single-variable experiment).** Bumped `tile_map_decay_time: 1.5 → 5.0` in `perception_test_params.yaml`, restarted **only** `controller_server` + `lifecycle_manager_perception_test`, sampled the same 10 s × 2 topics protocol as the baseline.

| Metric | decay 1.5 (baseline) | decay 5.0 | Δ |
|---|---|---|---|
| `/local_costmap/costmap` nonzero (p50) | 57 | **1759** | 31× ↑ |
| `/local_costmap/costmap` LETHAL (p50) | 7 | **990** | **141× ↑** |
| `/local_costmap/front/tile_map` tile_count (p50) | 311 | 893 | 2.9× ↑ |
| Compression (tile_map → LETHAL) | 44× | 0.9× | reasonable |

LETHAL count slightly exceeds tile_count post-fix because the master grid retains cells across cycles via `updateWithMax` while the buffer is the live snapshot. Inflation = 1759 − 990 = 769 cells, consistent with `inflation_radius: 0.3` puffing each LETHAL cell into a small ring.

### 2.3 Why `tile_map` had `frame_id="map"`

The kiwicampus visualization publisher inside `SegmentationBuffer` stamps `frame_id` as the literal string `"map"` regardless of what the buffer's `global_frame_` is set to (verified by inspection of the on-wire frame_id). The buffer is constructed with `global_frame_ = layered_costmap_->getGlobalFrameID()`, which for a local_costmap is `odom`, but the published viz topic's frame_id is hardcoded as `map`. This is purely a labeling issue: the *coordinates* in the message are in the buffer's actual storage frame (`odom`), so RViz/Foxglove visualizing it with `Fixed Frame: map` will display tiles in the wrong place if `map ≠ odom`. Not load-bearing for any of today's fixes — flagging it for future debugging.

### 2.4 Configs touched

```yaml
# perception_test_params.yaml (active runtime config of the perception_test launch)
local_costmap:
  local_costmap:
    ros__parameters:
      semantic_layer:
        front:
-         tile_map_decay_time: 1.5
+         tile_map_decay_time: 5.0
```

```yaml
# nav2_params_humble.yaml (active runtime config when navigation.launch.py runs)
# 3 occurrences (one per source: front, left, right) all bumped 1.5 → 5.0.
```

The header comment block in `nav2_params_humble.yaml` was rewritten to document the dual-clock issue and the post-2026-04-28 measurement (≈990 LETHAL cells from the same input that previously produced 7), so the rationale doesn't depend on commit-message archaeology.

---

## 3. EKF #1 had no translation source — robot rotated in costmap but didn't translate

**Symptom.** With the costmap fix in place and the robot driven via webui, the robot **rotated** correctly in RViz but its position in odom stayed at `(0, 0)` indefinitely. The local_costmap rolling window (centered on `base_link`) therefore never moved with the robot — new pothole detections kept appearing at the same forward distance from a static-looking robot icon.

### 3.1 Diagnosis

Sampled the EKF chain end-to-end:

```text
/wheel_odom               20 Hz  position (-37.23, -33.92)  vx=0.51  vyaw=0.45  cov=ALL ZERO
/odometry/filtered (EKF1) 30 Hz  position (0.000, 0.000)    vx=0.00  vyaw=0.002 pose.cov[0]=1.95e35
odom -> base_link TF              translation [0,0,0]                  yaw -1.4 rad
```

Two structural problems:

1. `actuator_node` was publishing `/wheel_odom` with **every covariance entry at 0.0**. `robot_localization` treats all-zero covariance as "the message has no useful information" and silently discards it — both for fusion and for subscription pre-filtering.
2. `ekf.yaml`'s `ekf_filter_node_odom` only declared `imu0: /imu/data` (with `imu0_config` zeroing all position components — IMU only contributing orientation) and `twist0: /filter/twist`. The Xsens `/filter/twist` topic had a **dual-publisher type conflict**: two nodes were advertising it with different message types (`TwistStamped` vs `TwistWithCovarianceStamped`), causing `ros2 topic echo /filter/twist` to refuse the topic and EKF's binding to be undefined depending on discovery order. Net effect: EKF #1 had no working translation source, and `/odometry/filtered` reported pose with `cov[0] = 1.95e+35` — robot_localization's "I have absolutely no idea where this is" sentinel.

The user's mental model — "GPS goes into EKF #1" — is a common confusion with the dual-EKF pattern from the `nav2_gps_waypoint_follower_demo`. In that pattern, **GPS feeds EKF #2 only** (`map → odom`), so global GPS corrections live entirely in the `map → odom` hop and `odom → base_link` stays smooth. Local translation in EKF #1 must come from a continuous, drift-prone source — wheel encoders, VIO, or IMU integration. GPS into EKF #1 would cause `odom → base_link` to teleport on each correction and the local controller to oscillate.

### 3.2 `actuator_node.py` changes

**Forward kinematics audit (no change needed).** Per-wheel encoder rates are converted with the textbook diff-drive formulas:

```python
v = (l_mps + r_mps) / 2.0      # body linear vel
w = (r_mps - l_mps) / track_w  # body angular vel
```

`track_width_m: 0.7366` matches CLAUDE.md (29 in). Frames are correct (`odom`/`base_link`). Twist content uses `linear.x = v`, `angular.z = w`, with all other components zero — standard for 2D diff drive.

**Covariance (added).** The `_publish_odom` path now writes a fixed diagonal covariance set up once in `__init__`:

```python
# pose covariance (row-major 6x6: x, y, z, roll, pitch, yaw)
self._pose_cov = [0.0] * 36
self._pose_cov[0]  = 0.001   # x — modest trust, drifts over time
self._pose_cov[7]  = 0.001   # y
self._pose_cov[14] = 1e6     # z (2D — unobservable)
self._pose_cov[21] = 1e6     # roll
self._pose_cov[28] = 1e6     # pitch
self._pose_cov[35] = 0.01    # yaw

# twist covariance (vx, vy, vz, vroll, vpitch, vyaw)
self._twist_cov = [0.0] * 36
self._twist_cov[0]  = 0.0001  # vx — wheels measure this directly
self._twist_cov[7]  = 1e6     # vy (no lateral slip on diff drive)
self._twist_cov[14] = 1e6     # vz
self._twist_cov[21] = 1e6     # vroll
self._twist_cov[28] = 1e6     # vpitch
self._twist_cov[35] = 0.0001  # vyaw
```

Pattern: trust what wheels actually measure (`vx`, `vyaw`) with low covariance, mark unobservable axes with effectively-infinite covariance so the EKF ignores them. Standard from the robot_localization tutorial / TurtleBot / Husky reference configs.

**Pose integration (Euler → midpoint).** The original code applied forward Euler:

```python
# before
self._odom_yaw = wrap_angle(self._odom_yaw + w * dt)
self._odom_x += v * math.cos(self._odom_yaw) * dt
self._odom_y += v * math.sin(self._odom_yaw) * dt
```

This uses the *new* yaw to compute the position update, which over-rotates displacement on turns. Switched to trapezoidal/midpoint integration (2nd-order accurate):

```python
# after
yaw_delta = w * dt
yaw_avg = wrap_angle(self._odom_yaw + yaw_delta / 2.0)
self._odom_yaw = wrap_angle(self._odom_yaw + yaw_delta)
self._odom_x += v * math.cos(yaw_avg) * dt
self._odom_y += v * math.sin(yaw_avg) * dt
```

Sub-cm-per-meter improvement at IGVC speeds; matters more for tight turns and longer trajectories.

### 3.3 `ekf.yaml` changes

```yaml
# ekf_filter_node_odom (EKF #1)

# REMOVED — Xsens onboard filter velocity had a dual-publisher type conflict
#   twist0: /filter/twist
#   twist0_config: [...]
#   twist0_queue_size: 10

# ADDED — wheel odometry from actuator_node
+ odom0: /wheel_odom
+ odom0_config: [false, false, false,    # x, y, z position (don't fuse — wheels drift)
+                false, false, false,    # roll, pitch, yaw
+                true,  false, false,    # vx, vy, vz — fuse vx
+                false, false, true,     # vroll, vpitch, vyaw — fuse vyaw
+                false, false, false]    # ax, ay, az
+ odom0_differential: false
+ odom0_relative: false
+ odom0_queue_size: 10
```

The reason for removing `twist0: /filter/twist` rather than fixing the dual-publisher conflict: with wheel_odom + IMU now providing the standard diff-drive fusion pattern, `/filter/twist` is redundant. It was a vendor-specific Xsens path that adds a latent bug surface for no incremental capability. If a future use case needs Xsens-fused velocity (e.g., wheel encoder failure), it can be reintroduced after fixing the type conflict.

`odom0_config` fuses **only velocities** (`vx`, `vyaw`), never absolute pose. Wheels measure velocity directly with bounded error per timestep; integrating velocity into a pose is robot_localization's job, not actuator_node's. The accumulated `_odom_x`, `_odom_y`, `_odom_yaw` in `actuator_node`'s nav_msgs/Odometry are still computed and published, but the high covariance on those pose entries (`pose.cov[0] = 0.001` is "modest trust" not "trusted") combined with `odom0_config: false` for all pose components means EKF ignores them.

### 3.4 Verification — drive test

User drove via phone joystick (`https://192.168.13.10:8000`) for 30 s while a recorder captured `/odometry/filtered`, `/wheel_odom`, and `odom → base_link` TF.

Excerpt (every 1.5 s):

```
    t |   filt.x   filt.y  filt.vx filt.vyaw |  wheel.vx wheel.vyaw |    tf.x    tf.y   tf.yaw
------------------------------------------------------------------------------------------------
  1.5 |    2.615    0.203    0.222     0.014 |     0.222      0.000 |   2.615   0.203      3.9
  7.5 |    4.108    0.792    0.339    -0.225 |     0.339     -0.069 |   4.108   0.792     38.1
 15.1 |    7.575    2.236    0.450     0.016 |     0.450     -0.015 |   7.575   2.236     19.9
 22.6 |   10.764    3.453    0.456     0.015 |     0.456      0.016 |  10.764   3.453     23.7
 28.7 |   13.252    4.690    0.472     0.012 |     0.472      0.013 |  13.268   4.699     29.0
```

Observations:

- EKF position grew from (0, 0) to (13.25, 4.69) — **translation tracking works**.
- `/odometry/filtered.x ≈ TF odom→base_link.x` at every sample — the TF chain is publishing the new pose, so the local_costmap rolling window will follow the robot.
- `filt.vx == wheel.vx` to 3 decimals throughout — EKF correctly weights wheel velocity as primary translation source.
- `filt.vyaw` differs slightly from `wheel.vyaw` (e.g. at t=4.5 s: 0.393 vs 0.222) — EKF weights IMU more heavily for yaw, which is correct: IMU yaw rate is more accurate than wheel-derived yaw rate.
- Yaw drifted from 4° → 29° over 14 m — robot drove a slight curve (joystick steering input), matches.

EKF position covariance went from `1.95e+35` (rejecting all sources) to `4.57e+04` immediately after wiring `odom0`, and into the low single digits after a few seconds of motion. **31 orders of magnitude reduction in reported uncertainty.**

### 3.5 Verification — 360° spin calibration

Recorded EKF, wheel-only, and IMU-only integrated yaw rate over a single full spin in place. Ground truth: 360°.

| Source | Reported rotation | Error vs 360° | Interpretation |
|---|---|---|---|
| EKF (fused) | 358.34° | −1.66° (0.46 %) | Final answer |
| Wheel only | 362.48° | +2.48° (0.69 %) over | Effective track ≈0.7 % wider than configured |
| IMU only | 357.21° | −2.79° (0.78 %) under | IMU bias or stop point ≠ start point |

**Covariance well-tuned.** EKF result sits between wheel and IMU and closer to IMU, which is the textbook-correct behaviour for a diff-drive robot with both sources: trust IMU more for yaw because gyro is more accurate than wheel-derived yaw rate. If wheel `vyaw` covariance were too low, EKF would land near 362° (over-rotation); if too high, near 357° (effectively ignoring wheels). 358.34° says the relative weighting is balanced.

**Kinematics ≈0.7 % off.** Wheel-derived yaw says the robot rotated 2.48° more than it did. That implies effective track width is wider than the configured `0.7366 m`:

```
new track_width = 0.7366 × (362.48 / 360) ≈ 0.7416 m
```

About 5 mm wider than configured. Within machining tolerance for a tracked vehicle (the track contact-point spacing on a tank-style chassis is fuzzy). Not corrected today — added to TODOs (5 m straight-line drive will isolate `m_per_rev` from `track_width_m`).

---

## 4. Cleanup, commit, and push

### 4.1 Stale local state on the Jetson before changes

`git status` on the Jetson at the start of the session showed 9 modified files + 3 untracked + 2 commits behind origin/main. Spent some time investigating whether the modifications were unique field-tuning work or just stale working-tree state from past `git checkout`s.

Verification approach: for each modified file, computed the working-tree blob's SHA and searched commit history for an exact match.

```bash
git hash-object <file>
git log --all --pretty=oneline --diff-filter=AM -- <file>
```

Result: every single byte currently in the working tree existed somewhere in committed git history (either earlier in the Jetson's own history, or in the upcoming commits the Jetson was behind on). No unique work to preserve. Therefore safe to:

```bash
git fetch origin
git reset --hard origin/main
git clean -fd
```

This restored the Jetson to `bed93c1` cleanly, with no loss of work. Lesson recorded for future "what's all this M state I forgot about?" moments: **before assuming local mods are unique work, hash them and grep history.**

### 4.2 Commit message and push

Single squashed commit `a750e37` on origin/main covering all five files. Message:

```
Fix wheel-odom EKF fusion + costmap decay

- actuator_node: publish proper diagonal covariance on /wheel_odom
  (was zero, robot_localization rejected the message). Switch pose
  integration from forward Euler to trapezoidal/midpoint for 2nd-order
  accuracy on turns.
- ekf.yaml: add odom0: /wheel_odom block to EKF #1 fusing only vx + vyaw.
  Drop twist0: /filter/twist (Xsens dual-publisher type conflict; wheel
  odom + IMU is the standard diff-drive pattern and works).
- perception_test_params.yaml + nav2_params_humble.yaml: bump
  tile_map_decay_time 1.5 -> 5.0. Even with the kiwicampus mutex patch,
  the dual-clock purge (cloud stamp in bufferSegmentation, node clock in
  updateBounds) left only 7 LETHAL cells in a 200x200 master grid. 5.0s
  recovers ~990 LETHAL cells from the same input.
- CLAUDE.md: add 5m straight-line wheel-odom calibration TODO with
  context from today's 360 deg spin (track ~0.7% wider than configured).

Verified 2026-04-28 by 14m drive (EKF translation tracking confirmed)
and 30s sample (master costmap ~990 LETHAL vs 7 baseline).
```

Pushed from the laptop (gh CLI authenticated as `Paarseus`); the Jetson's HTTPS credentials are configured for a teammate (`Ryan-Simpson`) who doesn't have push access to `Paarseus/IGVC_ROS2`. After push, ran `git fetch && git reset --hard origin/main` on the Jetson to drop its local-only commit (with the same content but a different SHA) and adopt the laptop's published commit.

---

## 5. Stack composition reference (for future "launch everything correctly" questions)

This came up mid-session; recording the answer for future reference. To bring up the full perception+localization+drive stack with **only the front camera** (no LiDAR, no RealSense, no left/right ZED), the standard composition is:

| Layer | Source | Brings up |
|---|---|---|
| Sensors + EKF + GPS | `localization.launch.py enable_velodyne:=false enable_realsense:=false enable_zed_front:=true` | robot_state_publisher, ZED front, Xsens, NTRIP, dual EKF, navsat_transform |
| Perception | `avros_perception/perception.launch.py cameras:=front` | perception_node (HSV by default) |
| Drive | `webui.launch.py` | actuator_node + webui_node (HTTPS on `:8000`) |
| Costmap | `nav2_controller controller_server` + `nav2_lifecycle_manager` with `perception_test_params.yaml` | `/local_costmap/costmap` painted by `semantic_layer` |
| Visualization | `rviz2 -d /tmp/perception_overlay.rviz` on `DISPLAY=:1001` + `foxglove_bridge` | overlay + costmap + TF in NoMachine; Foxglove for laptop |

Sequencing: launch localization first, wait ~20 s for ZED + EKF to settle (otherwise the local_costmap activation can hit a TF cache miss and segfault on first cycle — observed once today, recovered cleanly on second attempt), then perception + webui + costmap in parallel, then RViz/Foxglove.

`navigation.launch.py` brings up the same plus all Nav2 servers (planner, smoother, BT, route_server) and `actuator_node` directly (no webui). For perception+manual-drive testing, the manual composition above is preferable.

---

## 6. TODO additions

Added under `### Actuator / drivetrain` in `CLAUDE.md`:

- [ ] **5 m straight-line wheel-odom calibration test** — drive 5 m on a tape-measured line, compare EKF `pose.position.x` to actual. Validates `m_per_rev` (wheel diameter / gear ratio) and trans-velocity covariance. 360° spin already done 2026-04-28 — wheels +0.7 % (effective track ~0.7416 m vs configured 0.7366 m); EKF fused result 358.34° (acceptable). Translational accuracy still untested.

Not added but worth tracking elsewhere:

- Tighten HSV `pothole_*` thresholds in `pipelines/hsv.py` — concrete walkway is being misclassified as pothole at ~14 % of frame. Out of scope today (the costmap+EKF fix is independent of detection accuracy), but visible in field test overlay snapshots.
- Investigate / clean up `/filter/twist` dual-publisher type conflict at the Xsens driver source level. Currently masked because we removed the topic from EKF, but it's still a latent advertising bug.
- Document the kiwicampus tile_map `frame_id="map"` mislabel in the patches/ directory README (not a load-bearing bug, but a confusing one when debugging spatial issues).
