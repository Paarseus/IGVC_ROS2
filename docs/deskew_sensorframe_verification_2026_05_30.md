# Velodyne De-skew + STVL `sensor_frame` — Definitive Verification

**Date:** 2026-05-30
**Audience:** IGVC team lead
**Scope:** Whether re-enabling Velodyne motion de-skew (`fixed_frame: "odom"`) together with STVL `sensor_frame: "velodyne"` is the correct, standard way to recover per-sweep ego-motion compensation without breaking robot-centric costmap clearing — and which load-bearing assumptions survived adversarial verification.

---

## TL;DR

> **⚠ Read §0 first.** Post-report source verification (raw `ros2`-branch read) found the upstream arg-swap is **real**, which means the headline `fixed_frame:"odom"` change (C1) does **not** de-skew — and the corrected config is *simpler* (one file, no STVL edits). §0 supersedes C1 and §8-step-1.

**Verdict: the proposed fix is VERIFIED CORRECT and STANDARD — with one important caveat the team must absorb.**

- The clearing-origin mechanism the fix targets is confirmed verbatim in STVL source (`measurement_buffer.cpp:97-98`) and independently re-verified by 3 adversarial lenses on every load-bearing claim. With de-skew ON, the published cloud `header.frame_id` becomes `"odom"`, so STVL anchors its clearing/frustum origin at the **odom origin (0,0,0)** instead of the robot. Adding `sensor_frame: "velodyne"` to each STVL `velodyne_points` source restores the robot-centric origin. **This works.**
- **Recommended approach: `sensor_frame: "velodyne"` (NOT `target_frame: "velodyne"`).** `target_frame` is arguably the *more idiomatic single-file* fix (velodyne issue #342), but adversarial verification surfaced a **real argument-order swap bug** in the upstream velodyne `transform.cpp` `configure()` call that corrupts the `target_frame` path. The `sensor_frame` route never relies on `target_frame`, so it sidesteps the bug. Use `sensor_frame`.
- **CAVEAT (new, from adversarial L1 verdict — affects the team's mental model, not the fix's correctness):** because of that same `configure()` arg-swap, setting `fixed_frame: "odom"` with `target_frame` empty does **NOT** actually run per-packet `computeTransformToFixed` de-skew on the installed binary. It runs a single per-*scan* `computeTransformToTarget` rotation into odom at the scan stamp. The published `frame_id` is still `"odom"` (so the `sensor_frame` fix is still required and still correct), **but the ~11% rotation phantom-arc may not actually shrink.** This must be validated on a rotation bag — see §8. Do not write "per-packet de-skew" into the config comment as if it were proven.

---

## 0. ⚠ CRITICAL CORRECTION — `fixed_frame: "odom"` does NOT de-skew (source-verified, supersedes C1 + §8-step-1)

After the workflow, the `configure()` arg-swap (claim L1) was re-read **directly from upstream source** and confirmed line-by-line. Following it through changes the recommended config: the headline "set `fixed_frame:"odom"`" (C1) is **wrong on its own** — it republishes the cloud into `odom` per-scan but runs **no de-skew at all**.

**The swap, verified line-by-line** (`ros-drivers/velodyne`, `ros2` branch):
- `transform.cpp:138` calls `configure(min, max, target_frame, fixed_frame)`
- `datacontainerbase.hpp:189-196` declares `configure(min, max, fixed_frame, target_frame)` and assigns `config_.fixed_frame = arg3`, `config_.target_frame = arg4`
- → args 3/4 are swapped; `configure()` runs *after* the (correct) constructor and wins.

**Net effect of the intuitive `fixed_frame:"odom"`, `target_frame:""`:** post-swap `config_.fixed_frame=""`, `config_.target_frame="odom"`. Then:
- `computeTransformToFixed()` early-returns (`fixed_frame.empty()`, `datacontainerbase.hpp:245`) → **per-packet de-skew never runs.**
- `computeTransformToTarget()` runs once at the scan stamp (`:234`) → a single rigid `velodyne→odom` transform.
- `finishCloud()` sets `frame_id="odom"` via the target branch (`:151-152`).

So `fixed_frame:"odom"` = "republish the whole scan into odom, no motion compensation" — strictly worse than de-skew OFF (you take on the frame-republish problem for zero de-skew). It also means **de-skew was never actually working** when `fixed_frame:"odom"` was enabled on 05-29; the phantom arc was never being compensated by that knob.

**To get REAL per-packet de-skew you write the params "backwards" (because of the swap), and you keep the output in the sensor frame so NO costmap/STVL change is needed.** The installed `ros-humble-velodyne` apt binary may or may not carry the swap, so run this **2-minute Jetson characterization FIRST:**

1. Set `velodyne.yaml`: `fixed_frame: "odom"`, `target_frame: "velodyne"`. Relaunch the transform node.
2. `ros2 topic echo --once /velodyne_points --field header.frame_id`

| frame_id printed | binary | use this config (`velodyne.yaml` only) | result |
|---|---|---|---|
| `velodyne` | **no swap** (patched) | keep `fixed_frame:"odom"` + `target_frame:"velodyne"` | de-skew ON, output `velodyne` → **no STVL change** |
| `odom` | **swap present** | flip to `fixed_frame:"velodyne"` + `target_frame:"odom"` | de-skew ON, output `velodyne` → **no STVL change** |

Either branch ends at: **per-packet de-skew + output frame stays `velodyne` → STVL clears robot-centric natively → the `sensor_frame` edits (C2/C3/C4) become UNNECESSARY.** The mechanism: each point is motion-compensated `velodyne(t)→odom` per packet, then the assembled cloud is rigidly mapped `odom→velodyne` at the scan stamp (`addPoint` applies `to_fixed` then `to_target`, `datacontainerbase.hpp:267-270`) — de-skew preserved, frame back to `velodyne`.

The `sensor_frame:"velodyne"` route (C2/C3) remains valid **only** if you deliberately output the cloud in `odom` (e.g. `target_frame:"odom"` alone) — not recommended, since the velodyne-output form is cleaner and touches one file. If neither config yields de-skew on the binary (e.g. it ignores per-packet stamps), fall back to a dedicated twist-based de-skew node that outputs `velodyne` and bypasses this bug entirely.

**Bottom line of §0:** the change is **one file (`velodyne.yaml`), not three**, and the exact two-line value is decided by the wire test above. Everything below (C1–C6) is the pre-correction analysis — read it for the verification trail, but apply §0.

---

## 1. Verified change set

Categories: **necessary** (fix breaks without it), **standard** (idiomatic hardening), **optional** (only under certain configs), **avoid** (do not do).

| # | file | exact from → to | category | relies on | verification |
|---|------|-----------------|----------|-----------|--------------|
| C1 | `src/avros_bringup/config/velodyne.yaml:28` | **from:** `    fixed_frame: ""                # 2026-05-30: DESKEW OFF (was "odom", #21). The deskew published the cloud in the odom frame ...` **→ to:** `    fixed_frame: "odom"           # DESKEW ON. Output cloud frame_id becomes "odom" -> STVL velodyne_points sources MUST set sensor_frame: "velodyne" (local+global). Keep cut_angle 2pi (line 14) + organize_cloud true. NOTE: on the installed velodyne binary a configure() arg-swap routes "odom" into target_frame, so this is a single per-SCAN transform-to-odom, NOT per-packet computeTransformToFixed de-skew — verify phantom-arc actually shrinks on a rotation bag.` | necessary | L1 | CONFIRMED (frame_id→odom). Per-packet de-skew claim **not** confirmed — verify on Jetson. |
| C2 | `nav2_params_humble.yaml` local STVL, insert after line 300 `          topic: /velodyne_points` at 10-space indent | **from:** `          topic: /velodyne_points`<br>`          data_type: PointCloud2` **→ to:** `          topic: /velodyne_points`<br>`          sensor_frame: "velodyne"   # de-skew publishes frame_id=odom; without this STVL anchors clearing origin at odom (0,0,0) not the robot. measurement_buffer.cpp:97-98.`<br>`          data_type: PointCloud2` | necessary | L2, L3, L4, L5 | CONFIRMED (3/3 lenses, byte-identical upstream) |
| C3 | `nav2_params_humble.yaml` global STVL, insert after line 551 `          topic: /velodyne_points` at 10-space indent | **from:** `          topic: /velodyne_points`<br>`          data_type: PointCloud2` **→ to:** `          topic: /velodyne_points`<br>`          sensor_frame: "velodyne"   # global_frame=map; without this, origin = map-position of odom origin (the EKF map->odom offset). sensor_frame -> velodyne->map = true robot-in-map.`<br>`          data_type: PointCloud2` | necessary | L2, L3, L4, L7 | CONFIRMED. Needs `map→velodyne` TF (present). |
| C4 | `nav2_params_humble.yaml:304-307` (stale comment inside the C2 insertion block) | **from:** `          # ... confirmed in nav2_costmap_2d Humble source:`<br>`          # ObservationBuffer::bufferCloud transforms cloud to global_frame_`<br>`          # BEFORE the z check (lines 159-187 in observation_buffer.cpp).` **→ to:** cite the **active** plugin: `# STVL MeasurementBuffer::BufferROSCloud transforms cloud to _global_frame via cloud.header.frame_id BEFORE the z check (measurement_buffer.cpp:142-161). sensor_frame only sets the clearing ORIGIN, not the z-filter.` | standard | L5 | Comment cites wrong file; STVL is the active plugin. (Completeness critic) |
| C5 | `nav2_params_humble.yaml` global_costmap `ros__parameters` (block 490-528 has **no** `transform_tolerance`) | **add:** `      transform_tolerance: 0.5     # match local costmap (:253); default 0.3 is thin for map<-odom under RTK FIX/FLOAT steps` | standard | L7 | Local sets 0.5 (`:253`); global silently inherits 0.3. Robustness hedge. |
| C6 | `nav2_params.yaml` (fallback) local `:136` + global velodyne source | **from:** `          topic: /velodyne_points` **→ to:** `          topic: /velodyne_points`<br>`          sensor_frame: "velodyne"   # only if fallback config is ever run WITH de-skew` | optional | L8 | Fallback uses stock VoxelLayer/ObstacleLayer, same origin bug, but NOT the production path. Also see §5 height caveat. |
| C7 | `nav2_params_humble.yaml:340` local `decay_acceleration` / `:286` local `voxel_decay` (3.0) | **leave unchanged** — do NOT auto-revert local `voxel_decay` to 8.0; do NOT set `mapping_mode: true`; do NOT change `mark_threshold:1` | avoid | L4, C6-evidence | Slow clearing was the origin bug (root-fixed by C2/C3), not the decay value. |
| C8 (rejected) | `velodyne.yaml` add `target_frame: "velodyne"` | **DO NOT apply alongside C2/C3** | avoid | L1 | `configure()` arg-swap can invert fixed/target at runtime; `sensor_frame` route is safer. |

**Total production edits: C1 + C2 + C3 (necessary), C4 + C5 (standard) = 3 sites in 2 files, +2 hardening edits.** C6 is fallback-only; C7/C8 are "do not".

---

## 2. Load-bearing claims & adversarial verdicts

Eight load-bearing claims (L1–L8) were each run through three independent lenses (upstream-source, codebase-contract, official-docs). **All eight CONFIRMED. None refuted.** The single correction is on L1's *mechanism* (not its conclusion) and it is material — do not hide it.

| id | claim (abbrev) | status | 3-lens tally | decisive citation |
|----|----------------|--------|--------------|-------------------|
| L1 | `fixed_frame:"odom"` + target empty → published `frame_id` = `"odom"` (not velodyne) | **CONFIRMED** (conclusion) — **mechanism CORRECTED** | 3 confirmed / 0 refuted | [`datacontainerbase.hpp:151-157`](https://raw.githubusercontent.com/ros-drivers/velodyne/ros2/velodyne_pointcloud/include/velodyne_pointcloud/datacontainerbase.hpp) (finishCloud priority target>fixed>sensor); swap at [`transform.cpp`](https://github.com/ros-drivers/velodyne/blob/ros2/velodyne_pointcloud/src/conversions/transform.cpp) `configure(min,max,target_frame,fixed_frame)` vs signature `(…,fixed_frame,target_frame)` |
| L2 | STVL origin = (0,0,0) of `origin_frame` → global, where `origin_frame = sensor_frame ?: cloud.header.frame_id` | **CONFIRMED** | 3 / 0 | [`measurement_buffer.cpp:97-98`](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/humble/spatio_temporal_voxel_layer/src/measurement_buffer.cpp) |
| L3 | STVL declares & reads per-source `sensor_frame`, default `""` (recognized knob) | **CONFIRMED** | 3 / 0 | [`spatio_temporal_voxel_layer.cpp:182`](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/humble/spatio_temporal_voxel_layer/src/spatio_temporal_voxel_layer.cpp#L182), read `:208`, passed `:270` |
| L4 | The single `_origin` feeds BOTH marking range-gate AND frustum decay (one edit fixes both) | **CONFIRMED** | 3 / 0 | `spatio_temporal_voxel_grid.cpp:286-288` (mark dist) + `:140` (`frustum->SetPosition(it->_origin)`) |
| L5 | Marking is unaffected by `sensor_frame`: points always transform via `cloud.header.frame_id`; z-filter runs in global frame | **CONFIRMED** | 3 / 0 | `measurement_buffer.cpp:142-146` (point transform via header) + `:153-174` (z-filter on `cld_global`) |
| L6 | No src/ node subscribes `/velodyne_points` except Nav2 costmap layers; RViz/Foxglove/sim are header-driven | **CONFIRMED** | 2 / 0 | `perception_node.py:241-242` (ZED only); `avros.rviz:28` / `costmap_test.rviz:29` `Use Fixed Frame: true`; `avros_webots.urdf:21-22` (sim self-publishes) |
| L7 | All required TF chains published: `odom→base_link`, `base_link→velodyne` static, `map→odom` (navsat broadcast off, no loop) | **CONFIRMED** | 2 / 0 | `ekf.yaml:26,125` (`publish_tf:true`), `navsat.yaml:41` (`broadcast_cartesian_transform:false`), `avros.urdf.xacro:148-154` |
| L8 | Fallback `nav2_params.yaml` uses stock VoxelLayer/ObstacleLayer (not STVL); revert `b122e22` didn't touch it → fix contained to 2 production files | **CONFIRMED** | 3 / 0 | `nav2_params.yaml:127` (VoxelLayer), `:253` (ObstacleLayer); `git show b122e22 --name-only` = humble.yaml + velodyne.yaml only |

### The one correction that matters — L1 mechanism

**Conclusion (`frame_id` → `"odom"`): CONFIRMED by all 3 lenses.** The `sensor_frame` fix is therefore necessary and correct regardless.

**Mechanism: the team's belief is WRONG and the config comment must not restate it.** Upstream `velodyne/transform.cpp` calls `configure(min_range, max_range, target_frame, fixed_frame)` but the `DataContainerBase::configure()` signature is `(min_range, max_range, fixed_frame, target_frame)` — arguments 3 and 4 are swapped. `configure()` runs *after* the (correct) constructor and has the final word. Net runtime state with our YAML (`fixed_frame:"odom"`, `target_frame:""`):

- `config_.target_frame = "odom"`, `config_.fixed_frame = ""`.
- `finishCloud()` checks `target_frame` first → non-empty → `frame_id = "odom"` (claim conclusion holds via the *target* branch, not the *fixed* branch).
- `computeTransformToFixed()` early-returns (config_.fixed_frame is empty) → **no per-packet ego-motion compensation.** Instead `computeTransformToTarget()` does a single transform at the scan stamp.

**How this changes the change set:** it does NOT change C1/C2/C3 (still required, still correct). It DOES change **C1's comment** (do not claim "per-packet de-skew (computeTransformToFixed)") and it adds a **mandatory field check**: confirm on a rotation bag that the phantom-arc actually shrinks. If it does not, the team's stated motivation (de-skew the sweep) is unmet and a different de-skew path (true per-point/twist deskew node) would be needed — but that is a separate effort, not this fix.

Confidence note: the swap was read on the `ros2` branch / installed Jazzy headers (byte-identical to Humble 2.5.1 per rosdistro mapping). The production Jetson runs `ros-humble-velodyne` (apt). **Verify on Jetson** with the wire check in §8.

---

## 3. Standard vs works — is each change idiomatic?

| change | idiomatic Nav2/STVL? | evidence |
|--------|----------------------|----------|
| `sensor_frame: "velodyne"` per source (C2/C3) | **Standard, documented param** (though undocumented in the STVL README, it is the same name/semantics as stock Nav2 `sensor_frame`: "The frame of the origin of the sensor. Leave empty to read from sensor data" — [costmap_2d obstacles ref](https://devel.iri.upc.edu/docs/roswiki/wiki/costmap_2d(2f)hydro(2f)obstacles.html)). It is the documented escape hatch *for republished clouds.* | declared `spatio_temporal_voxel_layer.cpp:182`; stock parity `nav2_costmap_2d/plugins/obstacle_layer.cpp` + `observation_buffer.cpp:62-63` |
| `fixed_frame:"odom"` de-skew (C1) | **First-party, officially-supported** velodyne de-skew method (`velodyne_transform_node`, not `convert_node`). | [velodyne issue #342](https://github.com/ros-drivers/velodyne/issues/342); docs.ros.org velodyne_pointcloud |
| `target_frame:"velodyne"` (C8, **rejected**) | **More idiomatic single-point fix** in theory (one velodyne.yaml edit fixes local STVL + global STVL + fallback + sim simultaneously, never lies about the cloud's true frame — exactly what `target_frame` is documented for). **Rejected here** because the `configure()` arg-swap can silently invert fixed/target on this path, and the binary re-projection behavior was not runtime-verified. | velodyne issue #342; L1 swap finding |

**Recommendation:** Use `sensor_frame` (C2/C3). It is standard, robust to the arg-swap, and verified. The "more standard" `target_frame` alternative is real but blocked by the upstream bug until proven safe on the Jetson wire — if a future engineer confirms `target_frame:"velodyne"` yields `frame_id=="velodyne"` on the installed binary, it is the cleaner single-file fix and C2/C3/C6 all become unnecessary. **Never apply both routes.**

---

## 4. What breaks / frame-contract analysis

**Exhaustive `/velodyne_points` consumer list (verified L6):**

| consumer | cares about frame_id? | breaks on velodyne→odom? |
|----------|----------------------|--------------------------|
| Local STVL `velodyne_points` (`nav2_params_humble.yaml:299`) | YES (clearing origin) | Fixed by C2 |
| Global STVL `velodyne_points` (`nav2_params_humble.yaml:550`) | YES (clearing origin) | Fixed by C3 |
| Fallback VoxelLayer (`nav2_params.yaml:136`) / ObstacleLayer (`:256`) | YES (same bug) | Not production path; C6 if ever used |
| `perception_node` | NO — subscribes ZED RGB+cloud only (`perception_node.py:241-242`) | No |
| `semantic_segmentation_layer` | NO — subscribes `/perception/*` | No |
| RViz `avros.rviz` / `costmap_test.rviz` PointCloud2 | header-driven (`Use Fixed Frame: true`) | No — renders via TF |
| Foxglove `docs/foxglove_layout.json` | cosmetic frame toggle only | No |
| Sim (`avros_webots.urdf:21-22`) | publishes its OWN cloud `frameName:velodyne`, never runs `velodyne_transform_node` | No effect — de-skew config is inert in sim |
| `robojackets/` | out of `src/`, ROS1 catkin, git-untracked | Not in workspace |

**Nothing outside the Nav2 costmap layers breaks.** Sim is inert to the change and `sensor_frame:"velodyne"` agrees with the sim cloud's own frame, so it is sim-safe.

**Height-filter frame note (L5):** `min/max_obstacle_height` is applied in the **global frame**, after the cloud is transformed via `cloud.header.frame_id` (`measurement_buffer.cpp:142-161`). `sensor_frame` only moves the clearing ORIGIN — it does not touch the point transform or the z-filter. So the production Humble values `0.4 / 0.8` (already global-frame-correct) are unaffected by de-skew. **The fallback's `min_obstacle_height: -0.8` (`nav2_params.yaml:139`) is sensor-frame-referenced ("Sensor at z=1.0m; -0.8 = 0.2m above ground") and would be WRONG under de-skew** — another reason the fallback (C6) is not de-skew-ready without re-derivation. The stale comment at `nav2_params_humble.yaml:304-307` cites the wrong file (`observation_buffer.cpp`) — fix via C4.

---

## 5. TF / timing / RTK-jump preconditions

All TF chains exist and are singly-published (L7):

| dependency | chain | publisher |
|------------|-------|-----------|
| De-skew lookup `odom←velodyne` (per packet/scan) | `odom→base_link` + `base_link→velodyne` | odom EKF (`ekf.yaml:26` `publish_tf:true`) + static URDF (`avros.urdf.xacro:148-154`) |
| Local STVL origin `odom←velodyne` | same as above | same |
| Global STVL origin `map←velodyne` + cloud marking `map←odom` | `map→odom` + `odom→base_link` + `base_link→velodyne` | map EKF (`ekf.yaml:125` `publish_tf:true`); navsat does NOT also broadcast (`navsat.yaml:41` `broadcast_cartesian_transform:false`) → no loop |

**transform_tolerance:** local costmap sets `0.5` (`:253`); **global costmap has none → silent Nav2 default 0.3s** — promote to `0.5` via C5. STVL's own origin `canTransform` uses a hard-coded `0.5s` window and the cloud lookup is buffered, so the fix works at current values; C5 is a hardening hedge for the broader marking/aging pipeline.

**Risks:**
- **Startup TF race:** `velodyne_transform_node` and both EKFs launch concurrently (`localization.launch.py`, no `TimerAction` ordering). For ~1-2s after launch, `odom←velodyne` is unavailable → transient "TF Exception" + dropped early scans → self-clears. Benign; a small `TimerAction` on the velodyne nodes would silence it (optional, see §7).
- **RTK-jump (global only):** the de-skewed cloud is marked into the map frame via `map←odom`; under an RTK FIX↔FLOAT step `map→odom` can jump/lag, causing intermittent dropped global marks. **This is NOT new** — the cloud is already transformed into map for marking today; `sensor_frame:"velodyne"` only corrects the origin and adds no new map-jump sensitivity. Watch global-costmap clearing under degraded GPS.
- **Scan drops:** de-skew adds the per-scan `odom←velodyne` lookup back. The historical 1-scan/~6s extrapolation drop was cured by `cut_angle=2pi` (still set, `velodyne.yaml:14`). Watch `diag_min_freq` after re-enabling.

---

## 6. Completeness — missing changes & standard-practice gaps

From the completeness critic (folded into the change set above):

- **C4 (now included):** the pre-existing comment at `nav2_params_humble.yaml:304-307` cites `observation_buffer.cpp` (stock nav2), but the active plugin is STVL → authoritative file is `measurement_buffer.cpp:142-161`. Update it or the next reader is misled about which buffer governs the z-check and origin ternary.
- **C5 (now included):** global costmap lacks an explicit `transform_tolerance` (inherits 0.3s vs local's 0.5s) — asymmetry directly relevant to the de-skewed-cloud-in-map path.
- **C1 comment (corrected):** must NOT assert per-packet `computeTransformToFixed` de-skew (L1 swap finding).
- **Fallback height (C6 note):** `min_obstacle_height: -0.8` (`nav2_params.yaml:139, 259`) is sensor-frame-referenced and breaks under de-skew — `sensor_frame` alone is insufficient for the fallback. Documented, not auto-edited (fallback is not the production path).
- **Global decay context (C7):** `b122e22` changed only **local** `voxel_decay` 8.0→3.0; **global** `voxel_decay` is 5.0 (`:538`, untouched). Leave both as-is; the origin fix is the root cause, not decay. Decision documented, not auto-reverted.
- **Startup TimerAction (optional):** `localization.launch.py` has no ordering between velodyne nodes and EKFs. Optional mitigation; benign without it.

**Standard-practice gap acknowledged:** the most idiomatic Velodyne fix is the single-file `target_frame:"velodyne"`. We deliberately deviate to `sensor_frame` because of the `configure()` arg-swap. This is a justified deviation, but it means all sources consuming the de-skewed topic must be kept in sync (C2 + C3, and C6 if the fallback is ever used).

---

## 7. Field verification plan (on-Jetson A/B)

Run after applying C1+C2+C3 (+C4/C5), rebuilding, and relaunching `navigation.launch.py`. **Use laptop Foxglove, NOT RViz on the Jetson** (RViz over NoMachine starves MPPI — see project memory).

1. **Wire check (gating — resolves the L1 caveat):**
   `ros2 topic echo --once /velodyne_points --field header.frame_id`
   → MUST print `odom`. If it prints `velodyne`, the arg-swap inverted the intent — stop and reassess.
2. **Per-packet de-skew actually happening? (the L1 mechanism question):**
   Drive a slow **in-place rotation** and watch the local costmap in Foxglove. The ~11% rotation phantom-arc on the trailing edge should **shrink/disappear**. If it does NOT shrink, the arg-swap means you got per-scan transform-to-odom, not per-packet de-skew — record this; the fix still corrects clearing but the de-skew goal is unmet.
3. **Frustum clearing restored (the actual fix):** place a barrel in front, confirm it MARKS, then remove it. With `sensor_frame:"velodyne"` it should clear in **~2-3s** (frustum-accelerated decay from the robot), not the slow ~8s `voxel_decay`-only path. Repeat for BOTH costmaps (local odom + global map).
4. **Marking still works:** confirm the barrel marks LETHAL at the right place in both costmaps before removal (rules out a sensor_frame-broke-marking regression — L5 says it cannot, but verify).
5. **Control-loop health:** `ros2 topic hz /cmd_vel` (should hold ~20 Hz); `ros2 topic hz /velodyne_points` (should hold ~10 Hz — confirms no scan drops from the re-added TF lookup, `diag_min_freq` 2Hz floor).
6. **Degraded-GPS check (if time):** with RTK in FLOAT/SBAS, confirm the global costmap still marks/clears and isn't throwing repeated `map→odom` TF exceptions.

---

## 8. Bottom line — ordered apply steps + device confirmations

**Apply (in order) — CORRECTED per §0:**
1. **Run the §0 Jetson characterization test first** (`fixed_frame:"odom"` + `target_frame:"velodyne"`, echo frame_id). This decides the one-file config and replaces the old steps 1–3 below. If it yields de-skew + `frame_id=="velodyne"`, you are DONE in `velodyne.yaml` — **skip the C2/C3 `sensor_frame` edits entirely** (the cloud never leaves the velodyne frame, so STVL clearing is already robot-centric).
2. **Only if you choose the odom-output form** (`target_frame:"odom"` alone): then add `sensor_frame: "velodyne"` to local STVL (`:300`, C2) and global STVL (`:551`, C3). Otherwise skip.
3. ~~global STVL sensor_frame~~ — see step 2; unnecessary in the recommended velodyne-output config.
4. `nav2_params_humble.yaml:304-307` — fix the stale `observation_buffer.cpp` comment to cite `measurement_buffer.cpp` (C4).
5. `nav2_params_humble.yaml` global_costmap `ros__parameters` — add `transform_tolerance: 0.5` (C5).
6. Do **NOT** add `target_frame` (C8). Do **NOT** revert local `voxel_decay` to 8.0, set `mapping_mode:true`, or touch `mark_threshold` (C7). Leave `nav2_params.yaml` (fallback) untouched unless you actually run it with de-skew (then C6 + re-derive `min_obstacle_height`).
7. `colcon build --symlink-install`, relaunch.

**Confirm on device before trusting it (these could NOT be resolved off-Jetson):**
- [ ] **`/velodyne_points` header.frame_id == `odom`** on the wire (gates everything; resolves the arg-swap ambiguity).
- [ ] **Phantom-arc actually shrinks on in-place rotation** — this is the open L1 question. The installed binary may do per-scan, not per-packet, de-skew. If the arc does not shrink, the de-skew goal is unmet (the clearing fix still holds).
- [ ] **Removed obstacle clears in ~3s in BOTH costmaps** (frustum working from the robot origin).
- [ ] **Obstacle still MARKS** before removal in both costmaps.
- [ ] **`/cmd_vel` ~20 Hz and `/velodyne_points` ~10 Hz** hold (no scan drops, no loop starvation).
- [ ] **Jetson's built STVL still has the `measurement_buffer.cpp:97-98` ternary and the per-source `sensor_frame` declare** — verified against upstream `humble` source, but the Jetson build was not inspected; if STVL is pinned to a fork, re-confirm (a silently-ignored key makes C2/C3 no-ops).

**Confidence:** The clearing-origin fix (C2/C3) is high-confidence CORRECT and STANDARD — confirmed by 3 adversarial lenses on byte-identical upstream source. The single residual unknown is whether `fixed_frame:"odom"` delivers genuine per-packet de-skew on the installed apt binary (the `configure()` arg-swap says no); that is a Jetson-only determination and does not affect the correctness of the clearing fix, only whether the team's de-skew *motivation* is satisfied.
