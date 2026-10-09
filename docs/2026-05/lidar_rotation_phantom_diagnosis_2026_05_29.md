# LiDAR Rotation Phantom-Obstacle Diagnosis (2026-05-29)

> Multi-agent diagnostic workflow (6 probes + adversarial verification + synthesis, 13 agents).
> Symptom captured via RViz during the 2026-05-29 phase0 field run: during in-place rotation
> while passing physically close objects, the VLP-16 paints a ring/arc of phantom lethal cells
> in the local costmap, confusing MPPI. Intermittent.

# Rotation Phantom Obstacles: Root-Cause Report

**Symptom:** During in-place rotation while passing physically close objects, the VLP-16 paints a ring/smeared arc of phantom lethal cells around the robot in the local costmap, confusing MPPI. Intermittent.

---

## 1. Root-cause ranking

### PRIMARY (the actual driver of the symptom) — a two-part mechanism that is one causal chain

**P1. No point-cloud motion de-skew + STVL max-persistence decay = an accumulating stale arc.**

This is the root cause. It decomposes into a *trigger* and an *amplifier* that are physically inseparable for this symptom:

- **Trigger — intra-sweep skew (no de-skew anywhere in the pipeline).** Confirmed: the pipeline is `velodyne_driver_node → velodyne_transform_node → /velodyne_points → STVL` with nothing in between. `velodyne_transform_node` runs with `fixed_frame` unset (it is *not* in `velodyne.yaml`), so the entire revolution is stamped once and placed into `odom` with a single TF lookup. Points captured early vs. late in a revolution are mis-rotated by `ω · T_sweep`. The measured publish rate is **19.7 Hz** (`phase0_field_test_2026_05_29.md:23`) — so `T_sweep ≈ 50 ms`, giving `ω·0.05 = 0.05 rad ≈ 2.9°` of azimuth smear per scan at the actuator's max ~1.0 rad/s (and the MPPI `wz_max` is even higher). At zero ω the skew is exactly zero — which is precisely why the symptom is rotation-only.

- **Amplifier — STVL persistence (`voxel_decay: 8.0`, `decay_acceleration: 2.0`, `mark_threshold: 0`).** Confirmed at `nav2_params_humble.yaml:281, 289, 335`. The velodyne source has **no raytrace free-space clearing** (by deliberate design — comment at lines 264–275); clearing is *only* time-decay (8 s linear) + frustum-accelerated decay. Each skewed scan lays a slightly-displaced arc; with `mark_threshold: 0` a single stray return permanently marks a 10 cm voxel; that voxel then survives ~2.5–8 s — i.e. **30–120 MPPI cycles** (loop period 60–75 ms; costmap update measured as low as 2.3 Hz under load). Successive scans during a rotation never re-confirm the previous arc in the *same* place, so displaced arcs accumulate into the observed ring instead of being overwritten. The MPPI controller therefore sees a cost field that matches where surfaces *were* up to several seconds ago, not where they are.

The two halves are documented as a known tension: the persistence values were chosen specifically to suppress **stationary** flicker, and that exact persistence is what retains the rotation smear. Neither half alone produces the symptom — de-skew error without persistence would self-correct in one scan; persistence without rotation-induced displacement produces no arc.

### CONTRIBUTING FACTORS (real, secondary, geometrically gate the symptom toward *close* objects)

**C1. Close objects subtend a large azimuth arc → the fixed angular smear paints a *thick* arc near the robot.** A 0.3 m object at 0.7 m subtends ~24°; the same object at 5 m subtends ~3.4°. The per-scan smear (`~2.9°`) overlaps many more 10 cm voxels at short range, and arc-length error stacks. This is the dominant reason the symptom is *close-object-specific*.

**C2. Forward LiDAR lever arm (0.089 m) → short-range parallax.** Confirmed `avros.urdf.xacro:153` (`xyz="0.089 0.0 …"`). During in-place rotation the sensor optical center traces a 0.089 m circle, so a close object is seen from a *moving* origin within one sweep. Parallax ≈ translation/range → significant at <1 m, negligible far away. Reinforces C1's close-object specificity.

**C3. Close/low voxels fall at the near-frustum edge → slow timeout, not fast frustum decay.** Sensor height = 0.5556 + 0.159 = **0.715 m**; the −15° beam hits ground at 0.715/tan15° ≈ 2.67 m. Objects closer than ~2.67 m have low returns that sit near/below the modeled STVL frustum cone, so even when re-pointed-at they clear via the slow 8 s timeout rather than fast `decay_acceleration`. `vertical_fov_padding: 0.1` (line 325) was a timid attempt to widen this; it is too small.

**C4. CPU-load-dependent costmap update rate (2.3–10 Hz).** Confirmed `phase0:91,98`. Bursty consumption stretches the effective inter-scan ego-yaw (wider displaced arc per consumed cloud) and applies decay in coarse irregular steps (effective persistence > nominal). This is the primary explanation for **intermittency under load**.

### RULED OUT (do not act on these)

- **"EKF yaw lags true rotation" / 13% ω under-delivery / Xsens stuck-bias as the cause — REFUTED.** The repo's own data refutes it: `yaw_diag_session_2026_05_27/VERDICT.md` shows the EKF tracks IMU yaw with gyro bias 0.008°/s; the 2026-05-28 rotation test measured **105–106% ω delivery**, not 13% under-delivery. The 13% is *commanded-vs-delivered*, not *EKF-vs-truth* — it injects **zero** odom-frame bearing error. Do **not** retune `ekf.yaml`. (One operational caveat survives: an *unsettled Xsens quaternion* in the first ~150 s after boot can transiently mis-place marks — addressed by a warmup rotation, not a config change.)

- **`transform_tolerance: 0.5` as a primary cause — REFUTED.** It is a *ceiling*, not realized staleness. `odom→base_link` is published at 30 Hz (`ekf.yaml:24`), tf2 interpolates a fresh transform at the cloud stamp, and the phase0 SVGA run recorded **0 TF exceptions** on the local costmap. 0.5 s only bites during a genuine TF gap. **Do not raise it; do not blindly lower it** (it exists for the semantic layer's ZED stamp lag — Issue #18).

- **"STVL has no clearing path at all" framing — REFUTED.** `decay_acceleration` *is* STVL's frustum-based active-clearing mechanism (the STVL equivalent of raytrace). The path exists and is active; it was deliberately throttled (10→5→2). The true root is the *tuning*, not a structural absence. **Do not switch the LiDAR source back to nav2 `VoxelLayer`** — that re-introduces the documented broken raytrace-can't-clear-sparse-VLP-16 failure mode (lines 264–275).

- **`min_range 0.7` self-detection (payload re-crossing the gate during rotation) — LOW probability.** Verified at 0 cells when stationary; the hard threshold has no hysteresis so it *could* flicker, but this is a distinct near-body mechanism, not the external-object ring. Cheap to rule out empirically (open-area rotation test, §5).

---

## 2. Why it's intermittent and close-object-specific

**One unifying explanation:** the phantom ring requires the **simultaneous product of four factors**, each of which can be ~zero:

```
phantom_ring ∝ ω (rotation rate)
             × 1/range (close object: large subtended arc + parallax + low-frustum-edge)
             × persistence_window_still_open (voxel_decay 8 s / decay_accel 2)
             × inter-scan_yaw_gap (worse under CPU load → bursty consumption)
```

- Drive straight → `ω ≈ 0` → no smear.
- Rotate in open space → no close object → arc collapses (far objects smear less per C1/C2 and self-clear).
- Rotate slowly → small `ω·T_sweep` → thin arc within one voxel, no ring.
- Rotate past a close object **under CPU load** → all four terms large → full ring.

Close-object specificity comes from C1 (angular size) + C2 (parallax) + C3 (near-frustum slow decay) all scaling as 1/range. Intermittency comes from C4 (load-dependent update rate) and the multiplicative gate — most maneuvers zero out at least one factor.

---

## 3. The core tension and how to resolve BOTH at once

**The tension is real:** `voxel_decay: 8.0` + `decay_acceleration: 2.0` + `mark_threshold: 0` were tuned to maximize persistence and stop **stationary flicker** (~40% lethal-cell sawtooth). Persistence is exactly what leaves the **rotation trail**. Naively lowering persistence to fix rotation re-opens stationary flicker; raising it to fix flicker worsens rotation.

**The resolution: the flicker and the smear had a *common* upstream cause that is now partly fixed, and the right levers decouple the two.**

1. **The stationary flicker was largely a CPU-starvation artifact** — STVL consumed LiDAR in bursts (load 10.75; semantic-layer starvation). That root cause is now mitigated (ZED → SVGA dropped load 9.84→2.96; `point_cloud_res: REDUCED`; cmd_vel back to 13–16 Hz). So the *extreme* persistence (8 s) is no longer the only thing holding obstacles up.

2. **`mark_threshold: 0 → 1` attacks the flicker root without touching decay.** A real barrel/person returns many points and trivially clears a threshold of 1; a single stray rotation-transient return no longer marks at all. This removes a large fraction of *both* the stationary sawtooth (single-hit dropouts) and the rotation ghosts (single-hit smear) — it is the one lever that helps both problems in the same direction.

3. **`decay_acceleration` is the rotation lever; `voxel_decay` is the flicker backstop.** Raise `decay_acceleration` so in-frustum vacated voxels (the rotation trail — it stays inside the 360° horizontal FOV) clear in <1 s, and *moderately* lower `voxel_decay` so even any out-of-frustum stale mark expires within one rotation. Inflation (`inflation_radius`) provides the cost-field backstop that persistence used to provide.

4. **The real cure removes the tension entirely: de-skew.** If successive sweeps *agree* (de-skewed), there is no displaced arc to accumulate, so you can keep persistence high for anti-flicker AND have crisp rotation behavior. De-skew is the only fix that doesn't trade one failure for the other.

**Recommended posture:** ship the low-risk config decoupling now (3a/3b above) to make the field testable; land de-skew as the structural cure so persistence can stay high.

---

## 4. Recommended fix plan (ordered)

All paths relative to repo root `/home/mspacman/IGVC_ROS2/`. Apply config changes as a group and validate together.

### Tier 1 — Low-risk config (do first, test as a set)

**Fix 1.1 — Raise `mark_threshold` 0 → 1** *(highest value/effort ratio; helps flicker AND smear)*
- File: `src/avros_bringup/config/nav2_params_humble.yaml`, `local_costmap.stvl_layer.mark_threshold`
- New value: `mark_threshold: 1`
- Could break: real thin/sparse obstacles seen by only one beam-return per voxel (e.g. a thin pole at long range) may need a second hit before marking. Mitigated because IGVC obstacles (barrels, cones, people) are dense. Validate barrels still mark.

**Fix 1.2 — Raise `decay_acceleration` 2.0 → 6.0** *(rotation lever)*
- File: same, `local_costmap.stvl_layer.velodyne_points.decay_acceleration`
- New value: `6.0`
- Could break: re-introduces some stationary flicker *if* CPU starvation returns. Backstopped by Fix 1.1 (single-hit ghosts gone) and inflation. If flicker returns, it is a *signal* that CPU starvation is back — fix load, don't re-throttle decay.

**Fix 1.3 — Lower `voxel_decay` 8.0 → 3.0** *(flicker backstop, not the primary lever)*
- File: same, `local_costmap.stvl_layer.voxel_decay`
- New value: `3.0`
- Could break: faster timeout for genuinely-occluded static obstacles the sensor can't currently re-confirm. 3 s is still 40+ MPPI cycles of persistence — ample. Do **not** drop below ~2 s.

**Fix 1.4 — Widen `vertical_fov_padding` 0.1 → 0.35** *(let close/low voxels get fast frustum decay)*
- File: same, `local_costmap.stvl_layer.velodyne_points.vertical_fov_padding`
- New value: `0.35`
- Could break: voxels slightly outside the true beam cone become eligible for frustum decay, marginally faster clearing of legitimate low obstacles. Low risk; directly targets C3.

**Fix 1.5 — Set `expected_update_rate` on the velodyne source** *(flag stale sensor data instead of silently persisting it during a stall)*
- File: same, under `local_costmap.stvl_layer.velodyne_points`
- Add: `expected_update_rate: 0.2` (seconds; ~5 Hz floor, tolerant of the 19.7 Hz source dropping under load)
- Could break: if set too tight, transient stalls log warnings. 0.2 s is loose enough.

**Do NOT touch in Tier 1:** `transform_tolerance` (leave 0.5 — it serves the semantic layer), `ekf.yaml` (refuted), the choice of STVL vs VoxelLayer (keep STVL).

### Tier 2 — Higher-effort code/launch (the structural cure)

**Fix 2.1 — Enable driver-level ego-motion de-skew (cheapest de-skew, no new node)** *(test this first in Tier 2)*
- File: `src/avros_bringup/config/velodyne.yaml`, under `velodyne_transform_node.ros__parameters`
- Add: `fixed_frame: "odom"` (and `organize_cloud: true` stays). This makes `velodyne_transform_node`'s `computeTransformToFixed()` do a per-packet TF lookup so each packet is motion-corrected before being expressed in the output frame.
- Requires: low-latency `odom` TF at packet times — satisfied (odom EKF at 30 Hz).
- Could break: (a) adds per-packet TF lookups → slightly more CPU in the driver (small, packet-rate not point-rate); (b) if `odom` TF ever has a gap the packet transform fails — but the costmap already tolerates this. **Verify on the Jetson** that enabling `fixed_frame` does not change the output `frame_id` contract STVL expects; if STVL stops marking, the fixed-frame output frame changed and the STVL source frame must be re-checked. This is the single highest-leverage de-skew fix — validate carefully.

**Fix 2.2 — (Fallback if 2.1 misbehaves) dedicated de-skew node**
- Add a small node between `velodyne_transform_node` and the costmap that uses the per-point `time` field + `/odometry/filtered` twist to rotate-and-translate each point to the common stamp (full SE(3), including the 0.089 m lever-arm-induced translation), publishing `/velodyne_points_deskewed`.
- Launch change: `src/avros_bringup/launch/sensors.launch.py` adds the node; STVL `observation_sources` topic → `/velodyne_points_deskewed`.
- Could break: extra CPU on an already-loaded Jetson; must verify it doesn't re-starve MPPI. Heavier than 2.1 — only if 2.1's driver path is insufficient.

### Tier 3 — Optional supplements (only if smear persists after Tier 1+2)

**Fix 3.1 — Cap MPPI angular rate in close quarters.** `nav2_params_humble.yaml` MPPI `wz_max`/`vx`/`az_max`: lowering effective rotation rate linearly reduces smear. Trade-off: slower turns. Only if needed.

**Fix 3.2 — Verify rpm vs published rate, then consider rpm 1200.** The 19.7 Hz vs 600 rpm (=10 Hz spec) discrepancy must be resolved first (likely dual-return or the unit is already faster). If the true sweep is 100 ms, `rpm: 1200.0` halves smear at the cost of per-scan azimuthal density. **Do not change blindly** — resolve the discrepancy first (§5).

---

## 5. Field validation procedure (Jetson)

**Hard rule: no RViz on the Jetson during nav — it starves MPPI (load 10.75→3.5 when killed). Use laptop Foxglove** (`foxglove_bridge` is in `navigation.launch.py`). Kill any stray RViz with `pkill -x rviz2` (never `-f`).

**Step 0 — Resolve the rpm discrepancy and rule out self-detection (gates Fix 3.2 and the low-prob min_range hypothesis):**
- `ros2 topic hz /velodyne_points` → confirm 19.7 Hz; compute true `T_sweep`.
- Rotate in-place at ~0.5 rad/s in an **open area, no nearby objects**. On laptop Foxglove watch the local costmap. **If a persistent near ring at 0.7–1.0 m appears with nothing around → it's self-detection (min_range);** raise `min_range` to 0.9 in `velodyne.yaml`. **If no ring → min_range is excluded** and the smear is purely external-object stale-mark.

**Step 1 — Baseline the symptom (before fixes):**
- Place a barrel/cone ~1 m from the robot. Command an in-place rotation at ~0.5 rad/s for one full turn.
- On laptop Foxglove, watch `/local_costmap/costmap` (or `/local_costmap/stvl_layer/voxel_marked_cloud`).
- **Metric to log at ~2 Hz:** lethal-cell count and the x,y spatial spread of lethal cells around the robot (same instrumentation used for the stationary sawtooth in `docs/lidar_obstacle_avoidance_test_2026_05_21.md:22–24`). Also log `ros2 topic hz /local_costmap/costmap_raw` and `/cmd_vel` to capture the CPU-load gate.
- Expected baseline: a visible trailing arc/ring; lethal count rises during rotation and decays slowly over several seconds after stopping.

**Step 2 — Apply Tier 1 (config), rebuild, re-run Step 1:**
- `colcon build --symlink-install --packages-select avros_bringup` (config install), relaunch `navigation.launch.py`.
- **Pass criterion:** the trailing ring is gone or collapses within <1 s of yawing past the object; lethal-cell spread tracks the object's true position, not its history. Confirm the barrel still marks reliably while approached head-on (Fix 1.1 didn't suppress real obstacles).

**Step 3 — Confirm no stationary-flicker regression:**
- Park stationary next to the barrel for 30 s. Log lethal-cell count at 2 Hz. **Pass:** sawtooth stays well under the historic ~40% loss. If flicker returns, check load (`top`/`ros2 topic hz /cmd_vel`) — it indicates CPU starvation came back, not that decay needs re-throttling.

**Step 4 — Apply Tier 2 (de-skew, Fix 2.1), re-run Steps 1+3:**
- Confirm STVL still marks (frame contract intact) after adding `fixed_frame: "odom"`.
- **Pass:** with de-skew, the ring is eliminated even at higher rotation rates (test 0.5 and 1.0 rad/s), and you can optionally restore higher `voxel_decay` without the ring returning — proving the tension is decoupled.

**Step 5 — Warmup caveat:** perform one discard rotation after every launch before any scored/measured run (Xsens quaternion settle, ~150 s + motion). If the ring only appears on the *first* post-boot rotation, that confirms the warmup mechanism rather than the decay/skew mechanism.

---

### Bottom line
The phantom ring is **un-deskewed intra-sweep azimuth smear (trigger) retained and accumulated by STVL's anti-flicker max-persistence tuning (amplifier)**, geometrically concentrated near close objects (large subtended arc + parallax + near-frustum slow decay) and gated intermittent by CPU-load-dependent costmap update rate. The EKF/yaw, `transform_tolerance`, and "no clearing path" hypotheses are refuted. Fix it by **(Tier 1)** raising `mark_threshold` to 1, `decay_acceleration` to 6.0, lowering `voxel_decay` to 3.0, widening `vertical_fov_padding` to 0.35; then **(Tier 2)** enabling `fixed_frame: "odom"` de-skew in `velodyne.yaml`, which removes the tension permanently and lets persistence stay high for anti-flicker.
