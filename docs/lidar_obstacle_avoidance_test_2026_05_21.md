# LiDAR-Only Obstacle Avoidance Test — 2026-05-21

Field test of the Nav2 MPPI navigation stack using **only the Velodyne VLP-16**
(camera/perception disabled), on the Jetson Orin (`~/IGVC`, ROS_DISTRO=humble).
Goal: confirm the production pipeline (Navfn planner + MPPI controller + STVL
costmaps) detects obstacles from LiDAR, plans around them, and drives to a goal.

**Outcome: obstacle avoidance works.** The robot reliably detected obstacles
(including a person repeatedly stepping into its path), steered around them, and
recovered/replanned without collision. Several stack-level bugs were found and
fixed along the way; they are documented below with the config changes committed.

---

## Test method

- Launch (lidar-only is the default): `ros2 launch avros_bringup navigation.launch.py`
  — `enable_velodyne:=true`, all `enable_zed_*:=false`, `enable_perception:=false`.
- Goal sent via a small `NavigateToPose` action client (5 m ahead of the robot).
- Speed capped for safety: MPPI `vx_max` 0.7→**0.35 m/s** (live param) + actuator
  `max_linear_mps`→0.4 backstop.
- Live instrumentation (Python helpers, not committed — kept in `/tmp` on the
  Jetson): costmap lethal-cell count near the robot, `/cmd_vel`, `/odometry/filtered`
  pose, and `/proc/loadavg`, sampled at 2 Hz to a log per run.
- Visualization: **Foxglove on the laptop** (foxglove_bridge is in the launch).

---

## Findings (root causes, in the order discovered)

### 1. RViz on the Jetson starves the control loop *(biggest issue)*
Running RViz2 on the Jetson over NoMachine's software OpenGL (llvmpipe) consumed
**133 % CPU**; with gnome-shell (~105 %) and a background update-manager (~77 %),
**load hit 10.75 on 8 cores**. That starved the MPPI control loop:
`controller_server: Control loop missed its desired rate of 20.0000Hz` fired almost
every cycle, the STVL costmap consumed LiDAR in bursts, obstacles flickered, and the
run ended in `Optimizer fail to compute path` → abort ("robot got really close").

**Killing RViz dropped load 10.75 → ~3.5** and the symptom cluster largely cleared.
MPPI itself used only ~16 % CPU — it was never the bottleneck.

→ **Do not run RViz on the Jetson during navigation. Use laptop Foxglove.**
(Saved to assistant memory. Note: `pkill -x rviz2`, not `-f rviz2` — `-f` self-matches
the shell and drops the SSH session.)

### 2. STVL costmap flicker (sparse VLP-16 + aggressive decay)
With the robot **stationary**, the local-costmap lethal-cell count sawtoothed
~40 % (e.g. 2435→1658, then a fresh scan snapped it back up). `/velodyne_points`
was steady at ~19.8 Hz, so the sensor was fine — the costmap's *consumption* was
bursty under load, while STVL's decay clock kept deleting voxels on schedule.

Key insight from reading the STVL source: for sparse 3D LiDAR the dominant clearing
path is **`decay_acceleration`** (frustum-accelerated decay of in-view-but-unconfirmed
voxels, ~`(1/6)·accel·t³`), **not** the `voxel_decay` timeout. Raising `voxel_decay`
alone did **not** flatten the sawtooth; lowering `decay_acceleration` is the lever.
With `inflation_radius` raised to 1.0 m the near obstacle never fully dropped
(min ≥10 lethal cells within 3 m), so residual flicker no longer endangered avoidance.

### 3. odom-frame goals → TF extrapolation abort *(self-inflicted)*
While chasing an unrelated red herring (see #4), goals were briefly sent in the
`odom` frame. That forces an `odom→map` transform every plan cycle using the goal's
fixed-at-send-time stamp; as it aged past the `map→odom` buffer:
`planner_server: Extrapolation Error ... [odom]→[map]` → "Could not transform the
start or goal pose" → planner abort → controller abort → goal failed.
**`map→odom` yaw was rock-stable (±0.007°)**, so map-frame goals avoid this entirely.
→ **Use map-frame goals.**

### 4. Red herring: "circling / left-arc bias"
Two runs looked like the robot circled without progress. It was actually **correct
avoidance of a person standing in the robot's path** (the operator was the obstacle).
The odom trace showed translation along an arc, not spin-in-place, and detection
began before the person entered the 2 m monitor window (LiDAR sees ~3 m out). No
control bug.

### 5. Simple BT had no recovery → aborted on the first hiccup
`navigate_to_pose_simple_humble.xml` is `RateController(ComputePathToPose) → FollowPath`
with **no recoveries**. A single planner failure collapses the `PipelineSequence`
and aborts the whole goal. The 1 Hz "replanning" only survives if every cycle succeeds.
→ **Default switched to `navigate_igvc_autonav_humble.xml`** (clear-costmap → wait →
back-up → crawl, 3× outer retry, 45 s watchdog). In the final run it fired **7
recoveries** and kept replanning through repeated blocking instead of aborting.

---

## Config changes committed

`config/nav2_params_humble.yaml`:

| Param | Old | New | Why |
|---|---|---|---|
| `controller_server … FollowPath.batch_size` | 2000 | **1000** | Nav2 doc sweet spot for constrained HW; halves optimizer cost so 20 Hz holds |
| `local_costmap … stvl_layer.voxel_decay` | 2.0 | **8.0** | Obstacle persistence under CPU-load bursts; odom frame has no GPS smear |
| `local_costmap … stvl_layer.velodyne_points.decay_acceleration` | 10.0 | **5.0** | Dominant clearing path for sparse VLP-16; 10 drove the flicker |
| `local_costmap … inflation_layer.inflation_radius` | 0.65 | **1.0** | ≥ robot_radius (0.8) → real early-avoidance gradient |
| `global_costmap … stvl_layer.voxel_decay` | 2.0 | **5.0** | Modest only — map frame, long decay smears GPS-drift phantoms |
| `global_costmap … stvl_layer.velodyne_points.decay_acceleration` | 10.0 | **5.0** | Match local anti-flicker |
| `global_costmap … inflation_layer.inflation_radius` | 0.65 | *(unchanged)* | Kept low — map frame is smear-prone; wide inflation there boxed the robot historically |

`launch/navigation.launch.py`: Humble default BT
`navigate_to_pose_simple_humble.xml` → **`navigate_igvc_autonav_humble.xml`**.

The speed cap (`vx_max` 0.35) and actuator backstop were live-only test settings and
are **not** committed — raise/lower per run.

---

## Validation (final run, recovery BT, map-frame goal, no RViz)

- Load ~3.5–7 (no RViz). LiDAR detected the operator every time he blocked it
  (`near2m` 0→52), MPPI steered around, **7 recoveries**, **no collision**.
- The action returned **ABORTED (status 6)** — but only via the **45 s watchdog**,
  because the operator kept blocking past 45 s (IGVC anti-hold-up behavior, working
  as designed). The goal and the entire straight-line path were confirmed **free
  (cost 0)**, so an unobstructed run would reach the goal and return SUCCEEDED.

**Avoidance: PASS. Formal goal-reach SUCCEEDED still to be captured** in one
unobstructed run (let it go after an early block or two).

---

## Open items / next steps

- Capture a clean `SUCCEEDED` (status 4) run with no sustained blocking.
- Re-measure on **grass** (this was a hard surface): STVL `min_obstacle_height`
  (0.2 → 0.3 if grass is tall) and the Mandow skid `wheel_separation_multiplier`.
- If flicker still bites on grass, lower `decay_acceleration` further (toward 1–2)
  rather than touching `voxel_decay`.
- The 12 V shared-rail brownout risk (Jetson + SparkMAX) is unaddressed — keep speeds
  modest until the dedicated Jetson buck is installed.
- Consider `use_realtime_priority: true` on MPPI only if the 20 Hz miss returns under
  field load (it didn't once RViz was off).
