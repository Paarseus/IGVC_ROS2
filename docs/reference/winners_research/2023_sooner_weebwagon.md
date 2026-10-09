# Sooner Competitive Robotics — Weeb Wagon (IGVC 2023, 1st place — 2:19 fastest run ever)

## Repo & build
- Repo: https://github.com/SoonerRobotics/autonav_software_2023 (ROS2 Humble / Ubuntu 22.04, Intel NUC compute).
- Workspace `autonav_ws/`, custom `scr_*` framework on top of ROS2 (`scr_core`, `scr_state`, `scr_configuration`, `scr_logging`) for managed-node lifecycle + a key/value config store.
- Deployed launch file: `autonav_ws/src/autonav_launch/launch/competition.xml`. **No Nav2, no costmap_2d, no robot_localization** — everything custom.
- Sensors: USB webcam on a tall PVC pole (only forward-looking ranging sensor), VectorNav IMU, Emlid Reach RTK GPS, and 3× ultrasonic on a custom CAN PCB (`autonav_msgs/ObjectDetection`) as firmware-level emergency backstop.

## Architecture (one ASCII diagram)
```
camera(USB)──► transformations.py ──► /cfg_space/raw (80×80 OccupancyGrid) ──► expandify (C++)
                (HSV→mask→IPM warp→80×80 grid)                                   │ inflate w/ precomputed
                                                                                 │ circle kernel
                                                                                 ▼
GPS ─► filters.py (ParticleFilter 750 particles)                       /cfg_space/expanded
IMU ─►   x=motor odom,  z=GPS likelihood                                        │
encoders─► /autonav/position (lat/lon + x/y/θ)                                  │
                              │                                                  ▼
                              └──────────────────────────► astar.py (single node, two stages)
                                                            (1) "Smellification" BFS 50 deep on
                                                                expanded grid → bestPosition
                                                                (goal in robot-frame grid)
                                                            (2) A* (heapq) 80×80 → Path
                                                            running @ 10 Hz
                                                                  │
                                                                  ▼
                                                       path_resolver.py @ 20 Hz
                                                       (pure-pursuit lookahead, expanding radius)
                                                                  │ MotorInput (v, ω)
                                                                  ▼
                                                       serial_node.py ──CAN──► motor_control.ino
                                                       (CON-bus register R/W)   (Teensy + L+I PIDs)
```

## Localization
`autonav_filters/src/particlefilter.py` — 750-particle filter, state = (x, y, θ). Motion model = wheel-encoder ΔX/ΔY/Δθ (from `MotorFeedback` CAN msg, scaled `/10000`) with σ=(0.05, 0.05, 0.1). Measurement = GPS, weight = `exp(-d/(2·σ_gps²))` with `σ_gps=0.45 m`. Resample every GPS callback. **No IMU fusion in the particle weight** — IMU is only consumed by `astar.py` for heading bias and by `filters.py` for seed-heading on reset (`filters.py:62-64`). First GPS fix becomes the local origin; lat/lon ↔ meters uses a flat-earth scale (`latitude_length=110944.21`, `longitude_length=81978.2`). Dead-reckoning filter (`deadrekt.py`) is a selectable alternative via `filter_type` config.

## Costmap / world representation (if any)
80×80 cell `nav_msgs/OccupancyGrid`, 0.1 m resolution, only the camera-flattened forward strip (3.0 m wide × 2.75 m deep). `transformations.py` does HSV mask → region-of-disinterest cull → inverse-perspective warp → resize to 80×80. `expandify.cpp` precomputes a list of (Δx, Δy, r) offsets once at startup (`maxRange=0.65 m`, `noGoPercent=0.70`); on each frame it stamps every obstacle pixel with that kernel, painting 100 inside the no-go core and a linear gradient outside (`expandify.cpp:101-110`). This is the team's analog of `costmap_2d` InflationLayer, ~5 lines of math, no plugins.

## Planner — the tangent planner
**The novel circle-tangent planner described in the team's 2023 report is in `autonav_ws/src/autonav_pathing/src/path_planning/tangent_based.py` but was NOT wired into the competition launch file** — `autonav_launch/launch/competition.xml` runs `autonav_nav/astar.py`, not `autonav_pathing/path_planning.py`. The tangent code that's public is incomplete (`planning_test` is invoked once on each Obstacles callback at `path_planning.py:44` but its output is never published; `PathPlanner.publish_path` publishes an empty `Path()`). What the public code shows: each obstacle is a circle `(cx, cy, r)`; the planner iterates GPS waypoints, intersects each path segment against every obstacle circle using the line-circle quadratic from mathworld (`tangent_based.py:264-293`), and when a segment cuts a circle it inserts 5 detour points spaced evenly along the safe arc (`point_adder` at `tangent_based.py:86-119`), tried both clockwise and counter-clockwise, then runs `path_intersections` to delete loops created by overlapping detours. **The deployed planner** is `autonav_nav/src/astar.py`: per-cycle 8-connected A* (heapq, `findPathToPoint`, `astar.py:178-233`) from a fixed robot cell `(40, 78)` on the 80×80 inflated grid to `bestPosition`. **Planning horizon = the 80×80 grid only (≈2.75 m forward strip), recomputation rate = 10 Hz** (`self.create_timer(0.1, self.createPath)`, `astar.py:97`).

## "Smelly Algorithm" goal selection
Not a separate module — it lives inside `astar.py` as `onConfigSpaceReceived` (`astar.py:235-322`), bracketed by `performance.start("Smellification")` / `end("Smellification")`. It's a depth-limited BFS (max 50, frontier set, only cells with cost < 50 are walkable) that floods the inflated grid from the robot cell, scoring every visited cell with `cost = (80 - y)*1.3 + depth*2.2 - max(heading_err_to_gps_deg, 10)` (`astar.py:296-300`). The cell with max score becomes `bestPosition`, which is then the goal handed to A*. The three biases are: forward-progress (`80-y`), exploration depth (`depth*2.2`), and alignment with the next GPS waypoint's bearing (`heading_err_to_gps`). Waypoints are popped when within `pop_distance=1.1 m`; a `waypoint_delay=17.5 s` arms autonomy after the no-mans-land start.

## Controller
Pure pursuit. `autonav_nav/src/pure_pursuit.py` finds the lookahead point with expanding radius (`radius_start=0.7`, `radius_max=4.0`, `radius_multiplier=1.2` per retry, `path_resolver.py:113-115`). `path_resolver.py:106-146` computes heading error to the lookahead, sets `forward_velocity = 2.1 * (1-|err|)^5` (very aggressive slowdown in turns) and `angular_velocity = clamp(err * 2.2, ±0.5)` at 20 Hz. If no lookahead can be found for 8 consecutive ticks it commands a fixed reverse maneuver (`back_speed=0.4`, `reverse_speed=-0.4`) — recovery is hard-coded, no BT.

## State machine
`scr_core` exposes a custom 3-axis state: `SystemState` (DISABLED/MANUAL/AUTONOMOUS/SHUTDOWN), `SystemMode` (COMPETITION/PRACTICE/SIMULATION), per-node `DeviceState` (OFF/STANDBY/READY/OPERATING). Every node inherits `scr_core.node.Node` and overrides `configure()` and `transition(old, updated)`. The `scr_state/systemstate` node arbitrates global state from CAN E-stop (arb_id=0), CAN mobility-stop (id=1), mobility-start (id=9), and the operator UI (`autonav_display/broadcast.py` is a websocket bridge for a browser dashboard). E-stop is hardware: a separate Arduino in `firmware/estop_relay/` cuts motor power; the relay publishes arb_id=0 on the CAN bus and `serial_node.py:88-90` propagates it into the ROS state.

## CON-bus live tuning
A custom register protocol layered on raw CAN, **purpose-built for in-pit tuning without reflashing the Teensy.** Each firmware device has a device-id (motor controller = `0x10`, `motor_control.ino:44`) and exposes typed registers. Wire format on CAN arb_id range **1000–1399** (`serial_node.py:132`); host publishes `autonav_msgs/Conbus{id, data, iterator}` on `/autonav/conbus/instruction`, firmware echoes reads on `/autonav/conbus/data`. The browser dashboard (`autonav_display/broadcast.py:153-161`) sends conbus writes over websocket. Registers exposed by the motor controller (`motor_control.ino:106-129`):

| Reg | Field | Purpose |
|-----|-------|---------|
| 0x00 | updatePeriod | **read-only** loop dt |
| 0x01–0x06 | pulsesPerRadian, wheelRadius, wheelbaseLength, slewRateLimit, leftEncoderFactor, rightEncoderFactor | chassis kinematics |
| 0x10–0x13 | velocity kP/kI/kD/kF | linear-velocity inner PID |
| 0x20–0x23 | angular kP/kI/kD/kF | yaw-rate inner PID |
| 0x30, 0x31 | useObstacleAvoidance, collisionBoxDist | ultrasonic gating |
| 0x40 | sendStatistics | debug stream toggle |
| 0x50 | motor_updates_between_deltaodom | odom decimation |

This is exactly the analog of our `/avros/teensy_diag` + `K[PIDF]` line idea — but cleanly typed, addressable, lives on CAN (not USB-CDC), and is wired into the operator GUI so a team member retunes PIDs while the vehicle is on the course. **Note 0x01–0x06 expose `wheelbaseLength` and `pulsesPerRadian` as live registers** — Sooner can recalibrate the chassis kinematic constants without rebuilding firmware, the same capability we'd need for live `wheel_separation_multiplier` tuning on IGVC grass.

## Three lessons for us
1. **The fastest IGVC run ever does NOT use the fancy planner from the team's report.** It uses a 10 Hz A* on an 80×80 grid with a hand-rolled BFS goal-picker biased by `(forward_progress, depth, gps_heading_err)`. Our 2.8 s × 1000-sample MPPI rollout is dramatically more expensive than what won. The right architectural question for IGVC is not "how good is the trajectory optimizer," it's "how fast and forward-biased is the goal selector that feeds it." Consider building a "Smelly-style" local goal picker that biases Nav2's NavigateToPose target toward `(forward, GPS-aligned)` instead of always feeding it the literal GPS waypoint — Nav2 then plans the short hop.
2. **A typed register protocol over CAN/serial beats ad-hoc K-lines.** Our K[PIDF] line is one-way and untyped; CON-bus is addressable read/write with a known table, and the operator dashboard speaks the same protocol. If we generalize `/avros/actuator_command` ↔ Teensy into a register table (PID gains, slew caps, wheel_separation_multiplier, IMU heading-hold gains, encoder factors), we get pit-side tuning for free.
3. **Camera-only IPM-to-80×80-grid is a credible IGVC obstacle representation.** Sooner's entire world model is HSV + region-of-disinterest cull + 4-point perspective warp into a `0.1 m × 80 × 80` grid (3 m forward × 2.75 m wide). That's an 8 m² window. Our Velodyne+ZED stack is far richer; the lesson is that **the planning horizon that wins IGVC is tiny (~3 m forward) and the algorithm just needs to react fast enough.** A 10x10 m local costmap with MPPI rolling 2.8 s ahead is heavy for a course this dense — consider shrinking horizon to match.

## What was UNCLEAR
- Whether the tangent-circle planner was ever actually deployed at IGVC 2023, or whether it's a research prototype that lost to the simpler A*+Smelly stack. The public `competition.xml` runs A*; the `autonav_pathing` package's `path_planning.py` has `publish_path()` emitting an empty `Path()` and its main loop commented out (`path_planning.py:104-122`). If the team did use the tangent planner, that code lives in a private branch.
- The exact "obstacle circle" extraction step the tangent planner expects (`Obstacles.obstacles_data` with `center_x/y/radius`) — the public repo has no node producing that topic. `circumscriber.py` exists in `autonav_pathing/object_detection/` but is unimported by any node entry point.
- The IMU heading offset (`degree_offset=107.0` in `filters.py:41`) is a magic number with no derivation in the code or docs.
