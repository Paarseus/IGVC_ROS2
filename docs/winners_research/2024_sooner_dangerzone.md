# Sooner Competitive Robotics — Danger Zone (IGVC 2024, 1st place)

## Repo & build

`github.com/SoonerRobotics/autonav_software_2024` — ROS2 Humble, Ubuntu 22.04, NUC. `autonav_ws/` has 11 packages incl. in-house `scr/` state-machine framework. Repo is 78% C++ but perception, planner, pure-pursuit are **Python**; only `expandify` and `scr::Node` are C++.

## Architecture (one ASCII diagram)

```
2x C920x 8fps  VN-200  GPS+enc
       ↓                  ↓
  transformations.py (HSV→persp→80x80)
       ↓ raw/{l,r}    particlefilter.py
  combination.py         ↓ /position
       ↓                 │
  expandify.cpp (dilate) │
       ↓                 │
  astar.py (BFS+A*, 10Hz)←┘
       ↓ /path
  path_resolver.py (pursuit+backup, 20Hz)
       ↓ MotorInput → serial_node → SocketCAN
  CAN hub → RP2040 Pico (Servo.h + enc)
       ↓ PWM 1000–2000μs
  SPARK MAX (PWM brushed) → NEO @ 10.7:1
       ↑ LoRa e-stop → estop_relay → CAN
```

## Localization

Frame is **local ENU anchored at first GPS fix** (`particlefilter.py:62-66`). **Particle filter** at `autonav_filters/src/particlefilter.py` fuses **GPS + encoder deltas only**; VN-200 IMU NOT in PF. 750 particles, GPS σ=0.45 m, odom-θ σ=0.1 rad, encoder scales 0.95/0.8 (hard-coded L41-43). Resample on every GPS callback.

## Costmap / world representation

**80×80 × 0.1 m = 8×8 m rolling local grid, robot-fixed** — NOT world-anchored. `transformations.py:194` builds each raw grid (HSV → perspective warp → `cv2.resize(80×80)`); `combination.py:73-79` OR-merges L+R. `expandify.cpp:53-82` dilates with a **pre-computed circle kernel** (`max_range=1.0 m`, `no_go_percent=0.70`): inside `no_go_range` stamps 100, outside linear falloff. Hand-rolled InflationLayer.

## Planner

`autonav_nav/src/astar.py` — **two-stage**, **10 Hz**. Stage 1 (`cfg_space_Received:295-323`): BFS from `(40,78)` to **depth 50**, scoring `(80-y)*1.3 + depth*2.2 - heading_err*waypointWeight`, picks `best_pos`. Stage 2 (`find_path_to_point:187`): A*, **8-connected**, Euclidean h, edge cost `dist + map[neighbor]/10`. Waypoint popped at distance² < 1.1 m. Competition waypoints hard-coded (`astar.py:25-41`).

## Controller

Pure pursuit in `path_resolver.py` + `pure_pursuit.py`. **Adaptive lookahead**: `radius_start=0.7 m`, ×1.2 until hit, capped `radius_max=4.0 m`. `forward_speed=1.5` scaled by `(1-|heading_err|/π)^5` — strongly slows in curves. `angular_aggression=1.8`, `max_angular_speed=0.8 rad/s`. **20 Hz**. **Recovery: no lookahead OR path < 0.5 m → back up at -0.4 m/s with fixed angular bias for 8 cycles** (`path_resolver.py:144-152`). Entire stuck-recovery, no costmap clearing.

## State machine

**Hand-rolled** in `scr/`. Two orthogonal axes: per-node `DeviceState` {OFF/BOOTING/STANDBY/READY/OPERATING/ERRORED} and global `SystemState` {DISABLED/AUTONOMOUS/MANUAL/SHUTDOWN} × `SystemMode` {COMPETITION/SIMULATION/PRACTICE} (`scr/include/scr/states.hpp`). `scr_controller/src/core.cpp` owns global state. Every node inherits `SCR::Node` and implements `init`, `system_state_transition`, `config_updated(json)`. Config is JSON-over-topic — live retune from GUI.

## Three lessons for us

1. **An 8×8 m robot-fixed local grid is enough to win.** No global costmap, no map frame. Our 100×100 m global is overkill for AutoNav and is the source of our smearing/phantom-obstacle issues — consider dropping the global layer for AutoNav runs.
2. **Pre-computed circle-kernel dilation at 80×80 is free** (`expandify.cpp`). Nav2's InflationLayer does this at 1000×1000 every cycle — wasted on a tracked diff-drive that can't fit between 2-3 m lanes.
3. **Two-stage planner (BFS goal-pick → A*) replaces global planning.** The depth-50 BFS picking `best_pos` IS the "where do I want to go" logic, biased by next-waypoint heading. Bonus: **back-up-and-turn recovery is 5 lines** (`path_resolver.py:144-152`) — steal as fast-path before our recovery BT.

## What was UNCLEAR

- **VN-200 IMU role** — `vectornav.cpp` exists but isn't consumed by PF. Possibly seed heading (`useSeedHeading`), no caller visible.
- **SPARK MAX firmware mode** — they drive SPARK MAX via **PWM servo signal (1000–2000 μs, `Servo.h`)** with encoders on the **RP2040 Pico**, NOT CAN velocity. MCU closes the velocity loop (`differential_drive.h` has its own PID); we use CAN + onboard SPARK MAX velocity PID + NEO brushless. Their CAN hub carries host↔Pico, NOT host↔SPARK MAX. **None of their PID gains transfer.**
- **Camera FPS** — 8 fps × 480×640 suggests CPU-bound; unclear if HSV or U-Net ran in the winning round.
- **`useOnlyWaypoints`** (`astar.py:254`) zeros the costmap and pathfinds on GPS alone. Fallback or test-only?
