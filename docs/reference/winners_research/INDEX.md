# IGVC AutoNav Winners — Architecture Index

Cross-reference of the 5 deep-dive reports in this directory. Each report is a single-repo architectural breakdown; this index answers "which repo do I read for Q?"

## The 5 repos studied

| File | Team / year | Place | Course time | Public code? |
|---|---|---|---|---|
| [`2023_sooner_weebwagon.md`](2023_sooner_weebwagon.md) | Sooner Robotics — Weeb Wagon | **1st 2023** | **2:19 — fastest run ever** | Yes (ROS2 Humble) |
| [`2024_sooner_dangerzone.md`](2024_sooner_dangerzone.md) | Sooner Robotics — Danger Zone | **1st 2024** | 3:24 | Yes (ROS2 Humble) |
| [`2025_sooner_twistopher.md`](2025_sooner_twistopher.md) | Sooner Robotics — Twistopher | **1st 2025** | 2:20 | Yes (ROS2 Jazzy) |
| [`2026_tntech_lcas.md`](2026_tntech_lcas.md) | TnTech LCAS Lab | Active peer | n/a | Yes (ROS2 Humble + Isaac ROS) |
| [`hosei_orange_ros2.md`](hosei_orange_ros2.md) | Hosei KBKN Lab | Tsukuba + IGVC | n/a | Yes (ROS2 Humble) |

Why these five: 3 consecutive Sooner first-places (2023/2024/2025) give us the dominant pattern; TnTech is the **closest hardware match** to ours (Jetson AGX Orin + ZED + Nav2 MPPI); Hosei is the conservative-Nav2 baseline.

---

## Architecture pattern across the 5

| Aspect | 2023 Sooner | 2024 Sooner | 2025 Sooner | 2026 TnTech | Hosei |
|---|---|---|---|---|---|
| Uses Nav2? | **No** | **No** | **No** | Yes | Yes |
| Global costmap? | None | **None — 8×8 m rolling** | **None — camera mask** | Local-only; map≡odom | Standard Nav2 |
| `map` frame? | Particle filter local ENU | PF local ENU | PF local ENU @ 1st GPS fix | `map ≡ odom` identity TF | EKF + AMCL |
| Planner | A* on 80×80 + "Smellification" goal pick | A* on 80×80 + BFS goal pick | **No planner — raycast heading** | Nav2 default | Nav2 default |
| Controller | Pure pursuit | Adaptive pure pursuit (0.7→4 m) | **P-only on heading angle** | Nav2 MPPI | DWB |
| GPS waypoints | PF input | PF input | **Bias force per ray** | Carrot node → action goals | Waypoint YAML |
| LiDAR? | No (ultrasonic) | No (cameras only) | No (cameras only) | Yes (ZED) | Yes |

**Three consecutive Sooner wins (2023, 2024, 2025) used NEITHER Nav2 NOR a global costmap NOR a map-frame EKF.** That's a decisive pattern, not a marginal preference.

---

## Index by architectural question

### Q1 — Do we need a global costmap?

- **Read 2024 first** for the strongest "no" case: 80×80 cell × 0.1 m = 8×8 m **robot-fixed rolling grid**, never world-anchored. That whole world model fits inside `robot_radius × 10`. Code lives in `autonav_vision/src/combination.py` + `transformations.py`. Won 1st 2024.
- **Read 2025 next** for the most radical "no": no grid at all, raycasts on the camera mask directly. Won 1st 2025.
- **Read TnTech** for the "yes but defanged" middle ground: local costmap only, `map ≡ odom` identity TF, GPS waypoints fed via a separate carrot node that bypasses TF.
- **Read Hosei** for the canonical "yes" — and note their public IGVC config is unpublished.

### Q2 — Do we need a map-frame EKF / `map → odom` TF?

- **None of the 3 Sooner winners use a continuous `map` frame.** They all use a particle filter in local ENU centered on the first GPS fix. No `navsat_transform_node`, no map EKF.
- **TnTech does the smartest middle ground:** `map ≡ odom` identity TF so the costmap stays in odom-frame stability, GPS feeds a separate `carrot_to_nav2_action_node` that emits NavigateToPose goals at 2 Hz in the map-frame-that-is-actually-odom. GPS noise never enters the costmap.
- **Hosei uses navsat_transform + dual EKF (our pattern)** — closest to our current stack, but their public IGVC config is missing.

### Q3 — Nav2 MPPI vs Nav2 DWB vs no Nav2 at all?

- **No Nav2:** Sooner 2023/2024/2025 (all 1st place). Their controllers: pure pursuit (2023), adaptive pure pursuit (2024), P-only-on-heading (2025).
- **Nav2 MPPI:** TnTech 2026 (peer HW, not a podium finisher in this set).
- **Nav2 DWB:** Hosei. Worth borrowing as a fallback even if we keep MPPI.
- **Most actionable for us:** keep MPPI but configure DWB as a parallel-loaded fallback controller (Hosei pattern), and prototype a Twistopher-style raycast heading vector as a defensive fallback for when MPPI aborts.

### Q4 — Planner: Navfn vs SmacHybrid vs tangent vs raycast vs A* + goal-picker?

- **Sooner 2023's famous "tangent planner" was vaporware** — `tangent_based.py` had its main loop commented out and emitted an empty `Path()`. They actually shipped A* + a BFS goal picker called "Smellification."
- **The portable insight is the GOAL PICKER, not the planner itself.** A depth-50 BFS that biases the local goal selection by `(forward_progress, depth, gps_heading_err)` runs in front of A*. Could slot in front of our Nav2 NavigateToPose action with no replacement: a node consumes our GPS waypoint + costmap, picks a local goal 5 m ahead, sends THAT to Nav2.
- **Sooner 2025 has no planner at all** — 16 raycasts vector-summed → heading angle → P-only PID.

### Q5 — How do they handle GPS waypoints during the run?

- **Sooner (all years):** GPS feeds the particle filter; waypoints are checked by Euclidean distance in local ENU. 2025 specifically uses GPS heading as a **bias force** added to each ray.
- **TnTech:** `carrot_to_nav2_action_node` emits NavigateToPose action goals at 2 Hz with goal in map frame (where map ≡ odom).
- **Hosei:** Waypoint YAML in a separate versioned repo, fed to Nav2's `nav2_waypoint_follower`. Compatible with `§I.2` if the file lives outside the runtime memory.

### Q6 — State machine / mode switching (lane-follow vs GPS-waypoint)

- **Cedarville 2021 (not in this set but referenced):** 4-state FSM (Line Following / GPS Nav / Obstacle Avoidance from line / Obstacle Avoidance from GPS).
- **Sooner 2024:** state machine = `autonomous / manual / disabled / shutdown × competition / practice / sim` (9-state cross product, hand-rolled).
- **Sooner 2025:** simpler — system is reactive enough that mode switching is mostly a single autonomous flag.
- **TnTech / Hosei:** Nav2 Behavior Tree handles mode switching; recovery sub-trees are the failure-mode states.

### Q7 — IGVC §I.2 ("no mapping memorization between runs") compliance

- **Sooner (all):** trivially compliant — no map persists between runs because no map exists.
- **TnTech:** compliant — `map ≡ odom`, no saved map files.
- **Hosei:** their public stack uses AMCL-on-pre-saved-map (Tsukuba pattern), which **would be §I.2-violating at IGVC as-shipped**. Their actual IGVC config is unpublished. Slam_toolbox async-mapping-only mode IS compliant if re-run per attempt.
- **Us today:** compliant. Global costmap is built per-run, not saved.

---

## 3 cross-cutting lessons

1. **The grid does not need to be world-anchored.** Sooner 2024's 8×8 m robot-rolling grid is the proof: a costmap can be a *sensor-data fusion buffer* (just enough memory to integrate the last few seconds of perception in the robot's body frame) rather than a *world model*. Most of the 2026-05-19 GPS-smear / phantom-accumulation problems we fixed go away if we adopt this pattern. **Phase B should decide whether to keep our 100×100 m global costmap or shrink to 10×10 m robot-rolling.**

2. **The goal picker is cheaper than a better planner.** Sooner 2023's Smellification goal picker did more for course time than their fancy tangent planner ever would have. We could add a small node in front of NavigateToPose that re-projects "drive to GPS waypoint 50 m away" into "drive to this local goal 5 m ahead biased by waypoint bearing and forward progress," and feed THAT to Nav2 — without touching MPPI or the costmap layer.

3. **A fallback controller is a real feature.** Hosei's DWB-alongside-MPPI is worth ~3 hours of YAML work and a BT condition. Sooner 2025's raycast is more ambitious but the same idea — when the primary fails, have a dumb-but-robust secondary. Our current recovery BT (clear-costmap/wait/back-up) is a *recovery*, not a *controller*. They're different things.

---

## What to read next

- **For our biggest current pain point (GPS-smear / global-costmap phantoms):** start with `2024_sooner_dangerzone.md` §"Costmap" + `2026_tntech_lcas.md` §"Costmap / world representation."
- **For "what controllers are alternatives to MPPI":** `2024_sooner_dangerzone.md` §"Controller" + `hosei_orange_ros2.md` §"Planner + Controller."
- **For "is Nav2 even the right framework for IGVC":** read all 5 in order. The answer leans no for podium-grade performance, yes for development velocity.
