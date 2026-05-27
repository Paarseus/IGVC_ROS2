# Phase B — Architecture Decision: Global Frame for IGVC AutoNav

**Status:** RECOMMENDATION (not yet implemented)
**Decision owner:** team lead
**Inputs:** IGVC 2026 rules §I.2, [`docs/winners_research/INDEX.md`](winners_research/INDEX.md), current state of `nav2_params_humble.yaml` + `ekf.yaml`, memory `project_no_rtk_in_codebase.md`.
**Date:** 2026-05-26

---

## TL;DR — four decisions

| Question | Recommendation | Risk | Effort |
|---|---|---|---|
| Q1 — Global costmap necessity | **Shrink 40×40 m → 20×20 m, frame stays `map` for now (changes with Q2)** | Low | YAML only, < 1 hr |
| Q2 — Map-frame EKF / `map → odom` TF | **Adopt TnTech `map ≡ odom` pattern + carrot-to-goal node. Decommission `navsat_transform_node` + `ekf_filter_node_map`.** | Medium (new code path) | 1–2 days |
| Q3 — MPPI vs alternative controller | **Keep MPPI as primary; add DWB as a configured-but-inactive fallback (Hosei pattern)** | Low | ~3 hr YAML + BT |
| Q4 — Planner | **Keep Navfn; add a "Smellification-style" local-goal picker node in front of NavigateToPose (Sooner 2023/2024 pattern)** | Low (additive) | 1 day |

The intent is to keep what already works (MPPI, Navfn, local STVL costmap, local EKF) and prune the parts that have been costing us field-test time without IGVC benefit (the map-frame EKF, the GPS-drift-prone global costmap, the single-controller failure mode).

---

## Context: what does IGVC AutoNav actually require?

From `docs/igvc_rules/IGVC_2026_rules.txt`:

> "Mapping or course position memorization is not allowed. Judges will adjust course between runs to nullify any mapping/memorization."

Course geometry: "primarily sinusoidal curves with series of repetitive barrel obstacles. Two waypoint pairs for the course will be provided prior to competition. One waypoint pair will be the entrance and exit of the course in No Man's Land. The two additional waypoints in No-Man's Land will guide the vehicles to the ramp."

So the course has **two regimes**:
1. **Lane-keeping** — vehicle stays between two painted white lines, avoids barrels. **No GPS waypoint needed** for this regime; perception + obstacle avoidance are sufficient.
2. **No Man's Land (GPS-waypoint)** — vehicle drives between known GPS coordinates with obstacles in between. **GPS waypoint is needed**, but only as a *direction* — the obstacles still come from perception.

The §I.2 rule forbids persistent maps but allows within-run representations. Our current stack complies, but so does every alternative below. Rule compliance is not the differentiator; **engineering complexity vs course performance** is.

---

## What past winners did (full table at [`winners_research/INDEX.md`](winners_research/INDEX.md))

| | 2023 Sooner | 2024 Sooner | 2025 Sooner | 2026 TnTech | Hosei |
|---|---|---|---|---|---|
| Global costmap | None | 8×8 m **robot-rolling** | None | 40×40 m in `odom` | Std Nav2 |
| `map` frame | PF local ENU | PF local ENU | PF local ENU | **`map ≡ odom` identity** | EKF + AMCL |
| Planner | A* + "Smellification" goal pick | A* + BFS goal pick | **No planner — raycast heading** | Navfn | Navfn |
| Controller | Pure pursuit | Adaptive pure pursuit | P-only on heading | MPPI | DWB |
| Nav2 | No | No | No | Yes | Yes |

The three consecutive Sooner first-place wins (2023/2024/2025) provide the strongest pattern: **no Nav2, no global costmap, no continuous `map` frame.** That said, total-rewrite to a Twistopher-style raycast stack would be reckless this close to competition. The realistic moves are middle-ground: adopt the TnTech `map ≡ odom` simplification, shrink our costmaps to match real planning horizons, and add fallback controllers.

---

## Q1 — Do we need a global costmap?

### Current state (`nav2_params_humble.yaml:227-456`)

```yaml
local_costmap:  width: 50, height: 50, rolling_window: true, frame: odom, robot_radius: 0.8, inflation: 1.0
global_costmap: width: 40, height: 40, rolling_window: true, frame: map, robot_radius: 0.8, inflation: 0.65
```

**Notice the local is BIGGER than the global.** That's because both are rolling, and the "global" is really just a second local costmap in a different (GPS-noisy) frame. Sooner 2024 ran on 8×8 m robot-rolling and won 1st. TnTech runs 40×40 m global but in `odom` frame, so GPS drift never enters it.

### Recommendation: **Shrink global costmap 40 → 20 m, otherwise keep**

Rationale:
- Our typical planning horizon is < 10 m (the 2026-05-21 test goal was 5 m ahead). 40 m global is providing space we don't plan into.
- We can't trivially drop the global costmap because Nav2's Navfn requires a `nav2_costmap_2d` instance to plan on. But we can shrink it to match the planning horizon.
- 20×20 m leaves room for waypoint goals in No Man's Land (max distance between provided GPS waypoints is typically ~30 m, but we re-issue goals as the carrot advances).
- **Defer the "drop global entirely" decision until Phase 4 measurements** — if local-only planning succeeds with the carrot pattern from Q2, the global becomes redundant.

### Config edits

`src/avros_bringup/config/nav2_params_humble.yaml:474-475`:
```yaml
# OLD:
      width: 40
      height: 40
# NEW:
      width: 20
      height: 20
```

---

## Q2 — Do we need a map-frame EKF / continuous `map → odom` TF?

### Current state (`ekf.yaml:101-169`)

Two EKF instances:
- `ekf_filter_node_odom` — fuses IMU + wheel_odom + ZED VIO yaw. Publishes `odom → base_link`. **Stable, validated.**
- `ekf_filter_node_map` — fuses IMU + `/odometry/gps` (from `navsat_transform_node`, differential mode, x/y process noise 0.1, rejection threshold 4.0). Publishes `map → odom`. **Stationary-validated only; motion validation is the 2026-05-19 WIP.**

The map EKF exists to give us a GPS-anchored position estimate so the global costmap can stay coherent over long runs. But:
- Three consecutive Sooner winners (2023/2024/2025) **proved this is unnecessary** for IGVC AutoNav. They all use a particle filter in local ENU centered on the first GPS fix, with GPS providing only waypoint bearing/distance.
- TnTech (closest HW match) **explicitly avoids** `navsat_transform_node` driving the `map` frame. They pin `map ≡ odom` (identity TF) and feed GPS into a separate `carrot_to_nav2_action_node` that emits NavigateToPose action goals at 2 Hz. This decouples GPS noise from the costmap entirely.
- Our `project_no_rtk_in_codebase.md` memory says: "design all EKF/GPS/costmap tuning for unaided SBAS GPS (2-5m noise); defend differential GPS, rejection threshold, low map-EKF process noise." **The TnTech pattern is fully compatible with that memory** — we keep the GPS-tuning logic (in the carrot node, not the EKF), we just stop feeding it into TF. The defense holds; the surface area shrinks.

### Recommendation: **Adopt TnTech `map ≡ odom` pattern + custom carrot node**

Concretely:
1. **Decommission `ekf_filter_node_map`** — delete the whole block from `ekf.yaml:101-169`.
2. **Decommission `navsat_transform_node`** — remove from `localization.launch.py` and `navigation.launch.py`.
3. **Add `map_odom_broadcaster` node** — tiny ROS2 node publishing `map → odom` static identity TF (or part of `robot_state_publisher` extension). Reference: TnTech `launch/navigation.launch.py:329-339`.
4. **Add `gnss_carrot_node`** — new node in `avros_navigation`:
   - Subscribes: `/gnss` (NavSatFix), `/odometry/filtered` (current pose in `odom`), waypoint list (YAML loaded at startup).
   - Computes: bearing + distance from current pose to next waypoint, using local-ENU projection from a datum (first GPS fix).
   - Publishes: NavigateToPose action goals at 2 Hz, OR a `/carrot_target` PoseStamped topic that a small action client consumes.
   - Advances waypoint index when within `waypoint_tolerance` (default 2.0 m, same as our existing `general_goal_checker.xy_goal_tolerance`).
5. **Update `navigation.launch.py`** — remove navsat, add gnss_carrot_node + map_odom_broadcaster.

### What we lose
- A continuous GPS-anchored position estimate. We can no longer say "the robot is at (x,y) in the world." But we don't need that for IGVC — only bearing+distance to the next known waypoint.
- The ability to visualize the robot's track on a satellite-image overlay (Foxglove can still show it in odom frame).

### What we gain
- The map→odom transform no longer drifts under stationary GPS. The 2026-05-19 §6-8 fixes (global STVL decay, EKF map process noise 1.0→0.1, GPS rejection 6.0→4.0) become irrelevant — the underlying problem is dissolved, not mitigated.
- The 2026-05-21 §1 finding (RViz on Jetson starves the loop) becomes less consequential because we're not pushing GPS-driven TF updates as fast.
- Phase 4 of the empirical plan (global EKF integrity test) is **obsoleted** — there's no global EKF to test.

### Config edits & new files

- DELETE `src/avros_bringup/config/ekf.yaml:101-169` (the `ekf_filter_node_map` block).
- DELETE `navsat_transform` references in launch files.
- NEW: `src/avros_navigation/avros_navigation/gnss_carrot_node.py` (~150 lines).
- NEW: launch entry for `map_odom_broadcaster` (static_transform_publisher with `0 0 0 0 0 0 map odom`).
- Update `bt_navigator.global_frame` in `nav2_params_humble.yaml:21` — keep as `map` (works because `map ≡ odom`).

### Risk / rollback
- Keep the old EKF map config in a sibling file `ekf_with_map.yaml` so we can A/B by relaunching with one or the other.
- The carrot node is additive — we can run the old `navsat_transform` path AND the new carrot path in parallel for one test session, compare goal-reach behavior, then decommission.

---

## Q3 — Should we keep MPPI as the only controller?

### Current state

`nav2_params_humble.yaml:61`: `controller_plugins: ["FollowPath"]` — single MPPI controller. When MPPI fails (e.g., `Optimizer fail to compute path` on 2026-05-21), the recovery BT engages clear-costmap → wait → back-up → crawl. That's a *recovery*, not a *controller*. The robot is stationary during recovery; we lose 1-5 seconds per failure.

### Recommendation: **Add DWB as a configured-but-inactive fallback (Hosei pattern)**

- DWB is the canonical Nav2 controller. Battle-tested, simpler than MPPI, robust. It uses dynamic-window-approach trajectory rollout — same mathematical family as MPPI but deterministic and CPU-cheap.
- Adding it loaded-but-unused costs ~30 ms RAM and zero CPU until invoked.
- BT change: add a condition that switches to DWB if MPPI's `FollowPath` returns FAILURE twice in a row, OR after a 5 s `is_path_valid` failure. Reverts to MPPI on next `ComputePathToPose` success.

### Config edits

`nav2_params_humble.yaml` controller_server block:
```yaml
controller_plugins: ["FollowPath", "FollowPathFallback"]
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  # ... existing ...
FollowPathFallback:
  plugin: "dwb_core::DWBLocalPlanner"
  # standard Clearpath Husky/Jackal DWB params
```

BT XML edit: add a `ReactiveFallback` wrapping `FollowPath` and `FollowPathFallback` in `navigate_igvc_autonav_humble.xml`.

### What this is NOT
This is **not** "abandon MPPI." MPPI is genuinely better when it works (it samples trajectories around obstacles instead of just following the path). The fallback is a safety net for when sampling fails — e.g., when `batch_size=1000` isn't enough for a tight cluster of barrels.

---

## Q4 — Planner: Navfn vs alternatives?

### Current state

`nav2_params_humble.yaml:185`: `nav2_navfn_planner::NavfnPlanner` (Dijkstra, holonomic, `tolerance: 0.5`, `allow_unknown: true`).

The 2024 Sooner winner used A* + a BFS "Smellification" goal picker. The goal picker did more for course time than the planner choice. The same Smellification pattern slots cleanly in front of our existing Navfn.

### Recommendation: **Keep Navfn. Add a local-goal-picker node in front of NavigateToPose.**

The "Smellification" idea: instead of telling Nav2 "go to GPS waypoint 50 m away" (which forces Nav2 to plan a 50 m path across territory it can't see), tell Nav2 "go to this local goal 5 m ahead that I picked because it's biased toward the waypoint AND it has forward progress AND it's in known-free space." Nav2 then plans a short, reliable path.

This is what Sooner 2024's `astar.py:295-323` does (BFS from `(40,78)` to depth 50, scoring `(80-y)*1.3 + depth*2.2 - heading_err*waypointWeight`). We could implement it as a wrapper around our existing `gnss_carrot_node` from Q2 — instead of just bearing+distance to the GPS waypoint, the carrot node picks the BEST local cell from the costmap that lies in the waypoint direction.

### Config edits & new files

- Extend the `gnss_carrot_node` from Q2 to:
  - Subscribe to `/local_costmap/costmap` (OccupancyGrid).
  - Run a small BFS from robot position into the cost map, scoring cells by `(forward_progress, depth_from_robot, waypoint_heading_error)`.
  - Pick the best cell as the local goal (instead of straight bearing).
- This is essentially making `gnss_carrot_node` smarter; not a separate node.

### Risk
- The Smellification picker introduces a new failure mode: if it picks a goal that turns out to be in an obstacle (costmap stale), Nav2's planner will fail. Mitigation: re-pick on planner FAILURE.
- Easy A/B: a `use_smellification: true/false` param that falls back to straight-bearing carrot mode.

---

## Migration order (low-risk to high-risk)

| Step | Phase | Action | Reversible? |
|---|---|---|---|
| 1 | Phase B+1 | Shrink global costmap 40→20 m (`nav2_params_humble.yaml:474-475`) | ✅ trivial |
| 2 | Phase B+1 | Add DWB fallback controller (Q3) | ✅ trivial |
| 3 | Phase B+2 | Implement `gnss_carrot_node` (additive, runs alongside navsat) | ✅ trivial |
| 4 | Phase B+2 | Add `map_odom_broadcaster` (parallel to navsat output, compare TFs) | ✅ static TF, easy revert |
| 5 | Phase B+3 | A/B test: full waypoint run on old vs new path | ✅ launch-arg switch |
| 6 | Phase B+4 | If A/B passes: decommission `navsat_transform_node` + `ekf_filter_node_map` | ⚠️ keep `ekf_with_map.yaml` sidecar for rollback |
| 7 | Phase B+5 | Extend `gnss_carrot_node` with Smellification goal picker (Q4) | ✅ feature-flag |

Steps 1-2 can ship this week. Steps 3-5 are the core migration (~2 days). Step 6 is the irreversible-ish commit. Step 7 is optional and can be deferred to after the first competition-track field test.

---

## Consequence for empirical phases 1–6

| Phase | Status | Change |
|---|---|---|
| 1 (instrumentation) | UNCHANGED | `/avros/teensy_diag` still needed for PID delivery telemetry |
| 2 (baseline) | MOSTLY UNCHANGED | Bag-record list updated: drop `/odometry/global`, add `/carrot_target` and `gnss_carrot_node` diagnostics |
| 3 (local EKF integrity) | **MORE IMPORTANT** | Local EKF becomes the only EKF; closed-loop square is now the *primary* validation, not a secondary one |
| 4 (global EKF integrity) | **OBSOLETED** | Replaced with: validate that `gnss_carrot_node` advances waypoints correctly under unaided GPS noise. New name: "Phase 4 — GNSS carrot waypoint convergence test" |
| 5 (MPPI fidelity) | UNCHANGED + AUGMENTED | Add a step 5e: A/B MPPI vs DWB fallback on the same goal — measure trajectory smoothness and CPU |
| 6 (grass portability) | UNCHANGED | Same calibration + validation matrix on grass |

---

## Open questions for the team

1. **Do we keep the `nav2_route` route-graph code path?** It's the alternative BT we use when `ROS_DISTRO != humble`. With the carrot-from-GPS approach, the GeoJSON route graph becomes redundant — the carrot IS the route. Recommend: delete `navigate_route_graph_humble.xml` + `cpp_campus_graph.geojson` from competition deployment (keep in repo for the CPP campus demo case).
2. **Do we want to prototype the Twistopher-style feeler controller as a tertiary fallback (after DWB)?** It's ~700 LOC of C++ but bypasses Nav2 entirely. Could be a 1-week stretch goal after the main migration. Worth it only if Phases 1-3 reveal that MPPI + DWB both fail on grass for reasons we can't fix in tuning.
3. **Does the team want to lock in the carrot's "waypoint advance" tolerance (currently proposing 2.0 m)?** §II rule says "two-meter navigation waypoint" — the rule itself sets the tolerance.

---

## References

- IGVC 2026 rules: [`docs/igvc_rules/IGVC_2026_rules.txt`](igvc_rules/IGVC_2026_rules.txt)
- Winner repos: [`docs/winners_research/INDEX.md`](winners_research/INDEX.md)
- Current Nav2 config: [`src/avros_bringup/config/nav2_params_humble.yaml`](../src/avros_bringup/config/nav2_params_humble.yaml)
- Current EKF config: [`src/avros_bringup/config/ekf.yaml`](../src/avros_bringup/config/ekf.yaml)
- Last GPS / costmap session: [`docs/session_2026_05_19_mppi_planner_costmap_gps.md`](session_2026_05_19_mppi_planner_costmap_gps.md)
- LiDAR-only avoidance test: [`docs/lidar_obstacle_avoidance_test_2026_05_21.md`](lidar_obstacle_avoidance_test_2026_05_21.md)
- Memory: `project_no_rtk_in_codebase.md` (defend unaided-SBAS GPS tuning)
