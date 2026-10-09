# IGVC 2026 Lane-Following Strategy — Multi-Phase Plan

Synthesized from 3 parallel research agents (Sooner Robotics 2025, Sooner 2023/2024, official Nav2 docs) on 2026-05-28. The user identified that our Nav2 stack was treating lane lines as walls instead of corridor guides, causing the chassis to get trapped in 3 m lanes with 1.0 m inflation (effective corridor 1.0 m < robot diameter 1.6 m).

## The convergent finding

**Sooner Robotics won IGVC AutoNav three years in a row (2023, 2024, 2025) WITHOUT using Nav2.** Across all three years their stack:

1. Treats lane mask as obstacle, but **never inflates** — the lane is a hard-edge ray-stop / cell wall, not a radial cost gradient
2. Picks a **local goal between the lines** scored by `(forward_progress + waypoint_heading_alignment − corridor_intrusion)` — the GPS waypoint is a *bias vector*, never the planner's literal target
3. Drives straight toward that local goal with pure pursuit (2023/2024) or vector-summed feeler rays (2025)

**The bottleneck is the goal-selector, not the trajectory optimizer.** Sooner's planner *actively picks* a target inside the corridor that's biased toward the GPS waypoint. Our Nav2/MPPI tries to plan straight-line to the literal waypoint, which requires crossing the lethal-inflated lane walls.

**Nav2 has a native way to replicate this:** soft costs + `nav2_route` in augmentation mode + MPPI critic tuning. We don't need to rewrite the stack — we need to use Nav2's existing primitives correctly.

---

## Phase 1 — Soft lane cost (30 min, IMMEDIATE)

**Goal:** chassis can drive through a 3 m lane corridor without getting stuck. Same Nav2 stack we have today, one config change.

**Change:** in `src/avros_bringup/config/nav2_params_humble.yaml` (lines ~335-361 per Agent 3), split `lane_white` out of the `danger` class into a new `soft_lane` class_type:

```yaml
class_types: ["danger", "soft_lane", "ignored"]
danger:
  classes: ["barrel_orange", "pothole"]   # keep lethal — actual collisions
  base_cost: 254
  max_cost: 254
  mark_confidence: 1
  samples_to_max_cost: 1
soft_lane:                                # NEW class_type
  classes: ["lane_white"]
  base_cost: 180
  max_cost: 220                            # below INSCRIBED_INFLATED (253)
  mark_confidence: 0.6
  samples_to_max_cost: 3
```

Apply to BOTH `local_costmap.semantic_layer` and `global_costmap.semantic_layer` (lines 336 and 538 per earlier inspection).

**Why this works:** NavfnPlanner accepts cells with cost ≤ 252 (`navfn.cpp` checks `cost > 252` for rejection). MPPI's `CostCritic` weights cells with cost < 253 by `cost_weight × pose_cost`, so a 200-cost cell creates a strong gradient pushing trajectories away. DriveOnHeading's `Collision Ahead` check only fails on INSCRIBED/LETHAL cells (≥253), so backup recovery stops failing. The kiwicampus layer's existing `InflationLayer` already produces the centerline-attractive gradient we want from a 200-cost source.

**Procedure:**
1. Tear down nav2 (keep ZED + perception, or full tear-down)
2. Edit nav2_params_humble.yaml (changes above)
3. Relaunch `navigation.launch.py enable_zed_front:=true enable_perception:=true`
4. Wait 60s for stack settle + Xsens
5. Send same 10 m goal as before, observe whether chassis stays in corridor and reaches the goal

**Pass criteria:**
- Chassis traverses corridor without `Optimizer fail to compute path` errors
- BT doesn't trigger backup/drive_on_heading recovery (Collision Ahead messages absent)
- Chassis reaches within 2 m of goal (IGVC waypoint tolerance)
- Visual: chassis drives between the two lines, doesn't cross them

**If fail:** check the logs for which subsystem failed (planner vs controller vs recovery). The likely failure modes:
- Cost still too high → MPPI samples still avoiding corridor → try `base_cost: 150`
- Soft cost works but MPPI overshoots/oscillates → proceed to Phase 2 critic tuning

---

## Phase 2 — MPPI critic tuning for corridor centering (1-2 hrs, NEXT)

**Goal:** chassis drives **centerline** through the corridor, not zig-zagging between the lane walls.

Per Agent 3, MPPI's behavior in a soft-cost corridor is controlled by these critics (file: `src/avros_bringup/config/nav2_params_humble.yaml`):

| Critic | Current | Tune to | Why |
|---|---:|---:|---|
| `CostCritic.cost_weight` | 3.81 | **5.0–6.0** | Strengthens pull away from soft-cost cells (lanes) toward centerline |
| `CostCritic.consider_footprint` | `true` | keep `true` | Lane sweeps respected |
| `CostCritic.critical_cost` | 300.0 | keep | Only triggers on real lethals (barrels) |
| `PathFollowCritic.cost_weight` | 5.0 | **8.0** | Stronger pull toward the global path's carrot |
| `PathFollowCritic.offset_from_furthest` | 5 | **8** | Look further ahead on the path |
| `PathAlignCritic.cost_weight` | 14.0 | **10.0** | Loosen alignment for barrel dodges within corridor |
| `PathAlignCritic.max_path_occupancy_ratio` | 0.05 | **0.15** | Allow drift to avoid obstacles |

**Procedure:**
1. After Phase 1 passes, tune one critic at a time via `ros2 param set /controller_server FollowPath.<param> <value>`
2. Test goal after each change
3. Lock in values that work, commit to YAML

**Pass criteria:**
- Chassis centerlines visibly between the lanes (not hugging one side)
- Smooth path through 2-zigzag obstacle course at vx_max=0.7
- No oscillation or overcorrection

---

## Phase 3 — `nav2_route` augmentation mode (next session, 3-5 hrs)

**Goal:** multi-waypoint navigation through the IGVC AutoNav course, where waypoints may be on the other side of corridor segments.

Agent 3's recommendation: switch from `navigate_igvc_autonav_humble.xml` to `navigate_route_graph.xml` BT (already wired in `bt_navigator.plugin_lib_names`). The route graph is a hint — the planner still respects costmaps — so the chassis can leave the route to dodge a barrel and re-acquire.

**Required artifacts:**
- A `cpp_campus_graph.geojson`-style file for the actual IGVC course, generated from pre-competition GPS waypoints (we already have the tool: `src/avros_navigation/scripts/generate_graph.py`)
- A waypoint sequencer that pops waypoints from the IGVC-provided list and feeds them to the route_server

**Procedure:**
1. After Phase 1+2 pass, generate the course route graph from a mock 4-waypoint list
2. Switch `bt_navigator.default_bt_xml_filename` to `navigate_route_graph.xml` in nav2_params_humble.yaml
3. Send a `ComputeRoute` action goal (instead of `NavigateToPose`) with the start and goal node IDs
4. Verify chassis follows the corridor through waypoints, not straight-line

**Pass criteria:**
- Chassis enters corridor → reaches W1 → continues corridor → reaches W2 → … → completes all 4
- Doesn't try to short-cut across lanes
- Total time ≥ 30 s (44 ft @ 1 mph minimum) and < 360 s (6 min IGVC max)

**Reference:** Agent 3 cites [docs.nav2.org/configuration/packages/configuring-route-server.html](https://docs.nav2.org/configuration/packages/configuring-route-server.html) for the "Architecture 2" augmentation pattern.

---

## Phase 4 — Sooner-style local goal-picker (FALLBACK, pre-competition)

**Goal:** if Phases 1-3 still have edge cases (e.g., lane lines too far apart for MPPI's lookahead to react), implement Sooner's proven goal-picker as a Python node sitting between the waypoint queue and `NavigateToPose`.

**Architecture:**
- Input: GPS waypoint list + `/local_costmap/costmap` + chassis pose
- Logic: BFS on the local costmap, flood cells with cost < 80, score each visited cell `(forward_progress × 1.3 + depth × 2.2 − heading_err_to_waypoint × 1.0)`, pick highest-scoring cell as a **local goal**
- Output: publish that local goal to `/navigate_to_pose` action

This replicates Sooner's 2023/2024 "Smelly Algorithm" inside a Nav2 wrapper. **Their algorithm is the actual winning IP.** We keep Nav2 as the trajectory-rollout layer (MPPI is better than their pure pursuit on uneven ground), but use Sooner's goal-picker to ensure the literal Nav2 goal is ALWAYS reachable within the corridor.

**Implementation effort:** ~100 LOC Python + a small package. Cite: `autonav_ws/src/autonav_nav/src/astar.py` lines 254-316 in Sooner 2024 for the scoring function.

**When to invoke this:** only if Phase 1-3 don't deliver reliable corridor following. Phase 1 alone may be enough.

---

## Risk register

| Risk | Mitigation |
|---|---|
| Soft cost 200 still trips MPPI's collision check at boundary | Drop further (try 150, 100); but below 100 the gradient is too weak to push centerline |
| Lane detection still has transient asphalt false-positives at the soft cost (200 cells flood the corridor center) | Keep `sooner25` pipeline with `V_upper=215` (we tuned this morning); add a global lane-mask spatial filter (only keep N largest connected components) |
| `nav2_route` graph can't be generated for an unknown course in advance | Pre-competition waypoints are provided per IGVC §II.2. Generate offline. Have a fallback `NavigateToPose` BT for unknown segments |
| MPPI critic tuning doesn't transfer between practice and competition surfaces | Bring laptop + Foxglove to competition; re-tune onsite from a known-good baseline |
| ZED TF extrapolation errors (Issue #18) still cause goal aborts | Filed separately. Phase 1 changes lane handling but doesn't fix the TF bug. May need to relax `transform_tolerance` to 1.5 s as we tried this morning |

---

## What we are NOT doing (per agent recommendations)

- **Not switching planners** — NavfnPlanner is the right choice once costs are soft (Smac's hard-cost rejection threshold doesn't change with our edit; its turning radius constraint still applies)
- **Not adding `gradient_costmap_layer`** — kiwicampus's existing InflationLayer already produces the gradient we need from a 200-cost source
- **Not editing the BT to disable Collision Ahead** — `disable_collision_checks` doesn't exist on Humble (Jazzy-only). Soft costs make the check pass naturally
- **Not adding `ObstaclesCritic`** — `CostCritic` is the correct MPPI critic when costmap gradients carry the semantic intent
- **Not porting Inverse Perspective Transform (IPM) to perception** — Sooner used it but we have ZED's organized cloud giving us 3D directly; IPM is a 2D workaround we don't need

---

## TL;DR

1. **Immediate (Phase 1):** drop lane cost 254 → 200 in `nav2_params_humble.yaml`. Relaunch. Test goal. **Most likely fixes the corridor-trap.**
2. **Soon (Phase 2):** tune MPPI critics for clean centerline driving.
3. **Next session (Phase 3):** switch to `nav2_route` for multi-waypoint navigation.
4. **If needed (Phase 4):** add a Sooner-style local goal-picker as a Python wrapper.

The headline lesson across all 3 years of Sooner wins: **lane lines should not be lethal cells, and the goal-picker is the secret.** Both are achievable in our Nav2 stack with config changes only.

## Cited sources

- Agent 1: `/home/mspacman/IGVC_ROS2/docs/winners_research/2025_sooner_twistopher.md` + github.com/SoonerRobotics/autonav_software_2025
- Agent 2: `/home/mspacman/IGVC_ROS2/docs/winners_research/{2023_sooner_weebwagon,2024_sooner_dangerzone}.md` + the corresponding GitHub repos
- Agent 3: [docs.nav2.org/configuration/packages/configuring-mppic.html](https://docs.nav2.org/configuration/packages/configuring-mppic.html), [configuring-route-server.html](https://docs.nav2.org/configuration/packages/configuring-route-server.html), [kiwicampus semantic_segmentation_layer README](https://github.com/kiwicampus/semantic_segmentation_layer), navigation2 code (cost_critic.cpp, navfn.cpp, node_hybrid.cpp)
