# auto_drive — point-and-go autonomous driving

Click a destination in RViz; the vehicle plans a route over the campus road
graph and drives there, using the camera costmap to stay on drivable surface
and avoid obstacles.

## Quick start

```bash
# 1. Once per session (~40 s: brings up the stack, activates lifecycle, opens RViz)
ros2 launch avros_bringup auto_drive.launch.py

# 2. Once per destination (~2 s)
ros2 run avros_navigation auto_drive
```

Then: click **Publish Point** in the RViz toolbar → click a spot on a road →
review the red route it draws → answer `y` at the `Drive it? [y/N]` prompt.

Leave step 1 running and re-run step 2 for each new destination. Bringing the
stack up is the slow part; you only want to pay it once.

**Before it will move, you must** open the joystick page and press **AUTO**.
`auto_drive` never clears e-stop or engages AUTO itself — arming stays a
human action. It re-checks both *after* the confirmation prompt too, in case
time passed while you were reading the route.

Useful launch args:

| Argument | Default | Purpose |
|---|---|---|
| `graph_range` | `300.0` | Radius (m) of road graph drawn in RViz. `0.0` = whole campus (much heavier — see *RViz performance*). |
| `rviz` | `true` | Set `false` to run headless (you then have no way to click a destination). |
| `cloud_bridge` | path in `carla-nav2-avl` | Location of `costmap_to_cloud.py`. Change if that repo lives elsewhere. |

## Pre-flight

`auto_drive` refuses to run unless every check passes. Each one exists
because its absence caused a real failure during bench testing — none are
decorative.

| Check | Why it exists |
|---|---|
| Seven servers report lifecycle **`active`** | Checking that a *process* exists is not enough. A killed `controller_server` still matched a `ps` grep (its name appears in the lifecycle_manager argument string), and separately the whole managed set sat `inactive` after a bond break. Both looked healthy; neither could drive. |
| **GPS fix valid** (not just publishing) | `/odometry/global` keeps publishing at 20 Hz with *no* satellite fix — `navsat_transform` projects lat/lon `0,0` against the campus datum and emits a position ~900 km away. A liveness-only check passes that happily and plans a route from the Atlantic. |
| Vision costmap flowing | No `/perception/costmap_cloud` means Nav2's ObstacleLayer is blind: no road-keeping, no camera obstacle avoidance. |
| CPU headroom (< 1.6 load/core) | Sustained overload starved the 20 Hz MPPI loop, producing `Optimizer fail to compute path` and erratic motion. |
| e-stop cleared, AUTO engaged | Operator arming interlock. |

If pre-flight fails, it prints which line failed and what to do. It exits
non-zero and sends nothing to the vehicle.

## How it works

```
   RViz click ──► /clicked_point
                       │
                       ▼
              route_server  ──── campus GeoJSON road graph
                       │         (routes along real roads)
                       ▼
              route split into ~30 m legs
                       │
                       ▼
   for each leg:  NavigateToPose ──► bt_navigator
                                        │
                    ┌───────────────────┴──────────────────┐
                    ▼                                      ▼
             planner_server                        controller_server (MPPI)
             (global path)                         (local trajectories)
                    │                                      │
                    └──────────► costmaps ◄────────────────┘
                                    ▲
                    ┌───────────────┴────────────────┐
                    │                                │
         STVL / Velodyne LiDAR            ObstacleLayer ◄── costmap_to_cloud
         (geometric obstacles)                            ◄── perception_costmap
                                                              (road vs off-road,
                                                               HSV + CLAHE)
                                    │
                                    ▼
              controller ─► /cmd_vel_nav ─► velocity_smoother ─► /cmd_vel ─► actuator_node
```

**Why legs?** The global costmap is a 100 m rolling window (~50 m usable
radius). `NavfnPlanner` cannot plan to a goal outside it — it returns *"goal
off the global costmap"* and aborts. So a long route is chopped into 30 m hops
driven in sequence. This mirrors the "dumb BT, smart orchestrator" split the
competition behaviour tree documents.

**Why the cmd_vel remaps?** `controller_server` publishes to `/cmd_vel_nav`,
`velocity_smoother` consumes that and republishes on `/cmd_vel`. Without the
remaps both publish `/cmd_vel` directly, the smoother is bypassed entirely,
and its acceleration limits never apply.

## Known limitations

Real, observed, and unfixed — read before trusting a run.

- **Parks short of the goal.** Typically 0.5–1.7 m. A known interaction
  between MPPI deceleration and the goal checker; documented in
  `nav2_params` comments. Intermediate legs tolerate it fine; the final
  position may be a metre or two off.

- **Backward-routing detours.** The campus graph contains duplicate nodes at
  identical coordinates that connect only through a dead-end spur (e.g. nodes
  `7162` and `13889` both at `(-102.1, 3.7)`, bridged via `437` at
  `(-106.9, 3.0)`). The planner correctly finds the shortest path *through the
  buggy graph*, which can send the vehicle backwards before it heads to the
  destination. **This is why the route preview matters — look at it before
  confirming.** Fixing it means de-duplicating the GeoJSON.

- **Shadows are improved, not solved.** CLAHE plus multi-blob retention fixed
  a reproducible stall at a tree-shadowed path. But this is still a classical
  brightness/colour threshold; deep canopy, low sun, or wet pavement may still
  fool it. The learned segmenter (TwinLiteNet) is the real fix and is not set
  up here.

- **Retries re-plan, they don't re-send.** On leg failure `auto_drive` retries
  up to twice, re-planning from the current position each time. Note Nav2's
  behaviour tree *already* retries 4× internally with recovery actions, so an
  outer retry sits on top of that.

## RViz performance

RViz here runs under **llvmpipe (software GL)** — the NoMachine virtual
display exposes no usable hardware GL context, so every vertex is rasterised
on the CPU. It is slow by construction.

The config is tuned accordingly: road graph decimated to edges only within
`graph_range`, no TF display, 5 Hz frame rate, no covariance ellipse. This
took RViz from ~104% CPU to ~11%.

**If it still feels laggy, the cause is usually another RViz.** The team's
always-on dashboard (`costmap_cams.rviz`) runs ~158% CPU plus ~90% for its
`costmap_rgb_node` feeder — around 2.5 cores. Closing it while driving roughly
halves system load.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `FAIL GPS fix status=-1 lat=0.000000` | No satellite fix. Check antenna and sky view. Nothing downstream works without it. |
| `FAIL <server> inactive` | Lifecycle set went inactive, usually after a bond break or a killed server. Restart the launch. |
| `FAIL AUTO engaged` | Open the joystick page and press AUTO. |
| Vehicle far from the graph in RViz | Almost always the GPS-fix issue above, not a display bug. |
| `goal off the global costmap` | Destination beyond the rolling window. Handled by leg chaining; if you see it, the leg spacing may need lowering. |
| `Optimizer fail to compute path` | MPPI starved or boxed in. Check CPU load; close extra RViz windows. |
| Route goes backwards first | Duplicate-node graph defect — see *Known limitations*. Re-pick a destination or accept the detour. |

## Open question: Teensy firmware

On 2026-08-19 the Teensy was reflashed from a machine other than the Jetson,
with a build predating this repo's current firmware (it lacks the `CS`/`CF`
current-limit commands added 2026-08-05, so a bare `C` returns
`ERR unknown`).

A diagnostic that day showed velocity mode producing 0 measured RPM while
open-loop duty mode spun the wheels normally — which would point at the
`kFF`-to-wrong-CAN-parameter-ID bug this repo's firmware history documents as
previously fixed. **That conclusion is not confirmed**: the board was already
in `mode=DUTY` with the watchdog tripped when the test started, so the
velocity-mode result may be a test artifact rather than a real defect. The
vehicle had also reportedly driven fine on that firmware the night before,
which argues against a broken velocity path.

Unresolved. If driving is weak or dead in velocity mode but fine under duty,
reflash from this repo:

```bash
arduino-cli compile --fqbn teensy:avr:teensy41 firmware/teensy_diff_drive
arduino-cli upload  --fqbn teensy:avr:teensy41 -p /dev/ttyACM0 firmware/teensy_diff_drive
```

The repo firmware compiles clean and includes the `CS`/`CF` additions, which
are opt-in and never invoked at boot.
