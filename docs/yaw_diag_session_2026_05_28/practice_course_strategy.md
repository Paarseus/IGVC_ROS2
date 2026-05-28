# Practice Course Test Strategy — 2026-05-28 (afternoon)

Course: parking lot with two parallel white tape lines forming a lane corridor + obstacles (barrels/cones) placed within the lane. Test = drive end-to-end inside the lane while avoiding obstacles.

## Going in with eyes open

- **Lane-in-costmap is currently blocked** by issue #18 (ZED TF extrapolation). Camera sees the lane, but the semantic layer rejects every update.
- **LiDAR obstacle avoidance is proven** today (4/4 goals SUCCEEDED with 0 recoveries, this morning's `nav_lidar_obstacle_20260528_065103` bag).
- **MPPI at vx_max=0.7 handles 5 m goals** cleanly when costmap is healthy.

## Three-stage plan

Each stage is independent, has its own bag, and has explicit pass/fail. If a stage fails, the next one is skipped or modified.

### Stage 1 — LiDAR-only baseline (~15 min)

**Why first:** proves obstacle avoidance still works on this physical course; sets the baseline for how much MPPI's natural path planning keeps the chassis between lines (it doesn't know about lines, so any in-lane behavior is incidental).

**Launch:**
```bash
ros2 launch avros_bringup navigation.launch.py  # no cameras, no perception
```

**Setup:**
- BT XML at 45 s timeout / 3 retries (verify after launch)
- vx_max = 0.7 m/s
- Bag: `practice_lidar_<timestamp>` with the standard topic list (no `/perception/*`, no `/zed_front/*`)
- Wait 150 s + drive a 3 m discard warmup (Xsens needs motion per `feedback-xsens-quat-needs-motion-warmup`)
- Clear costmaps OFF this time — let MPPI accumulate LiDAR data before the goal

**Goal:** single goal at the END of the course, in current heading direction.

**Pass criteria:**
- Goal status SUCCEEDED
- Chassis avoided all barrels (no contact, visual confirmation)
- Chassis reached within 2 m of physical goal mark (IGVC waypoint tolerance)
- Net displacement vs commanded > 70 %
- No BT recoveries triggered (recovery firing = something went wrong)

**Failure modes to watch:**
- Chassis crosses a lane line — EXPECTED if barrel placement forces it. Note where it crossed.
- Goal aborts — log the BT error
- Chassis stops, can't find path — costmap probably has phantom obstacles, clear and retry

**What this tells us:** how much of "stay in lane" is free from MPPI, and how much we need vision for.

### Stage 2 — Vision + TF workaround attempt (~20 min)

**Only run if Stage 1 passed** OR if Stage 1 specifically failed because the chassis cut outside the lane (proving we need vision).

**Workaround attempt — raise transform_tolerance:**
```bash
ros2 param set /local_costmap/local_costmap transform_tolerance 1.5
ros2 param set /global_costmap/global_costmap transform_tolerance 1.5
# default is 0.3 s; we go to 1.5 s to absorb the 0.3-0.4 s ZED stamp lag
```

**Launch:**
```bash
ros2 launch avros_bringup navigation.launch.py enable_zed_front:=true enable_perception:=true
```

**Validation gate** (before sending the goal):
- Grep `/tmp/nav_*.log` for "TF Exception" count after the param change. If error rate drops to ~0/min, workaround worked. If errors continue at the same rate, workaround failed.
- Grab `/local_costmap/costmap` and verify some cells correspond to detected lane (cells where vehicle isn't — non-zero cost in front of the chassis position, especially at the lane line distance).

**If validation gate passes:** send the same Stage 1 goal. Compare:
- Did chassis follow a tighter centerline path?
- Did `/plan` show curves that AVOID the lane cells?
- Same pass criteria as Stage 1, plus lane cells visible in costmap.

**If validation gate fails:** revert transform_tolerance to default, document that this workaround doesn't fix #18, do not send a goal. Move to Stage 3 or stop.

### Stage 3 — Sequential waypoints (~15 min)

**Only run if Stage 1 OR 2 passed.**

Test IGVC-style waypoint navigation: 3-4 sequential goals along the course, each in `map` frame, sent one after another. Closer to actual AutoNav (4 waypoints provided pre-competition).

Pass: chassis hits each waypoint within 2 m, total course time matches the 1 mph minimum, no recoveries.

---

## Risk register

| Risk | Mitigation |
|---|---|
| Brown-out under load | Don't run RViz on Jetson; use laptop Foxglove |
| ZED container crashes mid-goal | Monitor `/zed_front/zed_node/rgb/color/rect/image` rate during run. If drops, abort, restart stack |
| TF errors abort goal in Stage 2 | Workaround gate above; if fails, document and skip |
| Chassis crosses a lane line | User visually observes; not catastrophic for practice — informative |
| Goal too far for 45 s BT timeout | Keep first goal ≤ 15 m. At vx_max=0.7 with obstacles that's ~25 s travel + slack |
| Map drift fakes a "success" at >1 m | Note residual and net displacement separately; tolerance gate is "physical distance to the goal mark" measured by user |

## Pre-flight checklist (every stage)

- [ ] BT XML at 45 s / 3 retries (`grep "Timeout msec\|number_of_retries=" src/avros_bringup/config/navigate_igvc_autonav_humble.xml`)
- [ ] Xsens gyro bias healthy (μ < 0.05 °/s, σ < 0.15 °/s)
- [ ] Velodyne reachable (ping 192.168.13.11)
- [ ] `avros-webui` systemd stopped
- [ ] Bag started BEFORE any motion
- [ ] E-stop in hand (Telegram or webui — confirm before motion)
- [ ] Operator has clear line of sight to chassis
- [ ] Chassis physically at the practice course start mark

## Open questions before Stage 1

These affect goal computation and pass thresholds:

1. **Course dimensions** — total length, lane width (gap between the two white lines)
2. **Obstacle placement** — how many, where in the lane (forcing left/right swerves?)
3. **Chassis start position** — at the entrance of the lane? Pointing along the lane?
4. **Goal location** — single endpoint or multiple waypoints? For Stage 1 a single end-of-course goal is simplest

Once those are answered I can compute the exact map-frame goal coordinates and Stage 1 is ready to launch.

## Files / bags this test will produce

Each stage produces one bag at `/home/dinosaur/IGVC/bags/practice_<stage>_<timestamp>` with the standard 15-topic manifest (perception only on Stage 2/3).
