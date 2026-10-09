# Nav2 LiDAR Obstacle-Avoidance Session — Analysis — 2026-05-28

Bag: `/home/dinosaur/IGVC/bags/nav_lidar_obstacle_20260528_065103` (33 GB) — `navigation.launch.py` with Velodyne ON, all cameras OFF, perception OFF. 2720 s total wall-time including ~30 min of stationary settling and 4 Nav2 goals.

CSVs: `bags/nav_lidar_obstacle_20260528_065103_csv/`.

## TL;DR

1. **All 4 Nav2 goals SUCCEEDED.** Zero recoveries triggered on any goal. MPPI emits clean continuous `cmd_vel` at ~20 Hz — no start/stop pattern (that was a separate bug in my warmup loop).
2. **L motor fix holds under Nav2 forward commands** — 93.8-94.1% delivery, L-R asymmetry within ±2.3 pp on forward legs.
3. **On the two "return" goals (W3, W5), Nav2 itself commanded high ω rates (13-23°/s)** to chase a goal whose map-frame coordinates were drifting at ~5-10 cm/s. The chassis tracked Nav2's commands accurately (`|imu_wz| ≈ |w_target|`); the actuator's heading-hold disengaged correctly (passes ω through when `|w_target|` is above deadband). Net effect: chassis drove in loops — covered 5.25 m of map distance in W3 while only 3.67 m of true odom distance. Same in W5 (10.17 m map vs 8.88 m odom).
4. **Map EKF generates 4× the actual chassis path** through sample-level pose noise (101.87 m map vs 24.87 m odom — odom matches wheel truth exactly). Stationary map drift is fine (0.11 m over 25 min); the problem is in-motion drift accumulating most heavily on the two longer goals.
5. **Chassis physically ended ~2.4 m from physical start** (odom net displacement). User's "it didn't return to start" observation is confirmed. Map frame drifted 5.3 m during the session — most during W3/W5.

## Per-window analysis

| Window | Event | Dur | L/Lc | R/Rc | L−R | ⟨v_target⟩ | Map dist | Odom dist | Map−Odom | Cum ω-hold |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| W1 | Buggy warmup loop | 9.2 s | 53% | 53% | +0.1 pp | 0.29 m/s | 0.26 m | 0.18 m | 0.07 m | +0.05° |
| W2 | Nav2 5 m fwd-A | 16.4 s | **93.8%** | **96.1%** | **−2.3 pp** | 0.35 m/s | 4.91 m | 4.90 m | 0.01 m | +75.6° |
| W3 | Nav2 return-A (rev) | 25.7 s | **90.3%** | **98.1%** | **−7.7 pp** | 0.28 m/s | 5.25 m | 3.67 m | **1.58 m** | **+552.9°** |
| W4 | Nav2 5 m fwd-B | 10.1 s | 94.1% | 92.7% | +1.4 pp | 0.35 m/s | 3.14 m | 3.07 m | 0.07 m | +19.9° |
| W5 | Nav2 10 m back | 33.4 s | 93.7% | 100.4% | −6.7 pp | 0.32 m/s | 10.17 m | 8.88 m | **1.29 m** | **+414.6°** |

Notes:
- W1 (buggy warmup): bash `while` loop calling `ros2 topic pub --once` produced only 6 commands in 9 s (~0.65 Hz effective). The slew limiter never reached steady state — that's why both L and R show ~53%. Visible as "go and stop" cycling. Filed as a tooling lesson, not a real motor issue.
- W2/W4 (forward goals): clean, in-band motor delivery, low map drift, mild heading-hold injection.
- W3/W5 (reverse goals): all three pathologies appear together — L delivery dips, R reports over-delivery, heading-hold injects huge cumulative ω, map drift accumulates.

## GNSS quality

- 2720 s of NavSatStatus=0 (basic FIX, **no SBAS, no RTK**) — 10878 samples
- Position spread over session: **4.4 m N-S × 9.8 m E-W**
- Reported covariance: ~0.65 m² (cov_e, cov_n)

The map EKF has to fuse these GNSS updates against `/wheel_odom` differential. With 4σ Mahalanobis rejection thresholds and the magnitude of noise present, occasional updates land and shift the map frame.

## Path-length sanity check (whole session)

| Source | Path integrated | Net start→end |
|---|---:|---:|
| `/wheel_odom` ∫|v|·dt (truth) | 24.86 m | — |
| `/odometry/filtered` (odom) | 24.87 m | **2.39 m** |
| `/odometry/global` (map) | 101.87 m | **7.68 m** |

- **Odom EKF is honest** — matches wheel truth to 0.04%
- **Map EKF inflates path 4.1×** through sample-level noise integration (per-sample step jitter at 30 Hz adds up). Not real chassis motion.
- **Net map displacement (7.68 m) − net odom displacement (2.39 m) = 5.3 m of structural map drift** over the session. Most occurred during the two reverse goals.

## Stationary map drift is fine

Longest stationary window: 1491 s (~25 min) of robot motionless.
- Net map drift during it: **0.11 m**
- Per-step drift: median **0.4 mm**, max 18.3 mm

Yesterday's drift_deep_analysis showed M0 stationary drift at 1.56 cm/s (6× M4 baseline). Today's is ~7 µm/s — **orders of magnitude better stationary**. The problem is purely motion-time.

## The "return goal" pattern — NOT a heading-hold bug

Three signals from W3 (the 5 m return goal that took 25.7 s and ended 1.58 m off in map distance):

| Source | Mean magnitude | Net integral |
|---|---:|---:|
| `\|w_target\|` (Nav2's commanded ω) | **22.79°/s** | — |
| `\|imu_wz\|` (raw gyro, chassis truth) | **22.19°/s** | +4.26° net |
| `\|w_after_imu\|` (actuator output) | **22.68°/s** | −15.41° net |

`|imu_wz| ≈ |w_target| ≈ |w_after_imu|` — the chassis is rotating at exactly the rate Nav2 commands, and the actuator passes Nav2's ω through (heading-hold deadband disengaged because `|w_target|` ≫ deadband). **Heading-hold is behaving correctly; the actuator is not over-correcting.**

The chassis rotated 553°-worth of motion (∑\|wz\|·dt) over W3 but ended only +4.26° different in heading. **Nav2 was driving the chassis in loops** — likely chasing a map-frame goal that was drifting at ~5-10 cm/s while MPPI's local plan kept getting updated.

W3 odom-frame distance: 3.67 m of physical chassis travel; W3 map-frame distance: 5.25 m of map-coordinates traversed. Difference = chassis going through loops to track a non-stationary target. Same pattern in W5 (10.17 m map / 8.88 m odom).

**Root cause** of the W3/W5 high-rotation behavior: **GPS-noisy map frame + tight goal tolerance** = MPPI sees the goal "moving" in map frame, replans continuously, and the chassis is forced into wiggling/circling motion to track it.

The "L−R asymmetry" numbers (−7.7 pp in W3, −6.7 pp in W5) are also explained by this: when the chassis is constantly rotating, the heading-hold IS active and pulls L and R asymmetrically to inject the ω corrections that Nav2 isn't already providing. That's the diff-drive inverse doing its job, not a motor problem.

## What the actuator and motors actually did

- **Motors:** L delivery 90.3% (W3) to 94.1% (W4); R delivery 92.7% (W4) to 100.4% (W5). All within the post-fix expected band; no regression from this morning's 96% baseline. The slight under-delivery on rotation-heavy windows is consistent with skid losses during in-place rotation.
- **Actuator's IMU heading-hold:** correctly engaged on straight-line forward (W2, W4), correctly disengaged on rotation-heavy commands (W3, W5). Not contributing to drift; not over-correcting.
- **Slew limiter:** working — no evidence of v_target getting stuck or oscillating in the Nav2 windows.

## What this means for IGVC AutoNav

- **Forward goals are clean.** L motor fix + accurate motor delivery + low map drift + Nav2 commanding straight-line motion.
- **Re-approach / return goals are fragile** — not because the chassis is broken, but because **the map frame is moving and MPPI chases**. Any backtrack mission, recovery BT back-up step, or "return to a previously-known waypoint" exposes this.
- **GNSS-anchored map frame is the wrong reference for tight (sub-meter) goal tolerance** with unaided FIX-only GPS. Good enough for routing-class waypoints (5-10 m tolerance), bad for 0.5 m tolerance.

## Recommendations

1. **Immediate (next test session): use odom-frame goals for tight tolerances.** Odom-EKF matches wheel truth to 0.04% over the whole session. Map drift becomes irrelevant. Nav2 accepts odom-frame goals; the only caveat is the CLAUDE.md note about goal-timestamp aging — solvable by re-sending the goal with current timestamp or using a planner that doesn't re-transform.
2. **For tighter map-frame anchoring:** the `odom0_pose_rejection_threshold` is the lever (already at 4.0 / 4σ Mahalanobis per ekf.yaml). Tightening to 3.0 was suggested in yesterday's `drift_deep_analysis.md:53` and is now well-motivated by today's data showing the stationary drift is tiny — the EKF rejection is gating motion-time updates more than stationary ones.
3. **Long-term: NTRIP/RTK if rules allow.** Subject to IGVC §I.2 judge ruling per memory `project-igvc-rtk-rule`. Would tighten the map frame substantially.
4. **The recovery BT back-up step** doesn't appear to be a brick — it's just a return-class motion in a different costume. Will exhibit the same map-drift wiggle if used. Worth field-testing the recovery BT explicitly in a future session.
5. **MPPI sample tuning** could potentially be made less aggressive on the rotation axis — the 22°/s commanded rotation is at the high end. But this is a tuning tradeoff against responsiveness; not urgent.

## Cleanup status

- Bag preserved on Jetson at `/home/dinosaur/IGVC/bags/nav_lidar_obstacle_20260528_065103` (33 GB)
- CSVs on laptop at `bags/nav_lidar_obstacle_20260528_065103_csv/` (135 MB)
- Nav2 stack still running on Jetson (kill cleanly when done)
- BT XML reverted to 45 s / 3 retries ✓
