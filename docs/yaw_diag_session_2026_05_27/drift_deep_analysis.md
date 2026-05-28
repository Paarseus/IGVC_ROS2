# Drift Deep Analysis — 2026-05-27 Field Session

> See [README.md](README.md) for folder index. See [SESSION_FINAL.md](SESSION_FINAL.md) for full session summary. See [TOMORROW.md](TOMORROW.md) for next-session test plan.

## Headline verdict

**Map→odom drift would NOT have broken obstacle avoidance.** Peak 1 s drift hit **55.4 cm/s** (t=552.2 s, mid-turn), but no tf step exceeded 0.2 m; net drift over the 70 s goal window was **2.54 m vector / 11.19 m path** (avg 16.0 cm/s). Obstacles older than ~4 s in the global costmap already smear past the 0.65 m inflation radius — risk for missions that re-approach a seen obstacle, harmless on this single-pass run. **Local dead reckoning is sound:** +7.5% path-scale error vs raw-GNSS over 14.5 m — inside the 1 m / 10 m budget. Map-EKF path excess over local-EKF (+13.6/+24.8/+37.3 m across the three bags) shows the **map frame is noisier than odom**, driven by stationary GPS-noise pumping (~2.7 cm/s), not inertial failure.

## Per-bag drift (cm/s; rows from `tf.csv` where parent=map, child=odom)

| Bag | Dur | rows | 60 s win (min/med/max) | Max 1 s | >5 | >50 | Δxy>0.2 m | Stationary | Motion |
|---|---|---|---|---|---|---|---|---|---|
| s2 (pre-fix)  | 1421 s | 42 078 | 1.88 / 2.69 / 9.97 | 28.2 @149.8 | 152 | 0 | 0 | **3.34** | 11.83 |
| s3 (post-fix) |  362 s |  8 047 | 1.94 / 3.89 / 13.5 | 36.1 @222.1 | 85  | 0 | 0 | **2.33** | 21.25 |
| obs           |  905 s | 24 684 | 1.39 / 2.62 / 14.8 | **55.4 @552.2** | 110 | 2 | 0 | **2.70** | 16.07 |

M4 baseline 0.26 cm/s — we are 9–13× worse stationary (looser SBAS, 4σ Mahalanobis gate fires; no catastrophic jumps). Top obs spikes: 552.2 (55.4), 725.1 (55.0, stationary GPS-snap), 553.2 (43.9), 551.2 (43.5), 598.0 (40.0, end-of-goal).

## Obstacle-avoid goal window (t = 530–600 s)

| Source | Path | Net displacement |
|---|---|---|
| raw `/gnss` ENU (ground truth) | **14.53 m** | 5.23 m @ +9.4° |
| `/odometry/filtered` (local EKF) | 15.62 m (+7.5%) | 11.21 m |
| `/odometry/global` (map EKF) | 18.89 m (+30%) | 9.85 m |
| `tf(map→odom)` accumulated | 11.19 m | 2.54 m |

Goal succeeded within 0.49 m at t≈599 s. Local-EKF (11.2 m net) vs GNSS (5.2 m net) gap is *geometric* — MPPI netted 5 m forward despite 14.5 m driven. The 30% map-EKF path excess is the GPS-noise penalty (~16 cm/s pumped per motion window).

|wz|>0.8 rad/s clusters at t=550–555 (0.98) and 575–585 (0.87) coincide with top drift spikes (552.2, 553.2). **Drift correlates with, but does not cause, the turns** — GPS correction lands while heading yaws fastest. Lone 2.09 s cmd_vel gap at t=594.2 is final approach, not abort.

## Bearing-fix verification

`/odometry/gps` vs raw-GNSS ENU bearing: s2 — all 6 motion windows show **−145.1° ± 0.0°** offset. s3 and obs — all windows show **+0.1°** / **−0.0°**. **Datum fix landed exactly as designed.**

## Costmap smearing risk (global frame)

At 16 cm/s in-motion drift: 1 s → 16 cm (safe), 3 s → 48 cm (encroaching), **4 s → 64 cm (at 0.65 m global inflation — older = unsafe)**, 10 s → 1.6 m (ghosts). **Local costmap (odom-frame, 1.0 m inflation) is immune** — VoxelLayer projects LiDAR in odom, which is why Velodyne reactive avoidance kept working despite the spikes.

## Verdicts

**(a) Map→odom drift large enough to break obstacle avoidance?** Not for **reactive LiDAR** (local costmap, odom-frame, immune). For **global-costmap memory** of obstacles re-approached: **yes, beyond ~4 s staleness.** The obs run was single-pass so risk never materialized; a backtrack mission would expose it.

**(b) Does dead reckoning fail short-distance (10 m) nav?** **No.** Local-EKF path scale error +7.5% over 14.5 m (≈1.1 m, on budget edge). Pure forward driving (t=555–575 s): map→odom drift 0.30 m net / 2.08 m path over 20 s — directional component stable; excess is integration noise, not heading drift.

**(c) Did obstacle_avoid actually fail due to drift?** **No.** Goal SUCCEEDED within 0.49 m. Drift spikes coincided with intentional turns, not the cause.

## Operational rules

1. **Don't trust global-costmap obstacles older than 4 s.** Cap obstacle-layer `observation_persistence` (currently unlimited) to ~4 s in `nav2_params_humble.yaml`.
2. **Reactive LiDAR via local costmap is the safety path;** global is for routing.
3. **Stationary drift 9–13× M4 baseline.** Try tightening map-EKF `odom0_pose_rejection_threshold: 4.0 → 3.0` and see if stationary rate falls toward 1 cm/s without losing fixes.
4. **Recalibrate Mandow `wheel_separation_multiplier`** on competition grass — current +7.5% scale error sits on the dead-reckon budget edge.
5. **GPS update rate** healthy at ~4 Hz; flag if it drops below 1 Hz (stationary rate would compound past 10 cm before correction).

---

Sources: `bags/{s2,s3,obs}_csv/{tf,cmd_vel,odometry_filtered,odometry_global,odometry_gps,gnss}.csv`. Top obs spike timestamps: 551.2, 552.2, 553.2, 597.99, 725.1 s (`parent=map ∧ child=odom`, 1 s buckets).
