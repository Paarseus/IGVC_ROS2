# 06 — GNSS and Global Localization

Path: MTi-680G internal u-blox receiver → `/gnss` (NavSatFix, 4 Hz) → `navsat_transform` → `/odometry/gps` (map frame) → global EKF (absolute x/y).
Corrections: NTRIP → EarthScope `PSDM_RTCM3P3` (MSM7 GPS/GLONASS/Galileo, 3.9 km baseline) → `/rtcm` → MTi.

## 1. Measured performance

### 1.1 Corrections and fix quality (2026-09-25)
| Condition | Result | Evidence |
|---|---|---|
| Caster link | `ICY 200 OK`, no reconnects, RTCM ~6 msg/s | session log |
| Time to first RTK FIXED, stationary, good sky | 677 s (11 min) | fix-wait log, session record |
| FIXED horizontal wander | 0.7 cm typical, 1.8 cm max (40 s) | session record |
| Mixed FLOAT/FIXED, 150 s static capture | FIXED 23% of samples; horizontal σ 10.8 cm E / 8.8 cm N; max radius 39 cm | `static_2026_09_25/SUMMARY.txt` |
| Receiver-reported horizontal σ in that capture | median 1.6 cm (range 1.4–15.2 cm) | same |
| Poor sky view (earlier spots) | mostly plain GPS, 1–3 m wander | session record |

### 1.2 Which point `/gnss` refers to (static check)
| Quantity | Value |
|---|---|
| Offset from MTi fused position to `/gnss` | 0.764 m, bearing −39.3° (ENU) |
| IMU heading at the time | −37.0° (ENU) |
| In the vehicle frame | **+0.764 m forward, −0.031 m left** |
| Configured lever arm | +0.740 m forward |

Source: `evidence/live_measurements/leverarm_check_2026_09_25.txt`, method `evidence/scripts/leverarm_check.py`.

## 2. Findings

**G1 — The GNSS antenna offset is not applied in the ROS pipeline (Accuracy, High, confirmed).**
The driver builds `/gnss` from `RawGnssPvtData`, the receiver's antenna solution (`gnsspublisher.h`), and stamps it `frame_id: imu_link`. The URDF has no antenna frame, so `navsat_transform` treats the antenna position as the IMU position (the IMU sits directly above `base_link`). The static check shows `/gnss` is 0.76 m ahead of the MTi's own lever-arm-corrected solution, exactly along the heading.

Effect on the map pose: `base_link` is reported ~0.75 m ahead of its true position, in whatever direction the robot faces. Driving toward a waypoint, the robot stops about 0.75 m short. A 180° turn moves the reported position by 1.5 m with no real motion. With RTK FIXED at ~1 cm, this is the largest error term in global localization by a factor of ~50.

Two standard fixes:
- (a) Add a `gnss_link` at (0.74, 0, h) under `base_link` in the URDF and publish `/gnss` with that `frame_id`. `navsat_transform` then applies the offset using the current heading (robot_localization supports this through the NavSatFix `frame_id`).
- (b) Use the MTi's fused, lever-arm-corrected position (`/filter/positionlla`, 100 Hz) instead of the raw PVT. It is smoother and already refers to the IMU, but its accuracy status must be taken from the status word, and it double-filters (see 05 §3).
Option (a) keeps the raw measurement and is the conventional robot_localization setup: `navsat_transform` removes the antenna offset for every fix through the `base_link → <NavSatFix frame_id>` transform (`getRobotOriginWorldPose()`, `navsat_transform.cpp` L564–603; `external/03` §5). The driver's `frame_id` parameter is shared with `/imu/data`, so the `/gnss` frame needs either a driver change or a small relay that rewrites it.

**G2 — Reported GNSS accuracy is optimistic when not FIXED (Estimation, High).**
The driver sets the covariance from the receiver's `hAcc` (`gnsspublisher.h`). In the static capture the receiver claimed a median 1.6 cm while the measured scatter was ~10 cm (1σ) and up to 39 cm, mostly in FLOAT. `navsat_transform` passes this through (`/odometry/gps` variance 4.84e-4 m², σ 2.2 cm), and the map EKF weights GPS accordingly. As a result the stationary robot's map pose moved 37 × 33 cm over 150 s, tracking the GNSS noise almost exactly (see 05).
Fix: inflate covariance by RTK state, e.g. FIXED: max(hAcc, 2 cm); FLOAT: max(hAcc × 5, 0.25 m); no RTK: max(hAcc × 3, 1.5 m). This can be done in the driver fork or in a relay node.

**G3 — FLOAT and FIXED are indistinguishable downstream (Estimation, Medium).**
The driver maps carrier-solution FIXED → `STATUS_GBAS_FIX` and FLOAT → `STATUS_SBAS_FIX`. `navsat_transform` and the EKF ignore status beyond fix/no-fix. Combined with G2, the filter gives FLOAT data (dm-level) the same weight as FIXED data (cm-level).

**G4 — Map-frame heading alignment is correct by configuration, untested by measurement (Accuracy, Medium).**
With `wait_for_datum: true`, `navsat_transform` does not read the IMU at all; the map orientation comes from the datum heading (0 → map +x = east) (`external/03` §5). That is consistent only if the EKFs' yaw is ENU referenced to true north. The Xsens manual states yaw is 0 at east and, in GNSS/INS profiles other than GeneralMag, referenced to true north without the magnetometer, so `magnetic_declination_radians: 0` and `yaw_offset: 0` are correct. What remains unmeasured is the residual offset after warm-up and the heading error before convergence (04, I2). A 1° error displaces the map position of a waypoint 100 m away by 1.7 m.
Test: V6 in 08 (drive straight under RTK FIXED, compare GNSS course with IMU yaw).

**G5 — Datum and frame configuration (Correct).**
`datum: [34.05930007, −117.82186044, 0.0]` uses the `[lat, lon, heading_rad]` format correctly (third value is heading, not altitude). `zero_altitude`, `broadcast_cartesian_transform: false` and `use_odometry_yaw: false` are consistent with the dual-EKF pattern. The waypoint conversion `/fromLL` matched an independent calculation (94.64, 42.35 m).

**G6 — Time to FIXED is long (Operations, Low).**
11 minutes stationary is slow for a 3.9 km baseline with 21–23 satellites. The session record ties the earlier failures to antenna placement. Remaining candidates are the antenna ground plane and the MTi's acceptance of the MSM7 stream. Cedarville (IGVC 2023) measured that a 15 cm-radius ground plane halved static GPS σ (`external/04`), a low-cost change worth making. The MTi status word (`rtk_status`) is available for a pre-mission gate.

## 3. Assessment
The corrections pipeline works and RTK FIXED delivers cm-level precision. Global localization accuracy is currently limited by integration, not by the sensor: a 0.75 m heading-dependent bias (G1) and a covariance that overstates FLOAT accuracy (G2). Both are configuration-level fixes with a direct test (08: V5, V7).
