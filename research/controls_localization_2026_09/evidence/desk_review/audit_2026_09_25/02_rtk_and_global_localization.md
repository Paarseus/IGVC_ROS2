# 02 — RTK GNSS and Global Localization (field accuracy, 2026-09-27)

Scope: MTi-680G (internal u-blox F9P, General_RTK) → `/gnss` → `navsat_transform` → `/odometry/gps` → map EKF → `map→odom` → Nav2 goals / global costmap (map frame). Builds on `../control_stack_analysis_2026_09/` (04, 05, 06, 08, 09). Deployed files: `deployed_snapshot/src/avros_bringup/config/`. Analysis script: `rtk_field_analyze.py` (this folder, bag-only, never publishes).

## Summary

- **The lever arm is set in the MTi, but it does not reach the ROS pipeline.** The device lever arm (0.74 m) moves only the MTi's fused output (`/filter/positionlla`) to the IMU. `navsat_transform` consumes `/gnss`, which is the raw F9P antenna fix. Result: the map pose of `base_link` is the antenna, 0.74 m ahead along the heading (audit G1 confirmed). Consequences: goals are reached about 0.74 m short, and after a 180° pivot obstacles in the map frame shift by up to 1.5 m, because the global costmap and the persistent semantic lane layer are both in the map frame.
- **New:** `base_link`/IMU is **not** at the chassis rotation centre. The footprint and URDF put the chassis centre about 0.27–0.31 m ahead of it. In a pivot, `base_link` itself moves on a circle of about 0.3 m radius, and the antenna on a circle of about 0.45 m (not 0.75 m). This corrects audit V5 ("base_link stationary") and makes the proposed "fuse vy = 0" (audit E2/O3) wrong at `base_link`.
- **New, measured from the 09-25 bag:** the dominant static error is a **35.6 cm step at FLOAT→FIXED**. The map EKF followed it within one 4 Hz update. FLOAT reported σ ≈ 14–15 cm before the first fix. FLOAT *after* FIXED held within 4 cm for 95 s at a reported σ of 1.4–2.2 cm. The audit's "6× optimistic" (G2) mixes these two states. The real hazard is FLOAT-before-first-FIXED plus the re-fix jump.
- **New, Probable:** `/gnss` is stamped about 92 ms old (70 ms minimum). robot_localization with `smooth_lagged_data: false` applies an old fix as if it were current, so while driving the map pose is pulled back by about v·70–100 ms (7–12 cm at 1.2 m/s).
- **Map heading:** it is aligned with true ENU. `/fromLL` for the test waypoint matches an independent ENU calculation to 0.001° (UTM meridian convergence is handled). The scale is 0.99968, which is harmless because GPS and goals use the same projection. Correction to the brief: with absolute GPS fusion, an IMU heading offset does **not** move where waypoints land. It rotates LiDAR obstacles about the robot in the map (17 cm at 10 m per 1°) and makes the robot crab between fixes.
- **The map EKF is not gated on RTK state.** `mission_manager` sends goals without checking `rtk_status`. The gate (13.8σ) is effectively off. `initial_estimate_covariance` is unset, so a launch more than 50 m from the datum ignores GPS for 26–105 s.

## Findings

| # | Finding | Sev. | Status | Evidence | vs audit |
|---|---|---|---|---|---|
| R1 | Device lever arm *is* active (flashed 05-30; `enable_deviceConfig: false` means it is simply not re-written each boot). It affects only MTi fused outputs. `/gnss` = raw `RawGnssPvtData` antenna fix, `frame_id imu_link`, and there is no antenna frame in the URDF, so navsat applies no offset. | High | Confirmed | `xsens.yaml:57,89`; driver `xdainterface.cpp:1146-1160` (lever arm written only if deviceConfig true); `gnsspublisher.h:65-73`; fused→raw offset 0.764 m fwd / −0.031 m left (`leverarm_check_2026_09_25.txt`); `navsat_transform.cpp` L564-603 | Confirms G1/I5, resolves the "is it flashed?" question |
| R2 | Effect of R1 on the map frame: the reported `base_link` = antenna, offset R(ψ)·(0.74, 0). Goals are reached with true `base_link` ~0.74 m short along the approach. Obstacles marked at heading ψ1 vs ψ2 are displaced by 1.48·sin(Δψ/2) m (1.05 m at 90°, 1.48 m at 180°) in the **global** STVL (5 s decay) and in the **persistent** global semantic layer (no decay). The 05-30 note "parks ~0.45 m short" was measured in this biased frame, so the true shortfall was probably ~1.2 m. | High | Confirmed (geometry) / Probable (goal history) | `nav2_params_igvc_autonav.yaml` global_costmap `global_frame: map`, `voxel_decay 5.0`, semantic `tile_map_decay_time 1e6`; L91 comment | Extends G1 (costmap impact new) |
| R3 | `base_link` (= IMU) is ~0.27–0.31 m **behind** the chassis centre and the likely ICR. Footprint front +0.813 / rear −0.279 gives centre +0.267; URDF chassis and tread boxes are at x = +0.3143. The URDF comment "base_link: chassis geometric center" and "Xsens at chassis center" contradict the geometry. In a pivot, v_y(base_link) = −ω·d ≈ 0.15 m/s at 0.5 rad/s. | Medium | Probable (ICR position needs test D) | `urdf/avros.urdf.xacro:8-9,43,71,128-131`; footprint L287 | **New.** Invalidates audit V5 pass criterion and E2/O3 "vy = 0" |
| R4 | FLOAT→FIXED re-fix produced a **35.6 cm** single-step jump. `/odometry/global` moved 35 cm between two consecutive outputs (t = 20.75→20.87 s). FIXED→FLOAT caused no step, and FLOAT-after-FIXED drifted ≤ 4 cm over 95 s. Cold FLOAT reported σ 14–15 cm while it was 36 cm off (≈ 2.5σ). | High | Confirmed (one 150 s bag) | `static_2026_09_25/gnss.jsonl.gz`, `odometry_global.jsonl.gz` (analysis in this session) | **New**; refines G2 ("6×" was a mixed-state artefact) |
| R5 | No RTK-state gating anywhere. The driver maps FIXED→`STATUS_GBAS_FIX`, FLOAT→`STATUS_SBAS_FIX`, plain → `STATUS_FIX`; navsat accepts every status except NO_FIX. `mission_manager` converts waypoints once and sends goals without checking `rtk_status`. | High | Confirmed | `gnsspublisher.h:76-95`; `mission_manager.py:34,141-153` | Extends G3 |
| R6 | Stale GPS stamps are applied to the current state: `/gnss` latency median 92 ms, p95 107 ms, min 69 ms, against IMU 26 ms. `processMeasurement` skips prediction when the measurement is older and just corrects (`smooth_lagged_data` is false by default and unset). Expected along-track lag ≈ v·(70–100 ms). | Medium | Probable (source + latency confirmed; magnitude needs test C) | `filter_base.cpp` processMeasurement (`delta > 0` → predict, else correct only); `SUMMARY.txt` gnss latency | **New** |
| R7 | GPS gate 13.8 is in σ units (χ² 190) and effectively off. Map EKF has no `initial_estimate_covariance`, so P = 1e-9. With Q_xy = 0.5 the first GPS fix at distance d from the start pose is accepted only after t ≈ 2·(d/13.8)² s: 10 m → 1 s, 50 m → 26 s, 100 m → 105 s. Until then the map pose sits at the origin. | Medium | Confirmed (source) | `ekf.yaml:194,209`; `filter_base.cpp` checkMahalanobisThreshold | Extends E4 with numbers for the current gate |
| R8 | Map EKF process noise Q_xy = 0.5 m²/s is very loose given wheel vx + IMU. The EKF reports σ = 29 cm while GPS is at 1.4 cm, so it follows each fix with gain ≈ 1 (stationary span 37 cm = GNSS span). Lowering it is correct **only after R1/R4/R5 are fixed**; otherwise it just trades jumps for lag. | Medium | Confirmed | `ekf.yaml:209`; `SUMMARY.txt` pcov 0.086 | Same as E1, with ordering constraint |
| R9 | Map orientation is correct. `/fromLL`(34.059682, −117.820835) = (94.64, 42.35) vs independent WGS84 ENU (94.669, 42.365): rotation 0.001°, scale 0.99968 (UTM k at 75 km from the CM). Convergence (≈ −0.46° here) is applied correctly. Datum `[lat, lon, heading_rad]` is correct. `yaw_offset`/`magnetic_declination` 0 is correct for General_RTK (true north, no magnetometer). These params *are* used: with `wait_for_datum` the datum heading goes through the same correction (`navsat_transform.cpp` L291). | — | Confirmed | computed this session; `navsat.yaml:16-17,40` | Confirms G4/G5; corrects E9 |
| R10 | The residual IMU-vs-GNSS-course heading offset after warm-up is still unmeasured. It does not affect waypoint landing (R9), but it rotates LiDAR obstacles in the map (1° → 17 cm at 10 m) and makes the robot crab between fixes. | Medium | Gap | — | = G4/I2 |
| R11 | Time to FIXED is 11 min, and FIXED→FLOAT toggling happens in clear sky while static (24 FIXED / 131 FLOAT in 40 s). Candidates: antenna ground plane or multipath from the mast/Velodyne; RTCM gaps; GLONASS inter-frequency biases with a non-u-blox base (1230 is sent, so this is less likely). Not yet correlated with `/rtcm` gaps or GGA age. | Medium | Hypothesis | session record; G6 | Extends G6 with a test |
| R12 | RTCM set 1005/1077/1087/1097/1107/1230 suits the F9P. 1107 (SBAS MSM7) is not an F9P RTK input and is ignored (harmless). No 1127 from the base, so `enable_beidou: false` loses nothing for RTK. GGA upload at `update_rate 1.0` Hz is harmless for a single-base mount. | Low | Probable (F9P list from u-blox docs, not re-read today) | `ntrip_params.yaml:110-115`; `ntrip_client.cpp:129-136` | New (answers brief item 5) |
| R13 | NTRIP credentials (EarthScope and MDOT) are stored in plain text in a git-tracked YAML. | Low | Confirmed | `ntrip_params.yaml:113-114,130-131` | New (not accuracy) |
| R14 | Stale or incorrect comments: `navsat.yaml:14-15` ("uses WMM declination"; General_RTK is true north from GNSS), `:20` ("use IMU yaw directly": the IMU is not subscribed with a manual datum), `:42` (TF loop); `ekf.yaml:7,19,188` (differential, "6σ²"); CLAUDE.md "lever arm [0,0,0] TODO", "4σ gate". | Low | Confirmed | files cited | Extends item 22 |

## Field tests — outdoors today

Common setup for all tests:
- `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml`
- Stack: `navigation.launch.py` for test E; `localization.launch.py` plus teleop/webui for A–D.
- No RViz on the Jetson.
- Record every test:
  ```bash
  ros2 bag record -o rtk_<test>_$(date +%H%M) /gnss /filter/positionlla /status /nmea /rtcm \
    /imu/data /odometry/gps /odometry/global /odometry/filtered /wheel_odom /cmd_vel /tf /tf_static
  ```
- Live RTK state: `ros2 topic echo /status --field rtk_status` (0 none / 1 FLOAT / 2 FIXED).
- Validity rule: only data with `rtk_status == 2` counts as truth. Truth point = `/filter/positionlla`, which is the IMU point, horizontally the same as `base_link`.
- Analysis: `python3 rtk_field_analyze.py <bag> <mode>`.
- Physical marks: tape a plumb-bob mark under the **IMU** (base_link) and one under the **antenna**, then measure the antenna-to-IMU distance with a tape. That gives a third value for the lever arm, next to 0.74 m configured and 0.764 m measured.

### A. Static RTK repeatability and time-to-FIXED at the test spot (15 min, first)
- **Purpose:** establish truth quality at this spot, the FIXED/FLOAT behaviour, whether drops correlate with RTCM gaps, and the size of the map jump on re-fix.
- **Procedure:**
  1. Cold start: power-cycle the MTi (USB) so the fix is truly cold, launch, and start recording immediately.
  2. Do not move for 12 min.
  3. Optional R4 probe: at minute 12, cover the antenna with a metal pot for 15 s, uncover it, and wait for re-FIX (3 min).
- **Analysis:** `rtk_field_analyze.py <bag> static` (and `--t0/--t1` around the pot event).
- **Pass:**
  - time to first FIXED < 3 min (known 11 min → fail = ground-plane/multipath work item)
  - FIXED ≥ 95 % after the first FIX
  - FIXED σ ≤ 1.5 cm, max radius ≤ 4 cm
  - FLOAT measured/reported σ ≤ 2
  - no FIXED→FLOAT drop without a `/rtcm` gap > 2 s (if it fails → sky/multipath, R11)
  - Record the re-fix step size in `/odometry/global` and `map→odom`. Anything > 10 cm is a costmap-smear event (R4).

### B. Known-point return (20 min)
- **Purpose:** absolute repeatability of truth and of the EKF pose, and a direct measurement of R1 as a heading-dependent bias.
- **Procedure:**
  1. Park over mark M (plumb under the IMU) facing north for 30 s.
  2. Drive a ~30 m loop, return, park over M facing north, 30 s.
  3. Repeat, parking facing south (180°), 30 s.
  4. Repeat facing east, 30 s.
  5. Tape-measure the plumb-to-M offset each time (fwd/left, ±1 cm).
- **Analysis:** `rtk_field_analyze.py <bag> stops --mark <lat lon of M from stop 1 truth>`.
- **Pass:**
  - truth vs tape agree within 3 cm
  - truth repeatability between stops ≤ 3 cm
  - EKF−truth in the vehicle frame: currently expect ≈ +0.74 fwd at every heading, so a north-vs-south world-frame difference of ~1.48 m. After the R1 fix: ≤ 5 cm at every heading.
  - antenna offset reported by `stops` = 0.74 ± 0.03 fwd, 0 ± 0.03 left.

### C. Straight line, both directions: heading offset, lever arm, GPS lag (25 min)
- **Purpose:** R10 heading offset, R6 lag, R1 along-track bias, and a map-vs-ENU rotation cross-check.
- **Procedure:**
  1. Warm up the heading with one 10 m straight drive first, which is discarded (known quaternion warm-up issue).
  2. Mark a 30 m straight line (two stakes).
  3. Teleop straight at 0.5 m/s: 2 passes out and back.
  4. Repeat at 1.0 m/s: 2 passes out and back.
  5. Also drive 2 passes in **reverse** at 0.4 m/s, so v < 0 separates the lever term from the lag term.
  6. Keep |ω| small and let heading-hold run.
- **Analysis:** `rtk_field_analyze.py <bag> line`. Per segment it reports course − IMU yaw, the map-vs-ENU rotation, and EKF along/cross error; the final line fits `err = a − v·τ`.
- **Pass:**
  - mean course − yaw |·| < 0.5°, same sign/size in both directions within 0.5° (a direction-dependent value means mount yaw or IMU error)
  - map-vs-ENU rotation < 0.1°
  - fit `a` ≈ +0.74 now (≤ 0.05 after R1 fix)
  - τ < 30 ms after enabling lagged smoothing (expect 70–100 ms now)
  - cross-track EKF error ≤ 5 cm
- **Action:** a consistent heading offset > 0.5° is **not** a `yaw_offset` fix (that would rotate the map). Treat it as IMU mount yaw / Xsens alignment (`MTi` alignment rotation) and re-check.

### D. Rotate in place: ICR and lever arm (10 min)
- **Purpose:** measure where the pivot centre is relative to `base_link` (R3) and confirm the lever arm independently of heading.
- **Procedure:**
  1. RTK FIXED, on pavement.
  2. Pivot 2 full turns CCW at 0.3 rad/s, stop 10 s.
  3. Pivot 2 full turns CW, stop 10 s.
  4. Repeat once at 0.6 rad/s.
- **Analysis:** `rtk_field_analyze.py <bag> spin`, which gives circle fits of the IMU point, the antenna and `/odometry/global`, plus the ICR in the body frame.
- **Pass / record:**
  - antenna − IMU offset = 0.74 ± 0.03 m
  - ICR forward offset d reported (expect +0.27…0.31). If |d| > 0.1 m, R3 is confirmed.
  - `/odometry/global` circle radius now ≈ antenna radius; after the R1 fix it should equal the IMU-point radius (≈ d), **not 0**.
  - Also note the CW vs CCW centre shift (track slip asymmetry).

### E. Short GPS waypoint goal accuracy (20 min, after A–D)
- **Purpose:** end-to-end error, split into localization error and controller stopping error.
- **Procedure:**
  1. Park at point W (~20 m from start) and average truth for 30 s. That lat/lon is the goal; put a stake under the IMU plumb.
  2. Drive ~20 m away and face W.
  3. Convert with `ros2 service call /fromLL robot_localization/srv/FromLL "{ll_point: {latitude: <lat>, longitude: <lon>, altitude: 0.0}}"` (read-only).
  4. Send the goal directly (bypass mission_manager, whose 2 m acceptance radius cancels early): `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: X, y: Y}, orientation: {w: 1.0}}}}"`
  5. When it stops, tape-measure plumb → stake (along/cross).
  6. Do 3 approaches from different directions.
- **Analysis:** `stops --mark <W>`, which gives EKF−truth (localization error) and truth−mark (total error).
- **Pass:**
  - localization error ≤ 5 cm after fixes; today expect ≈ 0.74 m along the approach direction
  - truth−mark ≤ `xy_goal_tolerance` (0.5 m) + 0.05 m
  - tape and `stops` agree within 3 cm

## Analysis script — `rtk_field_analyze.py`

- **Input:** a rosbag2 directory (sqlite3 or mcap).
- **Reads:** `/gnss /filter/positionlla /status /imu/data /odometry/{global,gps,filtered} /tf /rtcm`.
- **Safety:** creates no node and never publishes.
- **Frame:** local ENU about the navsat datum (WGS84 radii; ≤ 1 mm error at 100 m). This is directly comparable to the map frame per R9.
- **Modes:**

| Mode | Outputs |
|---|---|
| `static` | TTFF; per-state n/σ/max radius/reported σ/ratio; each state transition with step size; `/rtcm` max gap and gaps > 2 s; `/odometry/global` and `map→odom` span plus max single step |
| `stops` | auto stationary segments ≥ 6 s; FIXED-only truth mean, heading, antenna offset in body frame, EKF−truth in body frame, truth−mark |
| `line` | auto straight segments; course−IMU yaw, map-vs-ENU rotation, EKF along/cross error, least-squares fit of lever residual `a` and lag τ |
| `spin` | Kasa circle fits (IMU point, antenna, EKF), ICR in body frame, implied lever arm |

- **Not yet included, easy to add:** GGA parse of `/nmea` (quality, sats, HDOP, diff age) and NIS of GPS updates.
- **Limitations:**
  - `/status` has no header, so receive time is used (~25 ms skew).
  - `/filter/positionlla` is the MTi's own fused solution. It is truth only while FIXED and assumes the device lever arm is right, which B, D and the tape cross-check.

## Recommended config changes (proposals — not applied)

Order matters: 1 → 2 → 3, then re-run A–E.

1. **Apply the antenna offset in ROS (R1)**, as a small relay node `gnss_relay`:
   - Subscribes to `/gnss` and `/status`.
   - Publishes `/gnss/fix` with `frame_id: gnss_link`.
   - Add URDF joint `base_link → gnss_link` at `xyz="0.74 0.0 <antenna height>"`, and set x from tests B/D/tape (use the measured value, e.g. 0.76, if it differs by > 2 cm).
   - Remap navsat `('gps/fix', '/gnss/fix')` in `localization.launch.py:143`.
   - Alternative: convert `/filter/positionlla` (already at the IMU, 100 Hz) to NavSatFix in the same relay. It is smoother, but it double-filters and has no covariance.
2. **Covariance and gating by RTK state, in the same relay (R4/R5).**
   - Set σ from `rtk_status` plus hAcc:

     | State | σ |
     |---|---|
     | FIXED | max(hAcc, 0.02 m) |
     | FLOAT, < 120 s since last FIXED | max(3·hAcc, 0.10 m) |
     | FLOAT, cold | max(3·hAcc, 0.50 m) |
     | no RTK | max(2·hAcc, 2.0 m) |
     | NO_FIX | drop |

   - Add a `mission_manager` start gate: `rtk_status == 2` continuously for 60 s, and `‖/odometry/global − /odometry/gps‖ < 0.1 m`, before sending the first goal.
   - If `rtk_status < 2` for > 10 s mid-run: continue on the odom EKF but log it and cap speed (policy decision for the team).
3. **Map EKF (`ekf.yaml`, `ekf_filter_node_map` only):**
   - `smooth_lagged_data: true`, `history_length: 1.0` (R6). CPU cost: watch the 27 Hz rate (E7).
   - `initial_estimate_covariance`: x, y = 100.0 (other diagonals keep defaults 1e-9 → set yaw 0.1, vx 0.1) (R7).
   - `odom0_pose_rejection_threshold: 5.0` (σ). This is conservative because GNSS errors are time-correlated; do not use 3.72 until NIS is checked.
   - Process noise x, y 0.5 → **0.05** (R8). Re-tune from test A/C residuals.
   - Do **not** add vy = 0 at `base_link`. Either move `base_link` to the measured ICR (URDF + footprint + wheel odom; large change) or have `/wheel_odom` publish vy = −ω·d with σ 0.05 m/s (R3; the kinematics owner decides).
4. **navsat.yaml:** leave the numbers as they are (R9 verified). Fix the comments (R14). `use_local_cartesian: true` would remove UTM scale and convergence from the chain; this is optional and not needed.
5. **Global costmap:** keep the persistent semantic layer, but gate it on RTK FIXED + R1 applied, or accept up to 1.5 m smear after pivots today (R2).
6. **NTRIP:** move credentials to an untracked override file or env (R13). Keep the mount. For R11, first run test A with a 15 cm-radius ground plate under the antenna.

## Sources

- Xsens driver (vendored): `/home/mspacman/IGVC_ROS2/src/xsens_mti/src/xsens_mti_ros2_driver/src/messagepublishers/gnsspublisher.h`, `positionllapublisher.h`, `xdainterface.cpp:604-615,1146-1215`, `msg/XsStatusWord.msg`; `src/xsens_mti/src/ntrip/src/ntrip_client.cpp`
- robot_localization humble-devel: https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/navsat_transform.cpp (datum L148-190, heading correction L255-300, `setTransformGps` L840-880), https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/filter_base.cpp (`processMeasurement`, `checkMahalanobisThreshold`), https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/ros_filter.cpp (L877-898 `sensor_timeout`, `smooth_lagged_data`, `history_length`)
- Xsens MTi Family Reference Manual: https://www.xsens.com/hubfs/Downloads/Manuals/MTi_familyreference_manual.pdf (ENU yaw = 0 east; true-north heading without magnetometer except GeneralMag)
- Xsens MTi-680G: https://www.xsens.com/sensor-modules/xsens-mti-680g-rtk-gnss-ins ; MT Low-Level Protocol: https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf
- u-blox ZED-F9P Integration Manual (supported RTCM inputs; UNVERIFIED today): https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf
- Nav2 GPS tutorial: https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
- Audit inputs: `../control_stack_analysis_2026_09/06_gnss_localization.md`, `05_state_estimation.md`, `04_imu_xsens.md`, `08_validation_plan.md`, `evidence/live_measurements/static_2026_09_25/`, `leverarm_check_2026_09_25.txt`
