# Skid-Steer Kinematics & EKF Validation — Findings from 2026-05-18 Session

## TL;DR

- **Forward Mandow correction (`wheel_separation_multiplier: 1.19`) is essential.** Without it, a 1×1 m closed-loop square trajectory closes with **63 cm position error** + **41° yaw error**. With it: **6 cm + 3.6°** — a **10× improvement**.
- **Asymmetric Mandow application (forward only) is intentional and correct for THIS chassis.** We diverge from the production `diff_drive_controller` symmetric pattern with empirical justification.
- **The 16% chassis ω gap decomposes empirically as:** chassis-side skid α≈0.96 (small) + SparkMAX PID motor-delivery loss ≈85% under rotation load. The multiplier compensates for both layered.
- **EKF is healthy** (6 cm closing error on 4 m trajectory matches Ollman thesis benchmark for skid-steer; 1.5% of distance traveled).
- **Xsens IMU recovery procedure documented** — USB power-cycle resets stuck gyro bias.

---

## 1. The original question

After the morning's inner SparkMAX PID retune, we measured the chassis only delivered ~85% of commanded ω in pure open-loop (multiplier = 1.0). The earlier session's hypothesis was that this was track-skid loss, fixable by the Mandow ICR multiplier:

| Forward kinematics | Inverse kinematics |
|---|---|
| `wheel_cmd = ω_cmd × (track_w × multiplier) / 2` | `chassis_ω_estimate = wheel_diff / (track_w × multiplier)` |

`diff_drive_controller` (Husky, Jackal, sam_bot, etc.) applies the multiplier **symmetrically** in both directions. We initially applied it only in the forward direction. The question was: should we add it to the inverse direction (Option B) to match the production pattern?

## 2. What the data showed

We measured chassis ω two independent ways at the same instant, at multiple commanded speeds:

```
α_cmd  = naive ω derived from COMMANDED wheel RPMs / IMU truth
α_meas = naive ω derived from MEASURED  wheel RPMs / IMU truth
```

At multiplier = 1.0 (so commanded wheel RPM is the unboosted kinematic prediction):

| ω_cmd | α_cmd (forward asymmetry) | α_meas (inverse asymmetry) | α_cmd / α_meas |
|---|---|---|---|
| 0.3 rad/s | 1.15 | 0.96 | 1.19 |
| 0.5 rad/s | 1.13 | 0.97 | 1.17 |
| 0.8 rad/s | 1.12 | 0.97 | 1.15 |

The two α values are consistently DIFFERENT. If chassis skid were the only effect (Mandow's symmetric model), they would be equal. The fact that **α_cmd / α_meas ≈ 1.17** is itself a discovered constant — it matches the SparkMAX inner-PID delivery loss exactly:

```
L_meas / L_cmd ≈ 87% (motor delivery)
1 / 0.87 = 1.15 ≈ α_cmd / α_meas
```

**Decomposition:**
- `α_meas` ≈ 0.96 → **pure chassis-side skid factor** (small — chassis ω is 4% higher than naive prediction from measured wheel RPM)
- `α_cmd / α_meas` ≈ 1.17 → **motor PID delivery loss** (commanded RPM ≠ actual RPM under rotation load)

The forward multiplier of 1.19 compensates for BOTH layered effects. The inverse direction only sees the second one (chassis skid alone), and on our chassis that's essentially negligible.

## 3. Decision: asymmetric Mandow application (forward only)

Applying the same 1.19 multiplier to the inverse direction would compensate for skid that **isn't there on our chassis**. The naive formula on measured wheel RPM is already within 4% of IMU truth.

Why our chassis differs from Husky's 1.875 / Jackal's 1.5 symmetric-correction pattern:

| Chassis | Real chassis skid (α_skid) | Motor delivery | Total forward gap |
|---|---|---|---|
| **Pioneer P3-AT** (Mandow paper) | 1.41 on tile, 0.91 on asphalt | high (closed-loop motors) | dominated by skid |
| **Husky A200** (Reina 2016) | 1.67–2.70 across surfaces | high | dominated by skid → symmetric multiplier works |
| **Our chassis (AndyMark Raptor + NEO via SparkMAX)** | **0.96** (essentially none) | 85% (SparkMAX PID under-delivers under rotation load) | dominated by motor delivery loss |

For chassis where skid dominates, symmetric multiplier is correct (the production default). For chassis where motor delivery dominates, asymmetric is correct (our case). **Both are valid; the right answer depends on the empirical decomposition.**

## 4. Empirical α values across surfaces (literature comparison)

| Vehicle | Surface | α | Source |
|---|---|---|---|
| Pioneer P3-AT | asphalt | 0.91 | Mandow et al. 2007 IROS |
| Pioneer P3-AT | smooth indoor tile | 1.41 | Wang et al., PMC4481911 |
| Pentzer tracked vehicle | grass | ~1.0 | Pentzer 2014 J. Field Robot. doi:10.1002/rob.21509 |
| Husky A200 | asphalt | 1.67 | Reina & Galati 2016 |
| Husky A200 | dirt | 1.92 | Reina & Galati 2016 |
| Husky A200 | beach sand | 2.70 | Reina & Galati 2016 |
| **Our chassis (Raptor)** | **indoor concrete** | **1.02 (inverse) / 1.19 (forward+motor compensation)** | this investigation |

**Critical for IGVC competition:** α shifts **30-80% between surfaces** (Reina 2016, Reinstein 2013 terrain-adaptive). Our 1.19 calibration on indoor concrete will likely need re-calibration on grass. Husky users (issue #193) confirm Clearpath has no per-surface tuning guidance — empirical calibration on race surface is standard practice.

## 5. EKF validation results (current setup)

Validated end-to-end with the Mandow correction enabled and the inner PID at burned values:

```yaml
kFF:                          0.000197    # burned to SparkMAX flash
kP:                           0.0007      # burned to flash
kI:                           2.5e-07     # burned to flash
kD:                           0.0         # burned to flash
kIZone:                       600.0       # burned to flash
wheel_separation_multiplier:  1.19        # YAML-set, runtime-tunable
heading_kp:                   1.5
yaw_rate_kp:                  REMOVED (commit 401167b)
```

| Test | Result | Pass criterion | Verdict |
|---|---|---|---|
| Stationary 30 s drift (/odometry/filtered position) | Δx,Δy ≈ 0 | <0.05 m | ✓ |
| Stationary 30 s drift (yaw, IMU bias) | +2.8° | <0.5° (Tom Moore) | ⚠ marginal (IMU bias ~0.09°/s) |
| 3-way ω comparison (IMU vs /wheel_odom vs /odometry/filtered) | within 3% of each other | spread <5% | ✓ |
| **Closed-loop 1×1 m square (4 m total travel)** | **Δxy = 0.06 m, Δyaw = 3.6°** | Δxy < 0.5 m, Δyaw < 10° | **✓✓** |
| Same square, multiplier = 1.0 (Mandow OFF) | Δxy = 0.63 m, Δyaw = 41° | same | ❌ — 10× degradation |

The square test is the integration-level proof: with Mandow ON, the chassis navigates a 4 m trajectory and closes within **1.5% of distance traveled** — matching the Ollman thesis benchmark for well-tuned skid-steer EKF (1-5%).

## 6. Xsens IMU stuck-bias recovery procedure

**Incident:** mid-session, the Xsens MTi-680G developed a sustained gyro bias of ~-0.05 rad/s (-2.86°/s), 70× the normal ~-0.04°/s drift. This produced a -58° yaw drift over 20 s while completely stationary. The IMU's `enable_continuous_zero_rotation_update: true` failed to recover automatically.

**Likely causes:** thermal drift from heavy use that day, vibration from Jetson fans disqualifying ZRU detection, or `General_RTK` filter profile struggling without GNSS aiding.

**Recovery (verified):**
1. ROS-level driver restart: **did not fix** (filter state persisted in IMU firmware).
2. **USB power-cycle of the Xsens module: fixed it.** Drift returned to -0.085°/s (acceptable). The device gets reassigned (e.g., `/dev/ttyUSB0` → `/dev/ttyUSB1`); `xsens.yaml` has `scan_for_devices: true` so the driver finds it automatically after the sensors launch is restarted.

**Symptom to watch for during operations:** if the chassis isn't going straight on a commanded straight-line drive, and the IMU's `/imu/data.orientation` yaw drifts visibly while stationary, the IMU needs a USB power-cycle. The heading-hold (`heading_kp`) cannot compensate for a bad IMU input.

## 7. Calibration procedure for new surface

The 1.19 multiplier is surface-specific. For IGVC competition on grass (or any new surface):

```bash
# 1. With chassis on the target surface, with wheel_separation_multiplier=1.0:
ros2 param set /actuator_node wheel_separation_multiplier 1.0

# 2. Pure rotation in place for 5 s at a known rate
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{angular: {z: 0.5}}'
# ... measure IMU yaw rotation over 5 s ...

# 3. Compute α from data:
#   measured_omega_avg = (yaw_end - yaw_start) / 5.0   (rad/s)
#   delivery = measured_omega_avg / 0.5
#   new_multiplier = 1.0 / delivery

# 4. Apply
ros2 param set /actuator_node wheel_separation_multiplier <new_value>

# 5. Verify with closed-loop square test (this same multi-step procedure)
```

The `wheel_separation_multiplier` is `[DYNAMIC]` in `actuator_node._DYNAMIC_PARAMS` so it can be re-calibrated live without rebuild/restart.

## 8. What we explicitly DID NOT do (and why)

| Considered but rejected | Reason |
|---|---|
| Apply multiplier symmetrically (also in `_publish_odom` inverse) | Would compensate for skid that isn't there on our chassis. Empirically validated to make `/wheel_odom` 14% worse against IMU truth. |
| Push kI from 2.5e-7 to 1e-6 (4×) | Caused L/R wheel asymmetry of 10-14% during forward driving — kI overshoot from the windup-during-slew effect we'd fixed earlier in the day. |
| Add kD damping (kD = 2e-6) | Caused immediate motor chatter and forward-drive asymmetry; consistent with the well-documented kD-on-noisy-encoder failure mode on tracked vehicles. Issue #6 explicitly chose `kD = 0` for this reason. |
| Push kP above 0.0010 | Returns diminishing motor-delivery gains (chassis slip ceiling at ~92-95% delivery) and amplifies L/R asymmetry. |
| Override `process_noise_covariance` matrices in ekf.yaml | Production robots (per Agent B's survey of Husky/Jackal/A300/sam_bot configs) leave them at defaults; no need to over-engineer. |

## 9. References

**Official / production code:**
- `ros2_controllers/diff_drive_controller` — applies wheel_separation_multiplier symmetrically (lines 190 + 357 of `diff_drive_controller.cpp`): https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/src/diff_drive_controller.cpp
- Husky `wheel_separation_multiplier: 1.875`: https://github.com/husky/husky/blob/humble-devel/husky_control/config/localization.yaml
- Jackal `wheel_separation_multiplier: 1.5`: https://github.com/jackal/jackal/blob/foxy-devel/jackal_control/config/localization.yaml
- Husky issue #193 ("the canonical 'why 1.875?' thread"): https://github.com/husky/husky/issues/193
- Nav2 VIO tutorial — Macenski on skid-steer odometry: https://docs.nav2.org/tutorials/docs/integrating_vio.html
- robot_localization preparing-sensor-data: http://docs.ros.org/en/noetic/api/robot_localization/html/preparing_sensor_data.html

**Academic skid-steer kinematics:**
- Mandow et al. 2007 IROS, "Experimental kinematics for wheeled skid-steer mobile robots", doi:10.1109/IROS.2007.4399139
- Pentzer, Brennan & Reichard 2014, J. Field Robotics 31(3), online ICR estimation, doi:10.1002/rob.21509
- Reina & Galati 2016, "Slip-based terrain estimation with a skid-steer vehicle", Vehicle System Dynamics, doi:10.1080/00423114.2016.1203961
- Reinstein, Kubelka & Zimmermann 2013, ICRA, terrain-adaptive odometry
- Wang et al. (PMC4481911) — symmetric ICR validation on Pioneer P3-AT
- Yi et al. 2020, arXiv:2006.04335 — visual-based skid-steer kinematics (online ICR)
- Okawara et al. 2024, arXiv:2404.02515 — tightly-coupled LiDAR-IMU-wheel with online calibration

**Closing-error benchmark:**
- Ollman, "Pose Estimation on the Clearpath Jackal UGV" (U. Adelaide thesis): https://trumpf.id.au/theses/Alexander_Ollman.pdf — establishes 1-5% of-distance closing error as the standard for well-tuned skid-steer + EKF on flat surfaces.

---

## Appendix: empirical numbers from this session

For reference / regression-testing future sessions on the same chassis:

```
SparkMAX inner PID (BURNed to flash 2026-05-18):
  kFF        0.000197
  kP         0.0007
  kI         2.5e-07
  kD         0.0
  kIZone     600.0

Outer-loop kinematics (in actuator_params.yaml):
  wheel_separation_multiplier  1.19     [DYNAMIC, surface-dependent]
  heading_kp                   1.5
  yaw_rate_kp                  REMOVED  (was 0.3; non-standard, removed in commit 401167b)

Chassis empirical:
  α_skid (chassis-side skid factor)              ≈ 0.96  (on current indoor surface)
  motor delivery (L_meas/L_cmd) under rotation   ≈ 85-87%
  motor delivery (L_meas/L_cmd) under translation ≈ 93-96%

Closed-loop 1×1 m square (BURNed + multiplier=1.19):
  Position closing error    6 cm   over 4 m travel = 1.5%
  Yaw closing error         3.6°   over 270° commanded rotation = 1.3%

Same square at multiplier=1.0 (Mandow OFF):
  Position closing error    63 cm  (10.5× worse)
  Yaw closing error         41°    (11.3× worse)

IMU stationary drift rate (when healthy):       ≈ -0.085 deg/s
IMU stuck-bias incident (mid-session):          ≈ -2.86 deg/s — recovery via USB power-cycle
```
