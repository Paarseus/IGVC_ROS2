# Xsens mag-aided heading — staged fallback (2026-05-31)

Staged, **not applied**. Apply only if the powered drive-speed test (below) shows the current
`General_RTK` profile cannot converge its heading at real driving speed.

## Why this exists

Field check 2026-05-31 (bag `~/imu_heading_bags/heading_20260531_122911`, motors off, hand-push 2.97 m
under RTK FIXED):

- True heading (direction of travel, from GNSS **and** GPS-anchored EKF — agree): **−91.6° ENU ≈ 182° compass (due South).**
- Reported heading: **−57.3° → −67.2° ENU** (147° → 157° compass). Corrected ~10° toward truth, then
  **froze 24.5° short** the instant motion stopped.
- Diagnosis: `General_RTK` derives heading from GNSS course-over-ground; at the hand-push speed
  (~0.075 m/s, `wheel_odom` = 0 because motors were off) the course SNR was too low to converge. It
  locked **precisely but inaccurately** (+24.5°). **Not** the stuck-gyro-bias failure mode (that needs a
  USB power-cycle); the filter is healthy, just starved of course data.

## Decision gate — run this FIRST (free, reversible)

Powered straight drive under teleop at **~1–1.5 m/s for 5–10 m**, recording the same topics:

```bash
# on the Jetson, ROS env sourced + CycloneDDS:
ros2 bag record -o ~/imu_heading_bags/heading_drivespeed \
  /filter/euler /filter/quaternion /imu/data /imu/mag /imu/angular_velocity \
  /gnss /filter/positionlla /odometry/global /odometry/filtered /wheel_odom /tf
# drive straight at speed, then Ctrl-C, then:
python3 /tmp/analyze_heading.py ~/imu_heading_bags/heading_drivespeed
```

- **Converges to within a few ° of travel direction → KEEP General_RTK.** Do NOT apply this patch.
  Fix the startup error operationally: a short warm-up drive before trusting heading / sending
  map-frame goals.
- **Still lags >10–15° at speed → apply this patch** (mag is justified despite its risks).

## What the patch does

`switch_to_GeneralMag_RTK_plus_ICC.patch` edits `src/avros_bringup/config/xsens.yaml`:

| Param | Value | Effect |
|---|---|---|
| `enable_deviceConfig` | `false → true` | Master gate (`xdainterface.cpp:607-620`) — required to write ANY device config. **Revert to false after the flash boot.** |
| `enable_filter_config` | (new) `true` | Allows the filter-profile write (`:1220`). |
| `mti_filter_option` | (new) `2` | `GeneralMag_RTK` — fuses the magnetometer for heading. (0=General_RTK, 1=GeneralNoBaro_RTK, 2=GeneralMag_RTK.) |
| `enable_inrun_compass_calibration` | (new) `true` | Sets the onboard ICC option flag (`:932-955`) — onboard mag calibration, no PC/MT Manager. |

## Apply + flash procedure

```bash
cd ~/IGVC
git apply docs/xsens_mag_profile_2026_05_31_patches/switch_to_GeneralMag_RTK_plus_ICC.patch
colcon build --packages-select avros_bringup        # refresh installed config
source install/setup.bash
# Relaunch the stack (this is the ONE flash boot — driver writes profile + ICC flag to MTi firmware):
ros2 launch avros_bringup navigation.launch.py        # or sensors.launch.py to isolate the IMU
```

**Verify in the driver log** (these prove the flash took):
- `Onboard Kalman Filter Option: GeneralMag_RTK`
- `Enable In-run Compass Calibration Success!`
- `Optionflag InrunCompassCalibration is enabled.` (printed on the next boot too)

## ICC calibration drive

With the flag set, **drive the robot through varied headings** — several figure-8s / full loops, the
motion it'll actually do. The device self-calibrates the compass onboard and stores it. A straight line
will NOT calibrate it (ICC needs heading diversity).

## Revert `enable_deviceConfig` after flashing

The profile + ICC flag persist in MTi firmware. Stop re-flashing every boot:

```bash
# set enable_deviceConfig back to false in xsens.yaml (leave the other params — they're inert when false)
colcon build --packages-select avros_bringup && source install/setup.bash
```

## Rollback to General_RTK

```bash
# in xsens.yaml: enable_deviceConfig: true, enable_filter_config: true, mti_filter_option: 0
# (optionally enable_inrun_compass_calibration: false), relaunch ONCE to flash, then deviceConfig:false again.
```

## Confirm it worked

Re-run the decision-gate drive + `analyze_heading.py`. Success = reported heading tracks the travel
direction within a few degrees **at rest and at low speed**, with no large location-dependent bias
(watch for motor-load-dependent swing — the residual dynamic-distortion risk).
