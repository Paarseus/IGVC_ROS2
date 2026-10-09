# Phase 0 Results: 2026-09-27

**Session:** robot parked outdoors, motors idle. Run 1: 11 min, valid. Runs 2–3: stopped when the robot was moved. Run 4: 25 min, robot turned twice (only its still stretches are used). Run 5: after a Jetson reboot (fresh GPS receiver start), in progress.
**Code on robot:** `f242825` (new Teensy firmware). IMU filter profile: `General_RTK`.
**Raw data:** Jetson `~/field_2026_09_27/phase0/bag_static_10min` (90 MB), metrics in `phase0_metrics.json`, full table in `phase0_table.md`.

## Summary

| Area | Result | Verdict |
|---|---|---|
| Power and firmware | 12.22 V at idle, new firmware running | Good |
| Jetson | Clock synced, load 0.4–1.2, 50 °C, 843 GB free | Good |
| RTK GPS | **Never reached FIXED** in about 45 min over three spots. 10–15 satellites used (Friday's FIXED spot, about 4 m away: 19–23). FLOAT position wandered up to 2.1 m. | **Blocking** |
| Corrections stream | Live, complete, same base station as Friday (ID 1426, 3.9 km); login accepted | Good |
| IMU sensor | Gyro bias ≤ 0.06 °/s, gravity 9.795 m/s², level within 1.4° | Good |
| IMU heading | Right after start-up: swings up to ±9° per minute. **After the robot has been driven: steady within ±0.26° per minute.** | Good after warm-up drive |
| Wheel odometry | Zero movement while parked | Good |
| Local position filter | Zero position drift; heading follows the IMU wander (−2.7° net) | Follows IMU |
| Position filter rate | 24.6 Hz (run 1), 21 Hz (run 4); configured 30 Hz | Open question |
| Message delay | IMU 38 ms; GPS 79 ms (run 1), 112 ms (run 4) | GPS late |

**Phase 0 is not complete.** The RTK and antenna checks need a FIXED position, and the robot never got one. Everything that doesn't depend on RTK is measured.

## Details

### 1. Power, firmware, Jetson
| Measurement | Result | Target | Pass |
|---|---|---|---|
| Battery voltage at idle | 12.22 V | ≥ 12.0 V | PASS |
| Firmware ramp value | `M=100` | present | PASS |
| Motor mode at idle | velocity mode | `VEL` | PASS |
| Clock synchronized | yes | yes | PASS |
| CPU load / temperature / free disk | 0.4 (1.2 during recording) / 49–53 °C / 843 GB | < 2 / < 70 °C / > 50 GB | PASS |

### 2. RTK GPS
| Measurement | Result | Target | Pass |
|---|---|---|---|
| Time to first FIXED | never (11.7 min after corrections started) | < 3 min | **FAIL** |
| Fix type during recording | FLOAT 93 %, plain GPS 7 % | FIXED ≥ 95 % | **FAIL** |
| Satellites used / HDOP | 10–12 / 0.9 | ≥ 15 / ≤ 1.2 | **FAIL** / PASS |
| FLOAT position spread | 62 cm typical, 2.1 m largest | — | — |
| Plain GPS position spread | 1.1 m typical, 1.5 m largest | — | — |
| Accuracy the receiver reports vs measured (FLOAT) | 55 cm vs 62 cm | within 2× | PASS |
| Correction messages / longest gap | 5.3 per s / 1.9 s | steady / ≤ 2 s | PASS |

**What it means:** the corrections arrive fine, so the internet link and the EarthScope account work. The receiver used only 10–12 satellites, about half of Friday's clear-sky spot. That points to the sky view at this location (buildings, trees, or something on the robot blocking the antenna), not the NTRIP setup. The receiver's own accuracy estimate is honest in FLOAT (55 vs 62 cm), so the position filter isn't being misled; it just has a poor input.

### 3. Antenna offset
**Not measured.** It needs a FIXED position; FLOAT noise (60 cm) is as large as the 0.74 m being checked. It will be measured once RTK is FIXED.

### 4. IMU
| Measurement | Result | Target | Pass |
|---|---|---|---|
| Gyro bias x / y / z | +0.005 / +0.057 / −0.007 °/s | < 0.05 °/s | FAIL (y only, by 0.007) |
| Gyro noise x / y / z | 0.12 / 0.14 / 0.10 °/s | record | — |
| Heading change each minute | +9.1, +4.0, −7.8, +1.6, −1.3, −0.4, −4.2, −1.5, +0.6, −2.7 ° | < 0.2 °/min | **FAIL** |
| Heading change the gyro bias explains | −4.5 ° over the recording (−0.4 °/min) | — | — |
| Roll / pitch | +1.40° / −0.15° | record | — |
| Gravity | 9.795 m/s² | 9.81 ± 0.05 | PASS |
| Covariance values the IMU reports | 0 / 0 / 0 | non-zero | **FAIL** (known issue) |

**What it means:**
- The gyro itself is healthy: small bias, normal noise. The y-axis bias is pitch rate and doesn't affect heading.
- The heading wander comes from the Xsens filter, not the gyro. It swings several degrees per minute in both directions, while the gyro alone would drift −0.4°/min.
- With the `General_RTK` profile, heading is corrected only from GPS movement. The likely cause is the filter reacting to the FLOAT position noise (up to 2 m) as if the robot were moving. That is a hypothesis; it will be confirmed or ruled out by repeating the recording with RTK FIXED.
- **Impact:** a heading error of 5° puts an obstacle 10 m away about 0.9 m off in the map.
- The zero covariance values are the known issue from the audit: the position filters treat the IMU heading as perfect, so they copy this wander one-for-one.

### 5. Wheel odometry and position filters
| Measurement | Result | Target | Pass |
|---|---|---|---|
| Wheel speed while parked | 0 RPM | 0 | PASS |
| Wheel odometry movement while parked | 0.0 cm | 0 | PASS |
| Local position drift | 0.0 cm | < 1 cm | PASS |
| Local heading change | −2.7° (copies the IMU) | < 0.1° | FAIL |
| Map position spread while FIXED | not measured (no FIXED) | ≤ 2 cm | — |
| Map position jump on RTK re-fix | not measured (no FIXED) | < 10 cm | — |

### 6. Message timing
| Topic | Rate / longest gap | Target | Pass |
|---|---|---|---|
| IMU | 100.0 Hz / 40 ms | ≥ 95 Hz | PASS |
| Wheel odometry | 20.0 Hz / 61 ms | ≥ 19 Hz | PASS |
| Local position filter | 24.6 Hz / 71 ms | 30 Hz configured | FAIL (minor) |
| Map position filter | 24.2 Hz / 70 ms | 30 Hz configured | FAIL (minor) |
| GPS | 4.0 Hz / 281 ms | ≥ 3.5 Hz | PASS |
| Corrections | 5.3 Hz / 1.9 s | steady | PASS |

| Delay (measurement to arrival) | Median / 95th percentile | Target | Pass |
|---|---|---|---|
| IMU | 38 / 58 ms | < 50 ms | PASS (median) |
| Wheel odometry | 1 / 1 ms | < 50 ms | PASS |
| GPS | 79 / 99 ms | < 50 ms | **FAIL** |

**What it means:**
- The filters ran at 24–25 Hz instead of 30 Hz during this recording; the short test recording earlier showed 30 Hz. The cause is not known yet; to be checked while not recording.
- GPS positions arrive about 80 ms late. At 0.7 m/s that puts the GPS position 6 cm behind the robot. This matches the audit.

## Runs 2–4: follow-up

### RTK: why it doesn't fix
| Check | Result |
|---|---|
| NTRIP login | Accepted; the connection held the whole session |
| Corrections are live | Message times match the receiver's own GPS clock |
| Corrections are complete | Same message set as Friday's FIXED session; base position every 30 s; one base station (ID 1426, 3.9 km) |
| Receiver uses them | Yes: FLOAT 45–93 % of the time. FLOAT is only possible with corrections. |
| Satellites used | 10–15 at three spots, all within about 10 m of Friday's FIXED spot, which had 19–23 |
| Robot moving during checks | No (wheel encoders unchanged) |

**Conclusion:** the NTRIP account and corrections are working. The receiver tracks too few satellites to resolve FIXED. The same area gave 19–23 satellites on Friday, so the likely causes are physical: antenna placement (on Friday it fixed only after the antenna was moved), the antenna cable or connector, or something blocking the antenna. The Xsens publishes no per-satellite signal strength, so software cannot tell these apart.

License status page: https://www.earthscope.org/user/licenses (sign in with the EarthScope account).

### IMU heading after driving
Run 4, still stretches only (the robot was turned at minutes 5 and 13):

| Minutes | Heading change per minute |
|---|---|
| 0–4 | +0.19, +0.10, −0.07, −0.22, +0.12 ° |
| 6–12 | −0.26, +0.04, −0.14, +0.02, +0.09, −0.07, −0.01 ° |
| 15–23 | +0.04, +0.04, +0.01, +0.23, −0.07, −0.06, −0.07, 0.00, +0.15 ° |

**Conclusion:** once the robot has moved, the heading holds within ±0.26° per minute while parked; most minutes are under 0.1°. The large swings in run 1 were start-up settling, before any GPS movement had corrected the heading. The planned warm-up drive handles this. **No need to change the Xsens filter profile.**

### Run 5: corrections silently stopped
Run 5 started after a Jetson reboot. The robot sat on plain GPS for most of it.

| Check | Result |
|---|---|
| `/rtcm` (corrections) | **No data** |
| NTRIP client | Still running, still sending the robot position to the caster |
| Connection to EarthScope | Open but dead: 5.7 KB received in total (about 3 s of corrections), 9.5 KB stuck unsent, the system retrying every 2 min |
| Internet | Working (router 0.7 ms, internet 64 ms) |
| Jetson's internet uplink | Cellular (public address in a mobile carrier's range) |

**Cause:** the network path to the caster broke shortly after connecting, most likely when the cellular link changed address (the Jetson logged a network change at 17:28). The NTRIP client has no timeout for "connected but receiving nothing", so it never reconnected.

**Fix applied:** restarted the NTRIP client at 17:34. Corrections resumed at about 6 messages per second.

**Finding for the code:** the NTRIP client must reconnect when no corrections arrive for a few seconds. Until then, anyone running the robot has to watch `/rtcm`. The watcher used during testing now alerts when corrections stop.

### Satellite count capped at about 14: investigation
**Observation:** 11–15 satellites used all day, even with the antenna held up to open sky. Friday at the same place and time of day: 19–23. Satellite positions repeat almost exactly day to day at the same clock time, so the sky should look the same as Friday.

| Possible cause | Check | Result |
|---|---|---|
| Satellite count limited in the software | Driver source `ntrip_util.cpp:185`: the count is the receiver's own "satellites used" number, passed through unchanged | Not the cause |
| Xsens settings changed | `xsens.yaml` last changed 2026-05-30; device writes off (`enable_deviceConfig: false`); log confirms "no need to configure MTi" | Not the cause |
| Satellite systems switched off | Only BeiDou is off, and it was off on Friday too | Not the cause |
| Corrections server or login | Live, complete, same base station as Friday. Corrections don't change the satellite count anyway. | Not the cause |
| Full robot software stack | Minimal setup (Xsens + NTRIP only, Friday's exact setup): same 11–14 | Not the cause |
| Receiver fault or USB problem | Status flags normal; no USB errors; receiver restarted twice (reboots) | No evidence |
| Sky blocked | Antenna held up to open sky: still 14 | Unlikely |
| **Radio interference from the robot** | Not testable from software | **Most likely** |
| **Antenna, cable or connector damage** | Not testable from software | **Likely** |

**Why interference or cable:** both reduce the signal strength of every satellite at once. The receiver then drops the weakest ones and caps out at a fixed number, whatever the sky looks like, which matches what we see. Sources on the robot include the USB 3.0 hub (the Xsens and Teensy connect through it; USB 3.0 is a known GPS jammer), the Jetson, the LiDAR, motor controllers, LED ring, and power converters. The Xsens outputs no per-satellite signal strength, so software cannot confirm this.

**Tests run (17:40–17:58):**

| Test | Satellites used | Conclusion |
|---|---|---|
| Satellites overhead now, from published orbits (GPS + GLONASS + Galileo, above 10° / 15°) | 27 / 23 overhead; receiver uses 10–14 | Receiver gets about half of what is overhead |
| Same calculation for Friday 17:29 (when it fixed) | 29 / 22 overhead; receiver used 19–23 | Sky is the same today as Friday |
| Minimal software (Xsens + NTRIP only, like Friday) | 11–14 | Our software is not the cause |
| Web UI, actuator and motor-controller traffic off for 60 s | 10 on, 10 off | Robot drive electronics are not the cause |
| Cellular router uploading at 1 MB/s for 45 s vs quiet | 11–14 during, 11–14 quiet | Cellular transmitting is not the cause |

The robot's internet is a Sierra Wireless AirLink cellular router on AT&T (hardware address prefix 00:14:3E). Its transmitting does not change the count. Whether the unit disturbs the GPS just by being powered is still untested.

**Base station and GPS-only comparison (18:05–18:28):**

| Check | Result |
|---|---|
| Satellites the base station (3.9 km away) tracks now, from its correction messages | **29**: GPS 12, GLONASS 8, Galileo 9 |
| Satellites the robot uses | 10–14 |
| Space weather (planetary Kp index) | 0.3, very quiet |
| Robot count vs GPS satellites overhead, eight 5-minute windows (17:50–18:28) | Robot 11–13; GPS overhead 10–13; all three systems 26–28. **The robot matches GPS-only in every window.** |
| Robot HDOP vs computed HDOP | Robot 1.0–1.2; GPS-only set 0.8; all three systems 0.5 |
| Reseat antenna cable (18:01) | Dropped to 4 while unplugged, back to 10–12 within 10 s. No change. |

**Conclusion:** the robot's receiver is using about as many satellites as GPS alone provides, while the base station 3.9 km away uses GPS, GLONASS and Galileo. The sky, space weather, corrections, our software and the drive electronics are all ruled out. What remains is inside the Xsens GPS receiver or its antenna. Either GLONASS and Galileo are not being received (receiver setting, or an antenna or filter that passes only the GPS frequency), or their signals are too weak to use. The Xsens does not report per-system satellite data to ROS. Confirming requires Xsens MT Manager on a laptop (GNSS view: satellites per system with signal strength, and receiver settings).

**Can a setting cause it? No.**
- The Xsens API (`/usr/local/xsens/doc/xsensdeviceapi`, device option flags) has only two satellite-system switches for this device: `XDOF_EnableBeidou` ("enables Beidou, disables GLONASS") and `XDOF_DisableGps`. There is no switch that disables Galileo.
- The driver prints `EnableBeidou` at startup when it is set (`xdainterface.cpp:380-382`). It was not printed in any startup today, so BeiDou is off and GLONASS is on.
- The receiver is therefore set up to use GPS, GLONASS and Galileo, and still uses only about as many satellites as GPS alone.

**Root cause, by elimination:** the signal path between the sky and the receiver: the antenna, its cable or connectors, the receiver's own radio input, or radio interference at the antenna. Nothing in software or settings can produce this, and nothing in software was changed. MT Manager's GNSS view (per-satellite signal strength) or swapping in a spare antenna and cable will show which part.

**Resolved (19:26–19:45): the cap came from where the antenna sits on the robot.**

Per-satellite readings straight from the Xsens (Xsens Device API on the laptop; the output list was changed temporarily and restored, read-back matched every time):

| Setup | Satellites used | Strongest signals | Result |
|---|---|---|---|
| Xsens on laptop, antenna in its normal position | 9–12 | 35 dB-Hz | weak; most GLONASS unusable |
| Xsens on laptop, antenna **held up in the open** | **20–21** (GPS 8, GLONASS 5, Galileo 4, BeiDou 4) | 44–48 dB-Hz | healthy |
| Xsens back on **Jetson**, antenna held up | 9 → **18** within 4 min | — | **RTK FIXED** from 1 min after restart |

**Conclusions:**
- Receiver, antenna, cable, all four satellite systems, and the Jetson/USB/robot power are fine.
- The cap of about 14 came from the antenna's normal mounting position: signals there are about 10–15 dB weaker, which drops the weaker satellites (mostly GLONASS). A clear view of the sky restores 18–21 satellites and RTK FIXED.
- Friday's fix after "moving the antenna" is the same effect.
- Settings on the device: filter General_RTK, option flags 0x1A20 (no BeiDou swap, GPS on), u-blox platform 4, lever arm 0.74 m.

**Fix:** mount the antenna as the highest point on the robot with a clear view of the sky in every direction, on a metal plate if the antenna needs one. Then re-run the Phase 0 static recording from the robot.

**Tooling note:** the satellite logger repeated its last reading when the driver stopped (19:12–19:41 entries are stale repeats). It must only write fresh data.

**Test (hardware, with live count every 20 s):**
1. Take the antenna as far from the robot as its cable allows, pointed up. If the count jumps to about 20, the robot is interfering.
2. If it does, bring it back and switch off or unplug one device at a time (LiDAR, USB hub, LED ring, motor power) until the count jumps.
3. If it doesn't jump away from the robot, reseat or swap the antenna cable, then the antenna.
4. Reference: a phone GPS app (for example GPSTest) at the same spot shows how many satellites are actually overhead.

### Other observations
- Position filters slowed from 30 Hz (short test at start) to 24.6 Hz (run 1) and 21 Hz (run 4). Jetson load was only 1.4 of 8 cores. Cause not found yet.
- GPS delay rose from 79 ms (run 1) to 112 ms (run 4).
- `actuator_node` uses 35 % of one CPU core, mostly from logging 49 lines per second.
- A NoMachine remote-desktop session was open during run 4 (about 30 % CPU).
- The Jetson was rebooted after run 4, which also restarted the GPS receiver. Run 5 tests whether a fresh receiver start changes the satellite count.

## Run 6: full localization stack, RTK FIXED (19:52–20:03)
Xsens back on the Jetson, antenna held up in the open (not on its robot mount), 11-minute static recording, robot still.

| Area | Measurement | Result | Target | Pass |
|---|---|---|---|---|
| RTK | Time to FIXED / time FIXED | immediate / 100 % | < 3 min / ≥ 95 % | PASS |
| RTK | Position spread while FIXED | 0.9 / 0.7 cm (largest 3.5 cm) | ≤ 1.5 cm | PASS |
| RTK | Reported vs measured accuracy | 1.4 vs 1.2 cm | within 2× | PASS |
| RTK | Satellites / HDOP | 22–24 / 0.7 | ≥ 15 / ≤ 1.2 | PASS |
| RTK | Longest correction gap | 1.8 s | ≤ 2 s | PASS |
| Antenna | Antenna ↔ IMU distance / direction (Xsens outputs) | 0.741 m / −0.1° | 0.74 ± 0.03 m / ±5° | PASS |
| Filters | Map position after the first minute (largest from average) | 3.7 cm | ≤ 2 cm | close (antenna not on robot mount) |
| Filters | Map position start-up jump (0,0 → real position) | 2.4 m in < 5 s | — | expected |
| IMU | Heading change while parked (fresh start, no warm-up drive) | +15.7° in 11 min | < 0.2°/min | FAIL: start-up settling, as in run 1 |
| Timing | Position filter rate | 20 Hz | 30 Hz configured | FAIL (open question) |
| Timing | Apparent delay IMU / GPS | 63 / 138 ms | < 50 ms | FAIL: see clock finding |

**New finding: the Jetson clock is 122 ms off.** The time-sync service reports an offset of +121.6 ms. The Xsens stamps every message with GPS time, so on the Jetson every IMU and GPS message looks 60–140 ms old, and the apparent delay grew during the day (79 → 112 → 138 ms) as the clock drifted. The position filters use these timestamps, so this affects autonomy. Fix: proper time sync on the Jetson (for example chrony, ideally disciplined by GPS time).

**Antenna offset check** passed with the Xsens's own outputs. It must be repeated with the antenna on its final robot mount, together with the tape measurement.

**Analysis script:** now skips the first 60 s when measuring map-position spread (start-up jump).

## Xsens filter profile comparison (20:08–20:32)
Same spot, robot not moved (roll and pitch identical in both runs), fresh start, no warm-up drive, RTK FIXED, 10 min parked each. GeneralMag_RTK was written to the device once (other settings re-applied with the same values), without magnetic calibration. **The device is currently on GeneralMag_RTK**; option 0 switches back.

| Measurement | General_RTK | GeneralMag_RTK |
|---|---|---|
| Heading change per minute | −1.1° to +5.3° | −0.18° to +0.03° |
| Total heading change, 10 min | +15.7° | −0.17° |
| Drift, second half | 0.47 °/min | 0.008 °/min |
| Reported heading (compass bearing) | about 80° | about 308° |
| Phone compass, robot front | 270° | 270° |
| Error vs phone | about 170° | about 38° |
| RTK spread / map position after 1 min | 1.2 cm / 3.7 cm | 1.2 cm / 4.4 cm |

**Conclusions:**
- GeneralMag_RTK holds heading far more steadily while parked from a fresh start.
- General_RTK had not found its heading at all without a warm-up drive: about 170° off.
- GeneralMag_RTK is much closer but still about 38° off the phone compass. Likely causes: the magnetometer is uncalibrated (field distorted by the robot), the phone reading was disturbed by the robot's steel, and the phone may show magnetic rather than true north (local difference about 11.5°).
- A phone compass is only accurate to ±5–10° near metal. The reliable reference is a short straight drive with RTK: the GPS track direction is exact to a fraction of a degree.

**Straight drive, GeneralMag_RTK (20:38), 0.4 m/s, 3.2 m by RTK (RTK in FLOAT, about ±10 cm):**

| Measurement | Result |
|---|---|
| True direction of travel (RTK) | 258.7° compass |
| Xsens heading (GeneralMag_RTK, uncalibrated) | 306.7° compass |
| Heading error | **−48°** |
| Heading change while driving | −0.08° |
| Left vs right track distance | 0.42 % (target ≤ 1 %: pass) |

**Decision:** uncalibrated GeneralMag_RTK is unusable (48° heading error). **Switched back to General_RTK at 20:43** (confirmed after restart; other settings re-applied unchanged). GeneralMag_RTK can be retried after Magnetic Field Mapping with the Xsens on its final robot mount (MFM SDK is installed on the laptop).

**Next (original):** a 5–10 m straight drive to measure the true heading error of each profile. If GeneralMag_RTK stays more than about 5° off, run Magnetic Field Mapping (Xsens `magfieldmapper`) on the robot, then measure again.

## Straight drives, corrected analysis (20:38 and 20:48)
Each drive: 0.4 m/s forward for 10 s, heading-hold on, RTK in FLOAT (about ±10 cm). **Truth is the raw GPS antenna track (`/gnss`).** The first analysis used the Xsens fused position, which depends on the Xsens heading and was wrong when the heading was wrong; the script (`straight_analyze.py`) is fixed.

| Measurement | Drive 1: GeneralMag_RTK | Drive 2: General_RTK |
|---|---|---|
| True direction of travel (GPS antenna) | 264.2° | 263.5° |
| Xsens heading | 306.7° | 88.0° |
| Heading error | −42.5° | +175.5° |
| Sideways deviation from a straight line | 2.7 cm | 4.3 cm |
| Left vs right track distance | 0.44 % | 0.07 % |
| Encoder distance vs GPS distance | +0.8 % | +7.3 % (FLOAT noise too large to trust) |

**Findings:**
- The true direction is consistent between drives (about 264°) and close to the phone compass (270°).
- Uncalibrated GeneralMag_RTK is about 43° off. Confirms the decision to switch back.
- General_RTK was 175° off because the drive before it was **backwards**. It learns heading from GPS movement assuming forward motion, so **reversing flips the heading**. Rule: the warm-up drive must be forwards.
- The robot drives straight (< 5 cm over about 3 m) and both tracks match within 0.5 % (target ≤ 1 %).
- The distance check needs RTK FIXED and a longer drive.

## Heading-hold follows Xsens heading corrections (20:50–20:53)
- **IMU mounting is correct:** during every forward speed-up, the IMU forward axis measured +0.21 to +0.32 m/s² (expected +0.3). The 175° heading error was not a backwards mount.
- **Driver restart does not reset the Xsens heading:** 90.0° before, 89.6° after. The filter runs inside the Xsens; only a power cycle resets it.
- **Forward drive at 0.4 m/s with a 175°-wrong heading (after the reverse drive):**

| Measurement | Result |
|---|---|
| Xsens heading change during the drive | +9.65° (filter starting to correct) |
| True direction of travel (GPS antenna) | 226° (vs 262–264° on straight drives) |
| Left vs right track distance | 2.69 m vs 3.02 m (12 % difference) |
| Observed | The robot visibly made a slight turn |

**Finding:** heading-hold steers the robot to follow whatever heading the Xsens reports. When the Xsens corrects its own heading while driving, heading-hold turns the real robot. Matches review finding K4/D2. For autonomy:
- start every run with a forward warm-up;
- never warm up in reverse;
- consider disabling heading-hold for navigation (proposal in the scope 3 and 4 reports).

## First odometry accuracy check (20:56), early part of Phase 3
Backwards 0.3 m/s for 10 s, **heading-hold off** (equal track speeds, no IMU steering), then restored to 0.05. Truth is the GPS antenna track. RTK FIXED only 9 % of the run (FLOAT otherwise), so GPS distance is about ±3 % over this length. Scripts: `odom_run.sh`, `odom_analyze.py`.

| Measurement | Result |
|---|---|
| GPS antenna distance | 2.786 m |
| Encoders (average) | 2.685 m (−3.6 %) |
| `/wheel_odom` | 2.713 m (−2.6 %) |
| `/odometry/filtered` | 2.712 m (−2.7 %) |
| Left vs right track | 0.14 % |
| Sideways deviation | 3.0 cm |
| Implied distance per motor revolution | 0.02069 m (configured 0.01994) |

**Findings:**
- With equal track speeds the robot drives straight (3 cm over 2.8 m, tracks within 0.14 %). The earlier turn was heading-hold following the IMU.
- Odometry under-reads distance by about 3 % (tentative).
- **To confirm:** 20 s runs (about 8 m) with RTK FIXED throughout, 3 forward and 3 backward. If it holds, set `m_per_motor_rev` to the measured value.

## Still to do in Phase 0
1. **Get RTK FIXED.** Moving the robot a few metres did not help. Check the antenna: set it up the way it was when it fixed on Friday, make it the highest point on the robot, and reseat the cable at both ends. Aim for ≥ 18 satellites. Then repeat the 10-minute static recording. That completes: time to FIXED, FIXED accuracy, antenna offset, map position spread, and the re-fix jump.
2. **Heading:** resolved (see follow-up below). Still do the warm-up drive before any recorded test.
3. **By hand:** tape-measure from the antenna centre to the IMU centre; mark the ground under both.
4. **IMU warm-up:** drive 5 m out and back before any recorded driving test.

## Setup notes (for repeating this phase)
- The web UI launch left its two nodes running after being stopped; they had to be stopped separately before reading the Teensy.
- The first recording attempt failed (a script error with the ROS setup file). It was fixed and restarted 44 s after launch, before any FIXED, so no data was lost.
- Commands: see `../README.md`, *Procedure*.
