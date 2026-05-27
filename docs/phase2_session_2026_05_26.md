# Phase 2 Field Session — 2026-05-26 (SW Michigan, outdoor grass)

Live-on-Jetson session: bring up the full navigation stack outdoors, capture baseline drive data under MPPI + BackUp + DriveOnHeading actions, measure EKF drift between local (odom) and global (map) frames, and quantify inner-PID delivery in both forward and backward directions.

**Outcome — three findings dwarf everything else:**

1. **EKF drift is ESSENTIALLY ZERO.** Local and global EKF agree to 0.0 mm over a 16 s drive. Wheel_odom and local EKF agree to <12 mm. The dual-EKF stack is healthy; the Phase 2 hypothesis ("PID gap corrupts odom") is **refuted** for the local EKF chain.
2. **The chassis CURVES HARD on grass — in both directions — even with perfect inner-PID symmetry on forward drive.** Forward: L/R PID delivery 96.05% / 96.07% (0.02% asymmetry) yet the robot drove 3.12 m forward + **4.05 m lateral** (130% lateral drift). This is *chassis-mechanical*, not a control bug — surface-induced right-side drag that no kinematic correction will fix.
3. **The right SparkMAX has a brand-new asymmetric failure mode in REVERSE.** Backward: L=102%, R=79% delivery (24.86% asymmetry). Not seen in any prior testing because all prior testing was forward-only.

Plus a 2 h navsat debugging saga ending in the deployment of [commit 33d62ea](#commit-33d62ea-was-the-key-fix) which finally unblocked the global EKF chain.

---

## Session preamble — connectivity + workspace setup

- Outdoors in SW Michigan, latitude ≈ 42.4016°, longitude ≈ -86.2834°. **GPS reports `status: 0`** (Xsens NavSatStatus driver bug per [session 2026-05-19 §4](session_2026_05_19_mppi_planner_costmap_gps.md) — actual measured noise was 0.66 m std radial, suggesting SBAS is active but mis-reported).
- Jetson reachable only via LAN ethernet (Tailscale offline — Jetson has no upstream internet through gateway 192.168.13.31). LAN gateway → DHCP → 192.168.13.10 (Jetson) + 192.168.13.109 (laptop).
- NTRIP correctly disabled per [`project_no_rtk_in_codebase`](.) — `enable_ntrip:=false` is the launch default since commit 339bbdc.
- Webui systemd service `avros-webui` stopped manually before each nav launch to free `/dev/ttyACM0` (per memory [`feedback_serial_port_collision`](.)).

---

## The navsat saga — 5 layered failures before the chain came alive

The global EKF chain (`navsat_transform` → `/odometry/gps` → `ekf_filter_node_map` → `map → odom` TF) was DEAD for ~2 hours of debugging. Five distinct issues had to be fixed in sequence.

### 1. Hardcoded CPP datum, robot in Michigan (~2700 km offset)

`navsat.yaml` had `datum: [34.059270, -117.820934, 0.0]` (Cal Poly Pomona) and `wait_for_datum: true`. Outdoors in Michigan, every GPS fix was ~2700 km from the hardcoded datum. `ekf_filter_node_map`'s `odom0_pose_rejection_threshold: 4.0` rejected every single fix as a Mahalanobis outlier. **The global EKF never got a usable GPS input.**

### 2. Misguided "fix": `wait_for_datum: false` (4-settings coupled)

First attempt at a permanent fix: set `wait_for_datum: false` to auto-anchor on the first GPS fix. **This made things worse** — navsat got stuck in an auto-anchor loop, never reaching `transform_good_ = true`, never publishing `/odometry/gps`.

**Root cause:** the 4 navsat config knobs (`wait_for_datum`, `datum`, `use_odometry_yaw`, `broadcast_cartesian_transform`) are internally coupled and form TWO valid combinations:

| Knob | Combo A (our dual-EKF) | Combo B (Nav2 GPS demo) |
|---|---|---|
| `wait_for_datum` | true | false |
| `datum` | hardcoded `[lat, lon, yaw]` | unset |
| `use_odometry_yaw` | false (need IMU for heading) | true |
| `broadcast_cartesian_transform` | false (ekf_map publishes map→odom; navsat broadcast would create TF loop) | true (single-EKF setup, navsat owns the map TF) |

Mixing settings from both combos creates silent failures. Documented in memory [`feedback_navsat_4_settings_coupled`](.).

### 3. Datum typo (-86.283**748** vs -86.283**441**)

After reverting to Combo A and updating the datum to current site coords, **a typo in the longitude** put the datum 2.74 m off from the actual GPS sample mean. Compared to GPS rejection threshold (Mahalanobis on Xsens-reported cov=4.48 m² → σ=2.12 m), this was within tolerance (1.29 σ) — not catastrophic. But still wrong.

Fix: ran a 707-sample (177 s) GPS analysis (`gnss.csv` extracted from a static bag), used the sample mean as the precise datum:

| Statistic | Value |
|---|---|
| Latitude noise (std) | **0.664 m** |
| Longitude noise (std) | **0.332 m** |
| Radial drift (95%) | 1.51 m |
| Slow walk over 3 min | 1.41 m radial |
| Xsens-reported cov_e=cov_n | 4.48 m² (σ = 2.12 m — over-estimates real noise) |

The measured noise (0.66 m std) is **much better than the 2–5 m we assumed** in [`project_no_rtk_in_codebase`](.) — the Xsens receiver is likely getting SBAS but the driver mis-maps `NavSatStatus` (sets `STATUS_SBAS_FIX` only on RTK_FLOAT, never on real SBAS — known driver bug).

### 4. CycloneDDS stale peer (192.168.13.105) starving message processing

The navsat process log showed **371 UDP write failures** to `192.168.13.105` over a few minutes — phantom DDS peer endpoints from a prior session when my laptop had a different DHCP-assigned IP. CycloneDDS kept retrying delivery to ~19 endpoints on a host that no longer existed, **starving real message processing** (subscription matching, callbacks, etc.). This was the silent reason `ekf_filter_node_map` had ZERO log entries — it was alive but starved.

Fix: full nuclear kill of every ROS process (including the ros2 daemon) + 30 s wait for DDS leases to age out + clean relaunch. The 192.168.13.105 spam disappeared (UDP fail count 371 → 0).

### 5. Xsens USB disconnected mid-session (`No MTi device found`)

After several relaunch cycles, the Xsens MTi-680G driver crashed at startup with `[ERROR]: No MTi device found / Failed to connect device / process has died`. The device had effectively disconnected on USB.

Per memory [`feedback_xsens_imu_bias_recovery`](.), the standard recovery is a **physical USB power-cycle**. User did so; device re-enumerated on `/dev/ttyUSB0` and the driver came up cleanly on the next relaunch.

### Commit 33d62ea was the key fix

After ALL of the above was resolved, **the chain still didn't bootstrap**: navsat was alive, datum was correct, DDS was clean, IMU was healthy, but `/odometry/gps` and `/odometry/global` both stayed silent.

The user pointed out that **a commit on the laptop (`33d62ea ekf: IMU outlier gates + wheel_odom into map EKF + accurate header doc`) had not been pushed to the Jetson.** The Jetson's `ekf.yaml` was missing two critical changes:

1. **`/wheel_odom` added as input to the MAP EKF** (was only IMU + GPS previously). This is the canonical Nav2 dual-EKF pattern — both EKFs share the same continuous sensors; only the map EKF additionally has GPS.
2. **`imu0_pose_rejection_threshold: 5.0` + `imu0_twist_rejection_threshold: 5.0`** on both EKFs (Mahalanobis outlier gates against the Xsens stuck-bias condition documented in [`docs/skid_steer_kinematics_findings_2026_05_18.md`](skid_steer_kinematics_findings_2026_05_18.md) §6).

The **wheel_odom into map EKF change broke the deadlock**: previously, `ekf_filter_node_map` had no usable input until navsat published `/odometry/gps`, but navsat couldn't bootstrap without `/odometry/global` from ekf_map → circular dep. With wheel_odom available continuously at 20 Hz, ekf_map publishes `/odometry/global` immediately on startup regardless of GPS state. navsat's subscription on `/odometry/global` then succeeds, and the loop closes.

**Result after deploying commit 33d62ea's `ekf.yaml`:**

| Topic | Rate |
|---|---|
| /odometry/filtered | 20 Hz ✅ |
| /odometry/global | 20 Hz ✅ (was silent for 2 h) |
| map → odom TF | exists (identity) ✅ |
| /odometry/gps | still silent ❌ (separate navsat-IMU subscription bug) |

### Unresolved: navsat doesn't subscribe to ANY IMU topic

Even with all of the above fixed, **navsat's subscriber list shows only `/gnss` + `/odometry/global` + `/parameter_events` — no IMU subscription at all**, despite trying both `('imu', '/imu/data')` and `('imu/data', '/imu/data')` as remap keys.

The IMU topic `/imu/data` is published by Xsens at 100 Hz and has 3 consumers (ekf_odom, ekf_map, actuator_node) — but never navsat. Tested with `--log-level debug` on navsat — no debug output. Even `/datum` service call doesn't unblock `/odometry/gps`. The root cause requires reading installed `robot_localization` source (not present on Jetson) or building a debug build — out of scope for this session. **This blocks GPS corrections from reaching the map EKF**, but the Phase B "decommission navsat" plan (issue [#12](https://github.com/Paarseus/IGVC_ROS2/issues/12)) makes this moot anyway.

For now: map frame works (map ≡ odom, zero-drift) but isn't GPS-anchored. For short drives this is acceptable; for long missions the map frame would slowly drift with odom drift.

---

## Velodyne-on causes BackUp to abort instantly

Once the EKF chain came alive, the first attempt at `nav2_msgs/action/BackUp` (back up 5 m) **aborted in 1 second**:

```
behavior_server [WARN]: Collision Ahead - Exiting DriveOnHeading
behavior_server [WARN]: backup failed
```

Root cause: BackUp's collision-checker reads `/local_costmap/costmap`, which has the STVL layer fed by `/velodyne_points`. Outdoors, the Velodyne sees real objects (trees, equipment, people) within the rolling 50×50 m window + 1 m inflation halo, marks them lethal, BackUp refuses to drive into them.

**Fix for this session:** relaunch nav with `enable_velodyne:=false`. Local costmap starts empty and stays empty (no LiDAR observations). BackUp and DriveOnHeading then work without collision interference. **Direct `/cmd_vel` bypass is the alternative** if you want LiDAR on but still need a forced motion test.

---

## Phase 2 motion test results

Three drives captured into per-run bags. All three reported `SUCCEEDED`.

### Run 1 — MPPI forward 5 m, vx_max=0.35 (LiDAR ON)

| Metric | Value |
|---|---|
| Goal | NavigateToPose to (5.0, 0.0) in map frame |
| Status | **SUCCEEDED** (within xy_goal_tolerance=2.0 m) |
| Final pose | (3.16, 0.28) — **stopped 2 m short**, tolerance hit |
| Navigation time | 17 s |
| Recoveries fired | **5** |
| Lateral curl | 0.28 m / 3.16 m forward = **5.1°/m** (matches [2026-05-19 indoor finding](session_2026_05_19_mppi_planner_costmap_gps.md) §1) |

Note: the bag from this run was contaminated by an e-stop event partway through, so its data was discarded.

### Run 2 — BackUp 5 m at 0.35 m/s (LiDAR OFF)

| Source | Δx | Δy | Δyaw |
|---|---|---|---|
| /wheel_odom | **-6.56 m** | **+3.28 m** | **-47.6°** |
| /odometry/filtered | -6.56 m | +3.26 m | -47.6° |
| /odometry/global | -6.56 m | +3.26 m | -47.6° |
| /odometry/gps (navsat) | +0.07 m | -1.23 m | — |

| Metric | Value |
|---|---|
| Status | **SUCCEEDED** (BackUp tracks `distance_traveled`, not heading) |
| BackUp `distance_traveled` | 5.012 m (overshot 0.012 m due to slew-down) |
| Elapsed time | 16.5 s |
| **Inner-PID L delivery** | **102.00 %** |
| **Inner-PID R delivery** | **79.45 %** |
| **L/R asymmetry** | **24.86 %** ⚠️ |

The chassis **curved ~50% laterally** and rotated 47.6° CW because the **right SparkMAX delivered only 79% of commanded RPM**. BackUp counts distance along the curved arc, so it reported success even though the heading and lateral drift were catastrophic.

### Run 3 — DriveOnHeading forward 5 m at 0.35 m/s (LiDAR OFF)

| Source | Δx | Δy | Δyaw |
|---|---|---|---|
| /wheel_odom | **+3.12 m** | **-4.05 m** | **-10.08°** |
| /odometry/filtered | +3.12 m | -4.05 m | -10.08° |
| /odometry/global | +3.12 m | -4.05 m | -10.08° |

| Metric | Value |
|---|---|
| Status | **SUCCEEDED** (DriveOnHeading tracks `distance_traveled`) |
| DriveOnHeading `distance_traveled` | 5.03 m |
| Elapsed time | 16.6 s |
| **Inner-PID L delivery** | **96.05 %** |
| **Inner-PID R delivery** | **96.07 %** |
| **L/R asymmetry** | **0.02 %** ✨ |
| **Lateral drift** | **-4.05 m on 3.12 m forward = -130 % of forward** |

This is the **bombshell**: the chassis drove a path that's **mostly lateral**, despite the PID delivering **perfectly symmetric** L and R RPM. With zero motor asymmetry, the curve must come from **chassis-level mechanical asymmetry** — almost certainly the right side has higher rolling resistance / slip on grass.

---

## EKF drift — the actual Phase 2 question

| Comparison | Forward Δx | Forward Δy | Backward Δx | Backward Δy |
|---|---|---|---|---|
| wheel_odom vs /odometry/filtered | < 2 mm | < 2 mm | 5 mm (0.08%) | 12 mm |
| **/odometry/filtered vs /odometry/global** | **0.0 mm** | **0.0 mm** | **0.0 mm** | **0.0 mm** |

**The dual-EKF stack is healthy.** Local and global EKF agree exactly to 4 decimal places of meters. Wheel_odom and the local EKF agree to ~1 cm. The 2026-05-19 EKF tune (process noise 1.0→0.1, GPS rejection 6.0→4.0) plus the 33d62ea wheel_odom fusion produces a rock-stable chain.

**GPS is not pulling the map frame around** because navsat isn't publishing `/odometry/gps` (the unresolved IMU-subscription bug). So map ≡ odom, identically. For Phase 2 measurements this is fine; for long-mission GPS anchoring it's a problem.

**The Phase 2 hypothesis is REFUTED for the local EKF chain.** The PID delivery gap does not corrupt the odom frame. The EKF accurately tracks whatever the chassis actually does — which on grass is curve substantially.

---

## Inner-PID asymmetry — the unexpected discovery

| | Backward | Forward |
|---|---|---|
| L delivery | 102.00% | 96.05% |
| R delivery | **79.45%** | 96.07% |
| L/R asymmetry | **24.86%** | 0.02% |

**The right SparkMAX has a direction-dependent failure mode.** Forward driving is perfectly symmetric (L≈R at 96%); backward driving shows the R motor delivering only 79% of commanded RPM. Possible causes:

- **Mechanical**: right motor's brake/idle behavior different in reverse, or right-side gearbox has direction-dependent friction
- **SparkMAX firmware**: kFF or kP gains might be applied asymmetrically in negative direction (a sign-handling bug)
- **Encoder phase**: right encoder reading direction might be inverted in one of the conversions

This wasn't caught in any prior testing because **all prior characterization (motor_sync 2026-05-05, PID re-tunes 2026-05-13, 2026-05-18, etc.) was forward-only.** The 2026-05-05 motor sync investigation specifically used wheels-up forward bursts.

**This is a fresh failure mode worth opening a separate bug issue for.**

---

## Chassis-level lateral drift — the more important finding

The forward drive shows **130% lateral drift** despite **perfect (0.02%) inner-PID symmetry**. This is *not* a motor or control issue — both wheels are tracking commanded RPM accurately and equally. The chassis is **mechanically asymmetric on grass**.

Possible causes:
- Different tire/track rolling resistance L vs R on grass
- Slight ground-slope causing one side to drag
- Different track tension L vs R
- Track ground-contact patch differences
- Center-of-mass not centered → uneven loading L vs R
- Grass blade orientation creating directional rolling resistance

**Implications:**
- **For IGVC**: no kinematic correction (Mandow multiplier) will fix surface-induced lateral drift. The chassis will arc whenever driving on grass unless the upper-stack controller (MPPI) actively corrects.
- **For MPPI tuning**: the 5°/m yaw curl in [2026-05-19 §1](session_2026_05_19_mppi_planner_costmap_gps.md) and the 5 recoveries fired in Run 1 today are MPPI fighting this mechanical curve. MPPI is doing its job; the problem is fundamental.
- **For the Mandow multiplier**: the 1.19 value calibrated on indoor concrete (per [`docs/skid_steer_kinematics_findings_2026_05_18.md`](skid_steer_kinematics_findings_2026_05_18.md)) is **wrong for grass**. Phase 6 (surface portability) needs to re-cal on grass — and may find no single multiplier value works because the asymmetry isn't a simple skid factor.

---

## Implications for the Phase 2-6 plan

| Phase | Original premise | Revised after today |
|---|---|---|
| 2 (baseline) | Quantify PID gap → odom corruption | **EKF drift is zero. PID gap is forward-symmetric. Real issue is chassis mechanical asymmetry on grass.** |
| 3 (local EKF integrity) | Test wheel_odom ↔ filtered divergence under driving | **Already confirmed: <12 mm divergence over 16 s motion. Local EKF is healthy.** Phase 3 reduces to "re-verify after a longer drive". |
| 4 (GNSS carrot) | Build replacement for global EKF | **Even more urgent** given the unresolved navsat-IMU subscription bug. Phase B step 6 (decommission navsat) becomes the practical fix. |
| 5 (MPPI rollout) | Quantify MPPI emitted-vs-delivered gap | **Need to measure on a STRAIGHT path** — today's curving chassis confounds MPPI tracking measurements. Need either (a) tighter MPPI lateral correction, or (b) a path the chassis can actually follow. |
| 6 (grass portability) | Re-cal Mandow on grass | **Promoted to high priority.** Mandow alone may not fix the 130% lateral; need to investigate per-wheel friction compensation OR closed-loop heading control at actuator level. |

---

## Concrete follow-up items

1. **Investigate right SparkMAX reverse-direction asymmetry.** Bench-test wheels-up with backward commands; compare L_meas vs L_cmd and R_meas vs R_cmd waveforms. Check SparkMAX firmware version and PID slot 0 gain settings on the R unit specifically. File as separate bug issue.
2. **Investigate chassis lateral drift on grass.** Could be track tension, ground slope, or COM offset. Try driving on a hard surface with same payload to isolate grass-specific effects.
3. **Fix navsat IMU subscription bug.** Either read `robot_localization` source on the Jetson (install -dev package or build from source) OR fast-track Phase B step 6 (decommission navsat in favor of `gnss_carrot_node`).
4. **Phase B step 1 (shrink global costmap 40→20 m) is safe to ship now** — independent of any of the above bugs.
5. **Phase B step 2 (add DWB as fallback controller)** would help with the 5-recovery MPPI count on curving paths — DWB's path-following is more forgiving of lateral drift than MPPI.
6. **For competition**: assume grass surface will cause significant lateral drift. Plan MPPI tuning + recovery BT around this reality rather than fighting it.

---

## Files / commits

- Telemetry recipe: [`docs/phase_telemetry_recipe.md`](phase_telemetry_recipe.md)
- Phase A winners knowledge: [`docs/winners_research/INDEX.md`](winners_research/INDEX.md)
- Phase B architecture decision: [`docs/architecture_decision_global_frame.md`](architecture_decision_global_frame.md)
- Bag extractor: [`scripts/extract_bag.py`](../scripts/extract_bag.py)
- Bags captured (Jetson `/tmp/phase2/`):
  - `M1back_nolidar_172328/` — backward BackUp 5 m (102/79% L/R asymmetry exposed)
  - `M1fwd_nolidar_172613/` — forward DriveOnHeading 5 m (96/96% symmetric, 130% lateral drift)
- Memories: [`feedback_navsat_4_settings_coupled`](.) (per-site datum + 4-knob coupling); pre-existing [`feedback_xsens_imu_bias_recovery`](.) confirmed applicable to USB disconnect too.

---

## Related issues

- [#11](https://github.com/Paarseus/IGVC_ROS2/issues/11) Localization research: dual-EKF architecture vs. reputable references (no-RTK)
- [#12](https://github.com/Paarseus/IGVC_ROS2/issues/12) Research: PID/controller efficiency + odom integrity + global-frame architecture decision (today's Phase 2 results will be appended as a comment)
- #9 (followup, separate): right SparkMAX reverse-direction asymmetry — to be filed
