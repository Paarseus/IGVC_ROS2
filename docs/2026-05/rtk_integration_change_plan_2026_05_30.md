# RTK Integration Change Plan — IGVC_ROS2 (AutoNav)

Date: 2026-05-30
Author: Lead integrator (synthesis of five verified per-subsystem assessments)
Venue: IGVC AutoNav Challenge, Oakland University, Rochester MI
RTK source: MDOT CORS / MSRN free public NTRIP — mountpoint `NS-IMAX-MSM4` (iMAX/VRS, virtual base at rover, ~0 km baseline), host `mdotcors.michigan.gov:10010`, RTCM 3.2 MSM4 full-GNSS (GPS 1074 / GLO 1084 / GAL 1094 / BDS 1124). Receiver: u-blox ZED-F9P inside the Xsens MTi-680G.

> Provenance note: this session's Bash/Read I/O returned empty (the same degraded environment the per-subsystem agents hit). All "VERIFIED high" facts below were independently read from the actual files by at least two subsystem agents (`frame_strategy` and `nav2_costmaps_goals` cross-corroborate every shared line number; `rtk_health` re-read `ekf.yaml` and corrected a hallucinated "uncomment GPS" proposal). Items marked "low confidence" rest on rationale + MEMORY corroboration only.

---

## 1. Executive summary

RTK takes GNSS position from ~2-5 m (SBAS) to ~1-3 cm (FIX), degrading to decimeter (FLOAT) and back to standalone if corrections lag/drop. But **RTK fixes only the position half** of the problem that drove this stack into the odom frame (commit `5e6f279`). That runaway had two causes: (a) 6-12 m position drift — now collapsed by RTK; and (b) a **65-96 deg map rotation** from the single-antenna MTi-680G deriving yaw from GNSS course-over-ground on the no-magnetometer General_RTK profile. **A single-antenna receiver gives no heading even at cm FIX**, so the rotation is unchanged by RTK.

Consequently:

- **Keep Nav2 control in odom** as the static, RTK-drop-safe default. Do not statically revert `5e6f279`.
- Realize RTK's value through three things that are safe today: a **measured GNSS lever arm**, an **RTK health node** (NMEA GGA parsing — because `sensor_msgs/NavSatStatus` has no RTK enum and robot_localization ignores it), and a **covariance floor/scaler** for graceful de-weighting.
- Treat the aggressive EKF wins (differential->absolute fusion, lower process noise, real gate) as a single **FIX-gated, auto-reverting "RTK profile" that is OFF by default** — never static, because IGVC may forbid NTRIP (judge ruling pending) and RTK can drop mid-run.

Two latent **non-RTK** bugs surfaced during verification and must be fixed regardless of RTK:
- `mission_manager` already emits **map-frame goals** (`GOAL_FRAME_ID='map'`) while `bt_navigator.global_frame=odom` — a frame mismatch (inverse of MEMORY *odom goals need map=odom*).
- The `ekf.yaml` gate comment `13.8 = 6sigma^2` is **unit-wrong**: per `filter_base.cpp` the value is in sigma and is squared in code, so 13.8 -> threshold ~190 -> P(reject) ~4e-42 -> the gate is effectively **OFF**, not "generous" (`docs/fresh_audit_2026_05_29.md` N5).

---

## 2. THE BIG DECISION — map frame vs odom frame

**Decision: stay in ODOM. Do not statically move `bt_navigator` / `global_costmap` / `local_costmap` / `route_server` back to map.**

Why, decisively:

1. **RTK fixes position, not heading.** MTi-680G is single-antenna; General_RTK uses no mag and derives yaw from GNSS COG (observable only > ~0.5 m/s, 180-deg-ambiguous at rest). The 65-96 deg map rotation is unchanged by RTK. cm position into a rotated frame still recedes the goal -> MPPI runaway. Position accuracy is necessary but **not sufficient**.
2. **The map frame jumps discretely (REP-105).** Every fix discontinuity — and especially FIX->FLOAT->standalone — steps `map->odom`. `local_costmap` must stay odom unconditionally so body-relative STVL LiDAR never smears lethal cells on a jump (the documented L6 phantom-accumulation failure: 5878->24567 lethal cells from 8 cm/s drift, `nav2_params_humble.yaml` L493-499). `global_costmap` stays odom for the same reason (STVL-only; semantic layer disabled in global plugins, VERIFIED L518-528).
3. **RTK can drop or be banned.** IGVC §I.2 may forbid the correction source (MEMORY `project_igvc_rtk_rule`). odom is the only continuous, drift-free frame regardless of GNSS state.

**Conditions to re-enable map-frame nav (ALL required, runtime-only, auto-reverting):**
- **Heading observability solved** by EITHER a magnetometer-fused Xsens profile (e.g. GeneralMag_RTK, verified true-north post motion-warmup) OR dual-antenna / moving-base GNSS heading hardware. A position-only gate is explicitly insufficient.
- **Sustained RTK FIX** via NMEA GGA quality `== 4` (NOT NavSatStatus) with dwell/hysteresis.
- **Covariance floor in place** so absolute fusion does not collapse the Mahalanobis denominator (lockout).
- **Automatic immediate revert to odom** on GGA != 4, heading divergence (COG vs odom-EKF yaw), or NMEA staleness.

**Reconcile today (non-RTK):** `mission_manager` issues `frame_id='map'` goals into an odom `bt_navigator`. Default fix: send goals in odom by snapshotting `map->odom` per waypoint at send time (RTK-drop-safe). The gated runtime map<->odom flip is the upgrade path, not the default.

---

## 3. Ranked change list (survived verification — keep/modify only)

| # | Subsystem | Change | File | Current -> Proposed | Risk | Prereq |
|---|-----------|--------|------|---------------------|------|--------|
| 1 | rtk_health | Build `rtk_health_node` (GGA quality + staleness -> `/rtk/fix_state` latched + DiagnosticArray) | `avros_navigation/.../rtk_health_node.py` (new), setup.py, package.xml | no RTK fix-state on bus -> live fix-state keyed off GGA | very low | **yes** |
| 2 | xsens/ekf | Measure & enter GNSS lever arm; flash via device-config; fix imu_link URDF offset | `xsens.yaml` L47; `avros.urdf.xacro` | `[0,0,0]` TODO -> measured `[x,y,z]` m | low | **yes** |
| 3 | xsens | Verify General_RTK at boot (readback-first); flash only if wrong; reset flags after | `xsens.yaml` (`enable_filter_config`, `mti_filter_option`) | assumed -> verified | medium | **yes** |
| 4 | rtk_health/ekf | Build `gps_covariance_relay` (floor/scale `/gnss` cov by fix-state; conditional remap fallback) | `gps_covariance_relay.py` (new); `localization.launch.py` | raw optimistic cov -> floored/honest cov | medium | **yes** |
| 5 | launch | Unify `enable_ntrip` default to `false` across all launch files; opt-in only | `sensors/localization/navigation.launch.py` | mismatched defaults -> uniform `false` | low | no |
| 6 | ekf | Fix gate comment (sigma, squared in code); keep 13.8 interim; set ~4.3 only after floor | `ekf.yaml` L194 + comment | `13.8 #6sigma^2` (gate OFF) -> honest label; ~4.3 in FIX profile | low (comment) / med (value) | no |
| 7 | frame/mission | Reconcile mission_manager map-frame goals vs odom bt_navigator (odom snapshot default) | `mission_manager.py` L45; `waypoints.yaml` | map-frame goals into odom nav -> odom-frame goals | medium | no |
| 8 | nav2 | RTK+map+heading-gated goal-tolerance tighten (runtime only; keep 2.0/0.5/3.0 floor) | `nav2_params_humble.yaml` L70/71/29 + health node | static 2.0/0.5/3.0 -> floor + gated tighten | low | no |
| 9 | ekf/frame | differential->absolute fusion + process-noise drop ONLY as FIX-gated, auto-reverting profile | `ekf.yaml` L186, L209-210 (relay-applied) | `differential:true`, Q=0.5 -> `false`+~0.3 in FIX profile | **high** | no |
| 10 | navsat | On-site SetDatum at OU start pad under FIX (slot 3 = heading rad, never altitude) | `navsat.yaml` L39 | practice datum -> surveyed OU start | low | no |
| 11 | navsat/xsens | Verify `/imu/data` true-north (moving + warmed-up); keep declination 0.0 unless proven magnetic | `navsat.yaml` L16/17 | `0.0/0.0` -> unchanged unless proven | low | no |
| 12 | navsat | KEEP `broadcast_cartesian_transform:false`, `use_odometry_yaw:false`, `publish_filtered_gps:true` | `navsat.yaml` L20/40/41 | unchanged (explicit retain) | none | no |
| 13 | nav2 | KEEP `global_costmap.global_frame:odom`, `rolling_window:true` (+ optional comment) | `nav2_params_humble.yaml` L502/504 | unchanged | none | no |
| 14 | rtk_health | Verify MSM4 carries 1005/1006 (ARP) + 1230 (GLONASS bias); GGA uplink >=1 Hz | ntrip_params.yaml (mountpoint) | assumed -> verified | very low | no |
| 15 | security | Move plaintext MDOT creds out of tracked YAML; rotate exposed password | `ntrip_params.yaml` | committed creds -> placeholders + env/overlay + rotated | low | no |
| 16 | xsens | NavSatStatus.service bitmask = 15 + throttled carrSoln log | `gnsspublisher.h` | `service=0` -> 15 + log | very low | no |

### Notes on the high-risk and reversed-rationale items

- **#9 (absolute fusion)** reverses the deliberate SBAS differential choice (MEMORY `no_rtk_in_codebase` "defend differential GPS"). It is directionally correct for true FIX (robot_localization `integrating_gps`; the Nav2 `dual_ekf_navsat` tutorial this file derives from uses `differential:false`) but must be **relay-applied, FIX-gated, auto-reverting, off by default**, and only after #4 + a confirmed honest-covariance contract. Lowering process noise under differential/SBAS re-opens the documented 36 m lockout (Q was raised 0.1->0.5 on 2026-05-28 to keep P growing).
- **#6 (gate)** supersedes the `dual_ekf` agent's "keep 13.8 as a guardrail" verdict: 13.8 sigma does not "catch gross outliers" — it disables the gate (`fresh_audit_2026_05_29.md` N5). The anti-tighten-in-isolation instinct is right; pinning at a gate-disabling value is not. Sequence strictly: **floor -> differential decision -> gate; never atomically.**
- **DROPPED** (rejected in verification, not listed above): the `rtk_health` proposal to "uncomment `/odometry/gps` as odom0 / remove a stray mapEKF token" — the block is **already active** (`ekf.yaml` L180-195); acting on it would corrupt a working block.

---

## 4. RTK FIX -> FLOAT -> STANDALONE graceful-degradation strategy

Layered so the stack is **never worse than its current unaided-SBAS baseline**:

1. **Detection (#1, prereq):** `rtk_health_node` parses GGA field-6 (4=FIX, 5=FLOAT, 2=DGPS, 1=SBAS, 0=none) + staleness watchdog -> `/rtk/fix_state` (latched). Only honest fix-state source: NavSatStatus has no RTK enum (collapses to GBAS_FIX=2) and robot_localization never reads it.
2. **De-weighting (#4, prereq):** `gps_covariance_relay` floors/scales `/gnss` covariance — FIX ~0.04 m^2, FLOAT ~0.25 m^2, DGPS/SINGLE floored >=9 m^2 (true SBAS error vs the optimistic <1 m the receiver reports, `fresh_audit_2026_05_29.md` N5/P1), NONE -> NO_FIX dropped. Because the EKF de-weights purely by covariance, this one relay makes it trust FLOAT less and SBAS much less, and prevents cm-FIX covariance from collapsing the gate denominator. Ship/validate the floor **alone** first (audit ordering).
3. **Fusion-mode gating (#9):** `odom0_differential` stays TRUE by default; flips to absolute only inside a FIX-only profile after the floor is live and covariance is confirmed to track quality; auto-reverts on FIX->FLOAT->standalone so a multi-meter standalone step is absorbed as pseudo-velocity, not snapped.
4. **Control-frame safety (the big decision):** control never leaves odom statically. Map-frame promotion is runtime-only, gated on GGA==4 AND heading-converged AND floor-present, with dwell/hysteresis and immediate auto-revert. `local_costmap` stays odom unconditionally (REP-105). A mid-run drop reverts fusion mode, goal tolerance, and control frame to the SBAS posture — no runaway, no lethal-cell smear.
5. **Mission continuity (#7):** `mission_manager` reads `/rtk/fix_state` and pauses advancement / widens acceptance radius on degradation.
6. **Policy fallback (#5):** if IGVC forbids NTRIP, `enable_ntrip:=false` brings the stack up on unaided SBAS — every SBAS-tuned default (floor, gate, odom control, loose tolerances) remains valid because it *is* the SBAS design.

**Invariant:** every RTK exploitation is OFF by default and degrades to the validated SBAS baseline automatically. Worst case under any dropout/ban = current behavior.

---

## 5. GNSS lever-arm action (rank 2, prerequisite)

1. **Measure** MTi-680G origin -> GNSS antenna **phase-center** in the MTi body frame (X-fwd, Y-right, Z-down, meters) to ~1-2 cm (use the antenna's phase-center offset, not the connector/housing top).
2. **Confirm** the driver's `setGnssLeverArm` axis/sign convention (esp. sign of Z) before entering — body-frame claim matches Xsens docs generally but was not code-verified.
3. **Enter** the measured numeric triple in `xsens.yaml` `GNSS_LeverArm` (currently `[0,0,0]` L47, TODO, VERIFIED). **Never commit placeholder/symbolic values** — a guess is worse than `[0,0,0]`.
4. **Apply** via the device-config flow (CamelCase `GNSS_LeverArm` flashes to firmware): `enable_deviceConfig:true` once, flash, then back to `false` (so USB re-enumeration during stuck-bias recovery never re-flashes).
5. **Verify** in MT Manager / driver readback.
6. **Resolve** the `imu_link` URDF offset simultaneously (lever arm is referenced to IMU body frame; TODO.md notes they are coupled).

**Why prerequisite:** at 2-5 m SBAS the arm is buried; at cm RTK it dominates — static bias = arm rotated to world frame, plus dynamic phantom velocity v = omega x r (~0.25 m/s at omega=0.5, r=0.5 m) on turns (the GNSS analogue of the documented ZED VIO lever-arm bug, CLAUDE.md). It also inflates GPS innovations on turns -> false "unhealthy" flags. Harmless under SBAS/dropout, so zero downside to the fallback. Until measured, `rtk_health_node` should WARN that base_link RTK accuracy is not realized while FIX/FLOAT.

---

## 6. Open questions needing field data

1. Does the MTi-680G NavSatFix covariance actually shrink on FIX and inflate on FLOAT/standalone, or is it flat/optimistic? Determines whether the relay must *synthesize* covariance from GGA and whether #9 can ever enable.
2. Is `/imu/data` heading true-north ENU under General_RTK, measured **while moving and after warmup**? Expected yes (COG-derived, no mag) -> declination stays 0.0.
3. Measured lever arm vector, and the driver's expected axis/sign convention (sign of Z)?
4. Does the boot log confirm General_RTK, and which `mti_filter_option` index maps to it in this tree (do not assume 0)?
5. Does NS-IMAX-MSM4 carry RTCM 1005/1006 (ARP) and 1230 (GLONASS bias)? Without 1005/1006 the F9P cannot FIX.
6. Actual `enable_ntrip` defaults in `sensors.launch.py` and `localization.launch.py`? CLAUDE.md says sensors=TRUE; nav2 agent says navigation L135=false, localization L58=true. Resolve before unifying.
7. Has IGVC ruled on public CORS/VRS NTRIP under §I.2? Pending (MEMORY `project_igvc_rtk_rule`).
8. Surveyed OU AutoNav course-start lat/lon (+ any start heading in rad), and delta from the practice datum?
9. On the live device under sustained FIX with COG heading at speed, does `dist_remaining` converge on map-referenced goals, or does the 65-96 deg rotation persist? Determines whether the heading gate can open without dual-antenna/mag hardware.
10. Does `navsat.yaml` flag Phase B decommission of `navsat_transform` (issue #12)? Weigh datum/declination investment against that migration.
11. Bench-measured `map->odom` step on FIX->FLOAT->standalone with the floor in place — absorbed, or still snaps enough to disturb odom control?

---

## Appendix — verified anchor facts (line numbers as read by subsystem agents)

- `ekf.yaml`: `odom0: /odometry/gps` (L180), `odom0_differential: true` (L186), `odom0_pose_rejection_threshold: 13.8` (L194), x/y `process_noise 0.5` (L209-210); GPS block **active, not commented**; raised 4.0->13.8 and 0.1->0.5 on 2026-05-28 (lockout / Test 2.1 36 m gap).
- `navsat.yaml`: `datum: [42.667925,-83.218195,0.0]` (L39, slot-3=heading guard), `magnetic_declination_radians: 0.0` (L16), `yaw_offset: 0.0` (L17), `use_odometry_yaw: false` (L20), `publish_filtered_gps: true` (L40), `broadcast_cartesian_transform: false` (L41), `wait_for_datum: true` (L21).
- `nav2_params_humble.yaml`: `bt_navigator.global_frame: odom` (L21), `local_costmap` odom (L243), `global_costmap` odom (L502, rolling L504, STVL-only L518-528), `route_server` odom (L644); `xy_goal_tolerance: 2.0` (L70), `yaw 0.5` (L71), `goal_reached_tol: 3.0` (L29); L493-499 lethal-cell phantom-accumulation note.
- `mission_manager.py`: functional 294-line node, `GOAL_FRAME_ID='map'` (L45), `/fromLL` called (L140-141), default `acceptance_radius_m=2.0`.
- `xsens.yaml`: `GNSS_LeverArm: [0.0,0.0,0.0]` (L47, TODO).
- `navigation.launch.py`: `enable_ntrip` default `false` (L135, cites IGVC I.2); `localization.launch.py:143` remaps `gps/fix <- /gnss`.
- `filter_base.cpp`: YAML rejection threshold is in **sigma**, squared in code -> 13.8 -> ~190 -> gate effectively OFF (`docs/fresh_audit_2026_05_29.md` N5).
- `sensor_msgs/NavSatStatus`: NO_FIX=-1, FIX=0, SBAS_FIX=1, GBAS_FIX=2 — no RTK enum (confirmed live).

Cited docs: `docs/fresh_audit_2026_05_29.md` (N5/N6/P1 covariance + gate), CLAUDE.md (TF tree, ZED lever-arm precedent, NTRIP, General_RTK no-mag), commit `5e6f279` (odom-frame decision), commit `27d6b41` (datum slot-3 = heading_rad), MEMORY: `project_nav_must_run_in_odom_not_gps_map`, `project_no_rtk_in_codebase`, `project_igvc_rtk_rule`, `feedback_navsat_datum_yaml_semantics`, `feedback_zed_leverarm_in_rl`, `feedback_xsens_quat_needs_motion_warmup`, `project_xsens_param_naming_quirks`. REP-105 (map jumps / odom continuity); robot_localization `integrating_gps`; Nav2 `dual_ekf_navsat` tutorial; u-blox ZED-F9P Integration Manual (1005/1006/1230); Mandow et al. 2007 (skid context).