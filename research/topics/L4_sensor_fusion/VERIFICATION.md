# L4 — Sensor fusion: claim verification

| | |
|---|---|
| **Topic** | L4 — Sensor fusion (`README.md`) |
| **Date** | 2026-09-28 |
| **Reviewer** | independent — claims |
| **Method** | Every cited item was opened at the cited location in `sources/` (PDFs with `pdftotext -layout -f N -l N`, so page numbers are PDF pages; code and docs with `grep -n` / `sed -n`). Sources themselves are not graded here (see `SOURCE_AUDIT.md`). `README.md` was not edited. |

## Counts

| Section | Checked | Verified | Partly supported | Not supported |
|---|---|---|---|---|
| Summary | 6 | 3 | 3 | 0 |
| Foundational references (rows with a downloaded source) | 11 | 8 | 2 | 1 |
| Findings §1–§12 | 248 | 232 | 16 | 0 |
| Recommended practice | 18 | 16 | 2 | 0 |
| Key numbers | 23 | 22 | 1 | 0 |
| How it is tested | 11 | 11 | 0 | 0 |
| Common mistakes | 16 | 14 | 2 | 0 |
| Disagreements | 13 | 10 | 3 | 0 |
| Open questions (cited items) | 7 | 7 | 0 | 0 |
| **Total** | **353** | **323** | **29** | **1** |

Four foundational rows (L4-S05, S06, S11, S12) have no downloaded file and make no checkable claim beyond "not downloaded"; they are listed but not counted.

## Items needing correction (summary)

| # | Problem | Correction |
|---|---|---|
| SUM-3 | "optimal … only when" linear/white/Gaussian — Maybeck states sufficiency, not necessity | "is the optimal estimator when …" (or "under these conditions") |
| SUM-4 | "warns" cited to ekf.cpp 140–146, which is an `FB_DEBUG` (debug-mode only) message; the user-facing WARN diagnostic is in ros_filter.cpp 2501–2509 | cite L4-S25 lines 2501–2509 for the warning; note docs say 1e-6, code 1e-9 |
| SUM-6 | Anderson p. 101 says "By and large, a filter is optimal **if** …"; README says "only if" | quote p. 101 as written, or cite Theorem 6.1 (PDF p. 133) for the "if and only if" |
| FR-S02 | "First description of the unscented filter" — Julier & Uhlmann 1997 itself cites the earlier Julier, Uhlmann & Durrant-Whyte 1995 ACC paper "A New Approach for Filtering Nonlinear Systems" (ref. [11]) | change to "early/standard description", or cite the 1995 paper as first |
| FR-S04 | "First formal treatment" — priority claim not stated in the source | drop "first" or add a source for it |
| FR-S13 | "Widely used … in robotics" — not stated in the source | drop or source it |
| F1.17 | navsat fallback: only `getRobotOriginCartesianPose` (518–559) "assumes device at origin"; `getRobotOriginWorldPose` (566–603) logs "Will not remove offset" and leaves the output pose at identity | describe the two fallbacks separately |
| F1.22 | "map → base_link published directly" is at L4-S60 "Kinematics Fusion Filter"/"Output" (lines 201, 224), not "TF tree" | fix location |
| F2.18 | "indoor" test — source says "commercial environment" | replace "indoor" |
| F4.6, F4.7, F4.8 | Macenski bullets are on PDF p. 17, not p. 18 | change to PDF p. 17 |
| F4.20 | Parenthetical "general principle, applies to any device that runs an internal filter" is an unlabelled generalisation | label as inference |
| F4.21 | Table I shows odometry fused ẋ, ẏ, ż, ψ̇ (not only x and yaw velocity); "x and yaw velocity" text is on PDF p. 2 | fix list and page |
| F6.13 | "likely to discard some good measurements" is in "Detecting and Rejecting Bad Measurement" (line 1002), not "Gating and Data Association Strategies" | fix location |
| F7.9 | Transform is not strictly "computed once": `set_datum` service (line 365) and runtime `magnetic_declination_radians` change (line 921) reset `transform_good_` | add the reset cases |
| F7.20 | Covariance is copied from the fix without regard to status, but is then **rotated** into the world frame (lines 803–820), not "copied unchanged into the output" | reword |
| F7.22 | "differ only in default position error" ignores GST-sentence receiver EPE override (lines 179–184) | add "unless the receiver sends GST" |
| F9.9 | Radar example (Lerro) is about inconsistency only; "biased" belongs to the paper's own polar example | reword |
| F10.7 | Second-IMU/interference text is on PDF p. 4, not p. 5; source gives a second reason (IMU 2 stopped reporting halfway) | fix page, add second reason |
| F10.16 | Issue #630 was not "left open": it stayed closed and the maintainer said he would reopen it if someone submitted a PR | reword |
| F11.9 | "indoor" not in source | "commercial environment" |
| RP-4 | "fuse absolute orientation from the best source only" — S16 says this only when one or both sources under-report covariance; accurate covariances make fusing both "safe" | add the condition |
| RP-5 | PDF p. 18 → p. 17 | fix page |
| KN-18 | "541 m indoor route" | "commercial environment" |
| CM-2 | "replaced silently" — ros_filter.cpp 2501–2509 raises a WARN diagnostic; contradicts F5.6 | drop "silently" |
| CM-14 | Link "low process noise → gain tends to zero → learns wrong state" is the reviewer's chain; p. 143 lists gain→0 as a separate, "not guaranteed" indicator | label as inference or split |
| DIS-3 | L4-S19 "~publish_filtered_gps" gives no default; only L4-S22 says "Defaults to false" | cite only L4-S22 for the default |
| DIS-5 | "(level C vs level B)" is in reverse order to the sentence (S17 = B, S34 = C) | "(level B vs level C)" |
| DIS-9 | "gained acceleration states after the paper" is an inference | label as inference |

## Full claim table

### Summary

| # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|
| SUM-1 | odom continuous/drifts; map no drift/jumps; one parent; localizer publishes map→odom | S09 odom, map, Relationship, Frame Authorities | Verified | "can drift over time, without any bounds"; "each frame can only have one parent"; "broadcast the transform from map to odom" (rep105 l.52–214) | — |
| SUM-2 | Two filter instances suggested; "just a suggestion" | S18 Notes on Fusing GPS | Verified | integrating_gps.rst l.13–16 | — |
| SUM-3 | KF optimal only when linear, white, Gaussian; else BLUE; EKF "ad hoc" | S36 p.10; S35 p.8 | Partly supported | Maybeck p.10: "Under these three restrictions, the Kalman filter can be shown to be the best filter of any conceivable form" (sufficient, not "only when"); Welch p.8 quote exact | "optimal when …" |
| SUM-4 | Zero variance replaced with tiny value and warns; inflation "unnecessary and even detrimental" | S16 Odometry 3, Common errors; S24 l.140–146 | Partly supported | S16 l.64, l.103 ✓; ekf.cpp l.140–146 floor 1e-9 with `FB_DEBUG` only (debug build output); WARN diagnostic is at ros_filter.cpp l.2501–2509 (not cited) | cite S25 l.2501–2509 for the warning |
| SUM-5 | Gates are Mahalanobis, default max(); maintainer: "arbitrary" | S15 threshold; S53 | Verified | nodes.rst l.269; issue 630 "the values are arbitrary" | — |
| SUM-6 | NEES/NIS chi-square with n_x/n_z DOF; optimal only if innovations zero-mean white; mismatch "prime indicator" | S43 p.2; S14 p.11; S54 pp.101,143 | Partly supported | Chen p.2 "χ2 random variables with nx and nz degrees of freedom" ✓; Huang p.11 ✓; Anderson p.143 "prime indicator" ✓; p.101 "By and large, a filter is optimal **if** … zero mean and white" (not "only if") | match p.101 wording or cite Thm 6.1 p.133 |

### Foundational references

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| FR-S01 | Kalman 1960 introduced recursive KF | S01 | Verified | Paper is the KF paper (Theorem 2, recursive model) | — |
| FR-S02 | Julier & Uhlmann 1997: first description of the unscented filter | S02 | Not supported | S02 refs [11]: "S. J. Julier, J. K. Uhlmann and H. F. Durrant-Whyte. A New Approach for Filtering Nonlinear Systems … ACC … 1995" (earlier) | reword or cite 1995 |
| FR-S03 | Julier & Uhlmann 2004 standard journal UKF treatment, limits of linearisation | S03 | Verified | Proc. IEEE invited paper; §II linearisation limits | — |
| FR-S04 | Smith & Cheeseman: first formal treatment of compounding/merging | S04 | Partly supported | Compounding and merging defined p.2; no priority claim in source | drop "first" |
| FR-S05 | Bar-Shalom 2001 not downloaded | — | not checkable | no file | — |
| FR-S06 | Thrun 2005 not downloaded | — | not checkable | no file | — |
| FR-S07 | Moore & Stouch introduced robot_localization | S07 | Verified | p.2 "ekf_localization_node, as the first component of robot_localization" | — |
| FR-S08/09 | REP-103/105 official conventions | S08, S09 | Verified | REP headers | — |
| FR-S15–27 | rl 3.5.4 docs/configs/source = primary spec | S15–S27 | Verified | files present, tag 3.5.4 | — |
| FR-S10 | Maintainer's own guidance | S10 | Verified | ROSCon 2015 slides by T. Moore | — |
| FR-S11 | Mehra 1970 not downloaded | — | not checkable | no file | — |
| FR-S12 | Bar-Shalom 2002 not downloaded | — | not checkable | no file | — |
| FR-S13 | Larsen: widely used delayed-measurement method in robotics | S13 | Partly supported | method described p.2–6; "widely used in robotics" not in source | drop or source |
| FR-S14 | Huang: basic source of EKF inconsistency + fixes | S14 | Verified | p.1 abstract | — |
| FR-S54 | Anderson & Moore: whiteness, divergence, fixed-lag smoothing | S54 | Verified | pp.101, 142–144, 186 | — |

### Findings §1 — Frame conventions

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F1.1 | Right-handed; x fwd y left z up; ENU; SI | S08 Units, Chirality, Axis | Verified | rep103 "All systems are right handed"; "x forward y left z up"; ENU list | — |
| F1.2 | Yaw CCW, zero east; drivers transform | S08 Rotation Representation | Verified | "yaw is zero when pointing east … Hardware drivers should make the appropriate transformations" | — |
| F1.3 | `_ned` suffix; nearby origin for float32 | S08 Suffix, Axis | Verified | "_ned suffix"; "choose a nearby origin such as your system's starting position" | — |
| F1.4 | base_link any position | S09 base_link | Verified | "any arbitrary position or orientation" | — |
| F1.5 | odom quotes | S09 odom | Verified | l.53–65 exact | — |
| F1.6 | map quotes | S09 map | Verified | l.72–83 exact | — |
| F1.7 | Earth-referenced map default ENU | S09 Map Conventions | Verified | "x-axis east, y-axis north, and the z-axis up" | — |
| F1.8 | One-parent quote | S09 Relationship | Verified | l.161–163 exact | — |
| F1.9 | Frame authorities | S09 Frame Authorities | Verified | l.207–214 | — |
| F1.10 | Skid steer open-loop quote | S09 odom Frame Consistency | Verified | l.236 exact | — |
| F1.11 | ≈83 km | S09 odom Frame Consistency | Verified | l.247 "approximately 83km" | — |
| F1.12 | Pose→world_frame, twist→base_link_frame | S16 Coordinate Frames | Verified | preparing l.33–35 | — |
| F1.13 | Rotated IMU handled by static tf | S16 Coordinate Frames | Verified | l.40 "will automatically correct for the orientation" | — |
| F1.14 | Twist lever-arm term added | S25 l.3176–3180, 3226–3230 | Verified | `twist_lin = basis*twist_lin + getOrigin().cross(state_twist_rot)` | — |
| F1.15 | Accel lever arm not handled (@todo) | S25 l.2657–2665 | Verified | "@todo: This needs to take into account offsets from the origin" | — |
| F1.16 | Differential: translation zeroed (3016), tagged base_link (3056) | S25 l.3016, 3056 | Verified | `setOrigin(0,0,0)`; `frame_id = base_link_frame_id_` | — |
| F1.17 | navsat removes antenna offset; if tf missing logs error and assumes origin | S26 l.518–559, 566–603 | Partly supported | l.551–556 "Will assume navsat device is mounted at robots origin" ✓; l.596–602 "Will not remove offset …" and `robot_odom_pose` stays identity | distinguish the two fallbacks |
| F1.18 | Nav2 tf chain; sensor frames via URDF/RSP | S30 Transforms in Nav2; S32 §2 | Verified | S30 l.88–104; S32 l.163 | — |
| F1.19 | Nav2 GPS uses base_footprint | S32 Local Odometry; S33 | Verified | "base_link_frame: base_footprint" | — |
| F1.20 | Compounding and merging (first order, Kalman gain) | S04 pp.2–6 | Verified | p.2 compounding/merging; p.5–6 "based on the use of the Kalman filter equations" | — |
| F1.21 | Small errors, first-order, independent sensor errors | S04 p.2 | Verified | "errors are 'small,' … first-order model … sensor errors are independent of the locational error" | — |
| F1.22 | Autoware earth→map→base_link; localizer publishes map→base_link; odom/base_footprint optional | S59 TF tree; S60 TF tree | Partly supported | tree + "as long as the tf structure above is maintained" (S60 l.243) ✓; "Produces tf of map to base_link" is at S60 l.201/224 (Output / Kinematics Fusion Filter), not "TF tree" | fix location |
| F1.23 | base_link rear-axle ground projection; map ENU; UTM/MGRS | S59, S60 TF tree | Verified | S60 l.240; S59 map bullet | — |
| F1.24 | x_by_y naming, base_link_by_gnss_ins | S59 Estimating base_link | Verified | "x: estimated frame name / y: localization method/source" | — |
| F1.25 | gnss_poser tf shift, untransformed fallback | S62 Design; S26 l.518–559 | Verified | "If the transformation … cannot be obtained, it outputs the pose of the antenna position" | — |

### Findings §2 — Kalman filter theory

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F2.1 | Orthogonal projection optimal (Gaussian / linear+squared loss) | S01 p.4 Thm 2, remark (e) | Verified | Thm 2 conditions (A)/(B); remark (e) | — |
| F2.2 | KF predict/update equations | S35 p.6 Fig 1-2 | Verified | Fig 1-2 equations | — |
| F2.3 | Noises independent, white, normal | S35 p.2 | Verified | "independent (of each other), white, and with normal probability distributions" | — |
| F2.4 | Mean/mode/median coincide; BLUE quote | S36 p.10 | Verified | quote exact | — |
| F2.5 | White noise tractable, identical in bandwidth | S36 p.11 | Verified | "identical to the real wideband noise … made tractable" | — |
| F2.6 | EKF linearises about mean and covariance | S35 pp.7–8 | Verified | p.7 "linearizes about the current mean and covariance" | — |
| F2.7 | EKF "fundamental flaw … ad hoc" quote | S35 p.8 | Verified | exact | — |
| F2.8 | "difficult to implement, difficult to tune …" consensus | S02 p.1; S03 p.1 | Verified | S02 p.1 quote exact | — |
| F2.9 | Linearisation biased, inconsistent; padding | S03 pp.3–4 | Verified | p.3 "biased and inconsistent … underestimates the variance"; p.4 "padding" | — |
| F2.10 | UT second order, no Jacobians, "same order of magnitude as the EKF" | S03 p.6 | Verified | p.6 properties 2–3 | — |
| F2.11 | rl EKF & UKF same model; UKF quotes | S15 ukf_localization_node | Verified | nodes.rst l.10 | — |
| F2.12 | alpha 0.001, kappa 0, beta 2 | S15 ukf params | Verified | l.312–316 | — |
| F2.13 | ESKF properties | S42 §5.1 pp.52–53 | Verified | p.52–53 bullets; nominal/error-state p.53 | — |
| F2.14 | IEKF autonomous error, local stability, "identical tuning keeps converging" | S41 p.1 | Verified | abstract | — |
| F2.15 | EKF gain "may amplify the error" | S41 pp.1–2 | Verified | "unadapted gain that may amplify the error" | — |
| F2.16 | Factor graphs = NLS; fuse fixed-lag smoother | S39 pp.17–18; S40 Overview | Verified | S39 Eq.9, "fixed-lag smoother"; S40 "nonlinear least squares" | — |
| F2.17 | fuse advantages; higher compute | S39 p.18 | Verified | bullet list p.18 | — |
| F2.18 | 541 m **indoor** test; 1.44 m closer; <1 %; 3.7× CPU | S39 pp.18–19 | Partly supported | p.18 "541-meter long route through a commercial environment"; 1.44 m, 1 %, 3.7x ✓; "indoor" not stated | replace "indoor" |
| F2.19 | Factor-graph smoothing: async/delayed main advantage | S49 p.1 | Verified | abstract quote | — |
| F2.20 | Angle innovations wrapped | S24 l.176–183 | Verified | `normalize_angle(innovation_subset(i))` | — |
| F2.21 | Joseph form valid any K; asymmetry "usually leads … diverging" | S37 Stable Compution | Verified | l.615 quote; "valid for *any* K" | — |
| F2.22 | rl uses Joseph form | S24 l.194–202 | Verified | "(4) … Joseph form" | — |
| F2.23 | CI formula, ω∈[0,1], det/trace; quotes | S56 pp.1–3 Eqs 13–15 | Verified | Eq.14 "(ω(Π_A)⁻¹+(1−ω)(Π_B)⁻¹)⁻¹"; "0 ≤ ω ≤ 1"; "considers all admissible cross-correlations … best guaranteed quality" | — |
| F2.24 | CI+KF recursive not optimal | S56 pp.1, 6 | Verified | "cannot be obtained recursively" | — |
| F2.25 | REP-103 prefers quaternions; rl state Euler | S08; S07 p.2 | Verified | "Rotational values are expressed as Euler angles" | — |

### Findings §3 — Motion models

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F3.1 | 15-variable state, omnidirectional 3-D | S39 p.17 Eq.8; S15 | Verified | Eq.8; nodes.rst l.6 | — |
| F3.2 | 2014: 12-variable state | S07 p.2 | Verified | "Our 12-dimensional state vector" | — |
| F3.3 | No other model; unicycle by clamping | S39 p.17 | Verified | quote | — |
| F3.4 | two_d_mode quote | S15 two_d_mode | Verified | l.37 exact | — |
| F3.5 | forceTwoD sets 1e-6, marks measured | S25 l.339–385 | Verified | l.371–385 | — |
| F3.6 | two_d_mode "planar environment" quote | S15 | Verified | l.37 | — |
| F3.7 | Nav2 2-D mode reason quote | S32 Local Odometry | Verified | l.173 | — |
| F3.8 | Zero ẏ "perfectly valid measurement" | S17 item 2; S07 p.5 | Verified | configuring l.74; S07 p.5 "fuse the zero values" | — |
| F3.9 | Maintainers recommend zero ẏ | S39 pp.17–18 | Verified | p.17 bullet quote | — |
| F3.10 | Two nonholonomic constraints as virtual observations | S46 pp.3–4 | Verified | "two nonholonomic constraints"; "virtual observation" | — |
| F3.11 | Velocity/attitude observable; position unobservable w/o GPS | S46 p.1, p.12 | Verified | abstract; p.12 "position error since this is unobservable with the use of constraints alone" | — |
| F3.12 | No acceleration ref → sluggish | S20 above use_control | Verified | ekf.yaml l.181–186 | — |
| F3.13 | use_control, limits, IMU accel overrides | S15; S20 | Verified | nodes.rst l.190–192; ekf.yaml l.202–204 | — |
| F3.14 | Q integral; "engineering" factor | S37 Continuous White Noise | Verified | l.413, l.425 | — |
| F3.15 | Simplified Q quote | S37 Simplification of Q | Verified | l.599 | — |
| F3.16 | σ²(t3⁻) = σ²(t2)+σw²(t3−t2) | S36 p.17 | Verified | Eq.1-12 | — |
| F3.17 | delta_sec × Q | S24 l.427–431 | Verified | l.430–431 | — |

### Findings §4 — Measurement modelling

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F4.1 | _config order; in sensor frame_id | S15 [sensor]_config; S17 Sensor Configuration | Verified | nodes.rst l.105–107; configuring l.16 | — |
| F4.2 | H m×12 selector, partial updates | S07 p.2 | Verified | "m by 12 matrix of rank m" | — |
| F4.3 | Odometry: fuse velocity / fuse orientation | S16 Odometry 1 | Verified | l.57–58 | — |
| F4.4 | "duplicate information", fuse velocities | S17 item 1 | Verified | l.54 | — |
| F4.5 | IMU Ÿ "can cause your estimate to drift rapidly" | S17 item 3 | Verified | l.87 | — |
| F4.6 | Every dimension referenced; one pose / one pose + many velocities | S39 p.18 | Partly supported | text is on PDF p.17 ("Every linear and rotational dimension must have a reference") | p.17 |
| F4.7 | Linear accel "generally insufficient" | S39 p.18 | Partly supported | on PDF p.17 | p.17 |
| F4.8 | Minimum set, one at a time | S39 p.18 | Partly supported | on PDF p.17 ("Start with the minimum set … add inputs one-at-a-time") | p.17 |
| F4.9 | ERROR diagnostic "Neither … nor its velocity" | S25 l.1726–1752 | Verified | l.1743–1750 (only when `print_diagnostics_`) | — |
| F4.10 | _differential behaviour; variance "will grow without bound" | S15 [sensor]_differential | Verified | l.129–131 | — |
| F4.11 | Rule of thumb N−1 | S17 differential section | Verified | l.140 | — |
| F4.12 | 1.5 m vs 1.5 rad example | S17 | Verified | l.138 | — |
| F4.13 | Two IMUs variance 0.1 oscillate; overlap | S17 | Verified | l.134 | — |
| F4.14 | "This may result in oscillations" | S25 l.1713–1724 | Verified | l.1720 | — |
| F4.15 | _relative; both true → differential | S15; S25 l.1119–1124 | Verified | "Using differential mode." | — |
| F4.16 | GPS _differential false; "defeats the purpose" | S18; S19 note | Verified | gps.rst l.135; navsat.rst l.6 | — |
| F4.17 | ENU IMU; "does not work with NED" | S16 | Verified | l.38 | — |
| F4.18 | Accelerometer signs ±9.81 | S16 IMU 3 | Verified | l.82–84 | — |
| F4.19 | REP-145 frame_id, ENU "relative to magnetic north", yaw arbitrary | S28 Frame Conventions | Verified | l.35, 37, 41 | — |
| F4.20 | GNSS output filtered; "violated …"; "diverges from the track"; general principle | S38 Exercise | Partly supported | quotes at l.1451, l.1481 ✓; "(general principle, applies to any device that runs an internal filter)" is not in the source | label as inference |
| F4.21 | Moore: IMUs rpy+rates, **x and yaw velocity** from wheels, GPS x/y/z; 69.65/160.33 → 1.21/0.26 | S07 p.3 Tables I–II | Partly supported | Table II numbers ✓; Table I odometry row fuses x′, y′, z′, ψ′ (1 1 1 … 1); "x and yaw velocity" text is on p.2 | fix list/page |
| F4.22 | LC vs TC quotes | S58 p.2 | Verified | "easier to implement"; "more complicated but can still be valid without enough satellites" | — |
| F4.23 | 3 satellites: 10 m in 1 min; few metres after several minutes | S58 p.1 | Verified | abstract | — |
| F4.24 | NHC "most common and effective" | S58 p.2 | Verified | quote exact | — |

### Findings §5 — Covariances

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F5.1 | R measured prior; Q "generally more difficult" | S35 p.6 | Verified | quotes | — |
| F5.2 | P, K "stabilize quickly" | S35 p.6 | Verified | quote | — |
| F5.3 | R→0 trust measurement; P⁻→0 trust prediction | S35 pp.3–4 | Verified | p.4 "trusted more and more" | — |
| F5.4 | "Covariance values **matter**" | S16 Odometry 3 | Verified | l.64 | — |
| F5.5 | Inflation 1e3 "unnecessary and even detrimental" | S16 Odometry 3, Common errors | Verified | l.64, l.102 | — |
| F5.6 | Zero variance → small value + warning quote | S25 l.2502–2509; S16 | Verified | "should be corrected at the message origin" | — |
| F5.7 | Floor 1e-9, "blow up"; abs of negative | S24 l.120–147 | Verified | l.123–146 | — |
| F5.8 | REP-145 unknown cov 0; unreported −1 | S28 Topics | Verified | l.110 | — |
| F5.9 | NavSatFix covariance m², ENU, DOP, types | S29 | Verified | msg text | — |
| F5.10 | Moore σ "much smaller than true"; noisier; Q untuned | S07 p.5 | Verified | quotes | — |
| F5.11 | Q "can be difficult to tune"; larger Q faster convergence | S15; S07 p.2 | Verified | nodes.rst l.281 | — |
| F5.12 | Raise Q diagonal if slow | S20 | Verified | ekf.yaml l.214–216 | — |
| F5.13 | Default Q diagonal | S23 l.110–124 | Verified | values match | — |
| F5.14 | dynamic Q by velocity norm | S15; S23 l.129–152 | Verified | "stop growing when the robot is stationary"; `.norm()` | — |
| F5.15 | P0 tiny → "very slow to 'trust'" | S15 | Verified | l.289 | — |
| F5.16 | Velocity-only → not large P0 | S15; S20 | Verified | l.289; ekf.yaml l.236 | — |
| F5.17 | Default P0 1e-9 | S20; S23 l.91 | Verified | — | — |
| F5.18 | First measurement copies values/cov | S23 l.227–247 | Verified | l.233–247 | — |
| F5.19 | sensor_timeout 1/frequency; predict without correct | S15; S25 l.699–708, 877 | Verified | l.877 `1.0 / frequency_` | — |
| F5.20 | Condition number grew rapidly | S07 p.5 | Verified | quote | — |
| F5.21 | Q adapted online | S35 p.7 | Verified | "reduce the magnitude of Q_k if the user seems to be moving slowly" | — |
| F5.22 | NEES/NIS cost, Bayesian optimisation | S43 pp.1, 5 | Verified | abstract; §IV-B | — |
| F5.23 | "most often done manually" | S43 p.3 | Verified | quote | — |
| F5.24 | Adaptive filters "significantly affected by outliers" | S44 p.1 | Verified | abstract | — |
| F5.25 | Four groups; Mehra first; Odelson ALS | S55 PREVIOUS WORK | Verified | md l.17, 25, 27 | — |
| F5.26 | Covariance matching; "never been proved" | S55 | Verified | l.23 | — |
| F5.27 | Identifiability via rank; Odelson counter-example | S55 IDENTIFIABILITY | Verified | l.67; abstract | — |
| F5.28 | A&M remedies; "easiest" | S54 p.144 | Verified | items 1–4; "easiest techniques" | — |
| F5.29 | Autoware proc_stddev from physical limits | S61 §2 | Verified | l.130–131 | — |

### Findings §6 — Outlier rejection

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F6.1 | KF "provides no way to detect and reject" | S38 Detecting | Verified | l.917 | — |
| F6.2 | γ = ẑᵀS⁻¹ẑ ~ χ²_m; threshold at α | S45 p.7 Eqs 32–36; S44 p.5 | Verified | Eqs 32–36; S44 Eq.21 | — |
| F6.3 | 12.592, 95 %, 6 DOF | S45 p.10 | Verified | "12.592 … 95% (α = 0.05) and 6 degrees of freedom" | — |
| F6.4 | Inflate R by scale factor | S45 pp.1, 8 | Verified | abstract; p.8 iteration | — |
| F6.5 | Threshold Mahalanobis, default max() | S15; S25 l.1128–1137 | Verified | l.1131, 1137 | — |
| F6.6 | Pose/twist parts, not per variable | S20; S15 | Verified | ekf.yaml l.122 | — |
| F6.7 | Compares squared distance with n_sigmas² over all fused vars | S23 l.431–450; S24 l.185–189 | Verified | l.435–437 | — |
| F6.8 | No DOF scaling; chi-square depends on DOF | S23 l.435–439; S45 p.7 | Verified | code | — |
| F6.9 | "unsigned Z-score"; "arbitrary" | S53 | Verified | quotes | — |
| F6.10 | "strongly recommended … removed if not required" | S20 | Verified | ekf.yaml l.121 | — |
| F6.11 | Rejected → no update | S24 l.185–211 | Verified | update inside `if (checkMahalanobisThreshold…)` | — |
| F6.12 | Maintainer points to thresholds; author's lock-out quote | S52 | Verified | issue text | — |
| F6.13 | 3σ "likely to discard some good measurements"; "Theory says 3 std…" | S38 Gating | Partly supported | 2nd quote in Gating (l.1091) ✓; 1st quote is at l.1002 in "Detecting and Rejecting Bad Measurement" | fix location |
| F6.14 | Rectangular gates pass more spurious | S38 Gating | Verified | "rectangular gate is twice as likely to accept a bad measurement" | — |
| F6.15 | Switch variables; ~21 %; 0 or 1 | S48 pp.3–5 | Verified | p.5 "roughly 21% have been declared an outlier" | — |
| F6.16 | LS not robust; single outlier catastrophic | S48 p.3 | Verified | quote | — |
| F6.17 | Autoware gate DOF and table values | S61 §3 | Verified | table | — |
| F6.18 | "accuracy of covariance estimation itself is not very good" | S61 §3 | Verified | quote | — |
| F6.19 | EKF FDE "prone to high false alarm" | S57 p.13 | Verified | quote | — |

### Findings §7 — GNSS integration

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F7.1 | GPS jumps; "unfit" | S18 | Verified | l.11 | — |
| F7.2 | Suggested dual setup | S18 | Verified | l.13–14 | — |
| F7.3 | "something else"; "should *not* fuse the global data" | S15 [frame] 3 | Verified | l.59–60 | — |
| F7.4 | Dual-EKF inputs | S21 | Verified | odom0 vx,vy,vz,vyaw; imu0 roll,pitch, rates, accels; odom1 x,y in map | — |
| F7.5 | Q x,y 1.0 vs 1e-3; P0 1.0 vs 1e-9 | S21 | Verified | yaml l.43–44/125–126; l.59–60/141–142 | — |
| F7.6 | "a common setup" quote | S32 §2 | Verified | l.167 | — |
| F7.7 | Three inputs | S18 Required Inputs; S19 Subscribed | Verified | gps.rst l.32–40 | — |
| F7.8 | UTM/local cartesian; transform from first fix | S10 slide 14; S26 l.102, 247–330 | Verified | slide 14 "Transform all future GPS measurements using T" | — |
| F7.9 | Computed once; not recomputed; IMU unused after | S26 l.247–330, 660–664 | Partly supported | l.323 `transform_good_ = true`; l.660 early return ✓; but `transform_good_ = false` at l.365 (set_datum service) and l.921 (magnetic_declination change) | note reset cases |
| F7.10 | Heading formula; convergence 0 local cartesian | S26 l.270–299 | Verified | l.285–296 | — |
| F7.11 | IMU zero east; north → π/2 | S19 yaw_offset; S18 | Verified | navsat.rst l.25 | — |
| F7.12 | Declination quote | S19 | Verified | l.21 | — |
| F7.13 | use_odometry_yaw earth-referenced | S19; S22 | Verified | navsat.rst l.41; yaml comment | — |
| F7.14 | datum form; wait_for_datum true | S18; S22 | Verified | gps.rst l.44–52 | — |
| F7.15 | Automatic vs fixed datum | S32 Navsat Transform | Verified | l.226–228 | — |
| F7.16 | delay quote | S22 | Verified | "especially important if you have use_odometry_yaw set to true" | — |
| F7.17 | "mandatory"; IMU heading problems | S32 Overview | Verified | l.60–66 | — |
| F7.18 | "initialization dance"; dual GPS good | S32 | Verified | l.68 | — |
| F7.19 | 1–2 m / 10 m / 1 cm | S32 Overview | Verified | l.56 | — |
| F7.20 | Discard only NO_FIX/NaN; covariance copied **unchanged into the output** | S26 l.619–652 | Partly supported | l.619–622 ✓; l.643 "Copy … so that we can rotate it later"; rotated at l.803–820 | reword |
| F7.21 | NavSatStatus four values | S29 | Verified | msg | — |
| F7.22 | RTK fixed/float both GBAS; **differ only** in default error × HDOP | S51 l.58–64, 75–117, 179–191 | Partly supported | mapping and defaults ✓; l.179–184 receiver EPE from GST replaces defaults when available | add GST case |
| F7.23 | Default EPE values | S51 l.59–64 | Verified | 1000000, 4.0, 0.1, 0.02, 4.0, 3.0 | — |
| F7.24 | Infrequent GPS; 12.06, 0.52 m | S07 p.5 | Verified | quotes | — |
| F7.25 | Rovitis problems; rl "did not perform well" | S47 pp.1, 3–4 | Verified | abstract | — |
| F7.26 | Autoware smooth update | S61 Features, §1 | Verified | l.21, l.121–122 | — |
| F7.27 | ~10 m / ~10 cm; blocked signals | S60 GNSS | Verified | l.72–77 | — |
| F7.28 | AL/PL/TTA/IR; 10⁻⁷–10⁻⁹; not set urban | S57 pp.4–5 | Verified | p.4, p.5 | — |
| F7.29 | RAIM large errors, single faults; open sky vs urban | S57 pp.6, 13 | Verified | p.6 bullet; p.13 | — |

### Findings §8 — Delayed measurements

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F8.1 | "not a trivial problem"; trade-off | S13 p.2 | Verified | abstract | — |
| F8.2 | Four options compared | S13 pp.3–6 | Verified | p.6 list A–D | — |
| F8.3 | Extrapolation optimal if none fused in delay | S13 p.5 | Verified | "If no measurements has been fused in the delay period … will be optimal" | — |
| F8.4 | Recalculation quote | S13 p.6 | Verified | quote | — |
| F8.5 | smooth_lagged_data / history_length | S15; S25 l.612–660 | Verified | nodes.rst l.239, 243 | — |
| F8.6 | Revert failure → processed without revert | S25 l.634–650 | Verified | code | — |
| F8.7 | Old meas: no predict, correct only | S23 l.205–226 | Verified | "Only want to carry out a prediction if it's forward in time. Otherwise, just correct." | — |
| F8.8 | Older than previous on topic dropped; "bad timestamp" | S25 l.204–262; 1953, 2379 | Verified | l.206, 257, 1955, 2381 | — |
| F8.9 | Before set_pose ignored | S25 l.1822–1830 | Verified | quote | — |
| F8.10 | permit_corrected_publication | S15 | Verified | l.171 | — |
| F8.11 | predict_to_current_time | S15 | Verified | l.297 | — |
| F8.12 | transform_timeout | S15 | Verified | l.70 | — |
| F8.13 | queue_size | S15 | Verified | "useful if your frequency parameter value is much lower than your sensor's frequency" | — |
| F8.14 | Ignoring delay "high systematic errors" | S49 pp.2, 6 | Verified | p.6 | — |
| F8.15 | Fixed-lag via augmented model | S54 pp.186–188 | Verified | p.186 §7.3 | — |
| F8.16 | Autoware augmented state; cost; WARN | S61 | Verified | "does not significantly change"; WARN bullet | — |
| F8.17 | Autoware preliminaries | S61 §0 | Verified | l.115–116 | — |

### Findings §9 — Consistency and evaluation

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F9.1 | NEES/NIS defs, χ² n_x/n_z | S43 p.2 | Verified | Eqs 18–19 | — |
| F9.2 | NEES needs truth; NIS online | S43 pp.1–2 | Verified | p.2 | — |
| F9.3 | Average NEES ≈ 3 | S14 p.11 | Verified | quote | — |
| F9.4 | NEES larger as P smaller; below dimension | S38 NEES | Verified | quotes | — |
| F9.5 | "smug"; P theoretical | S38 Evaluating Filter Order | Verified | l.595 | — |
| F9.6 | Residual vs 3σ plots | S38 | Verified | l.496–498 | — |
| F9.7 | EKF-SLAM inconsistency | S14 pp.1, 7 | Verified | quote p.1 | — |
| F9.8 | FEJ, OC-EKF | S14 pp.1–2, 9–10 | Verified | abstract | — |
| F9.9 | Linearised conversions **biased** and inconsistent with small bearing errors (radar) | S03 p.3 | Partly supported | radar (Lerro) sentence says only "can become inconsistent" when bearing σ < 1°; "biased" refers to the paper's own example | reword |
| F9.10 | RPE / ATE | S50 p.2 | Verified | quote | — |
| F9.11 | Time sync measured | S50 pp.4, 6 | Verified | §D p.6 | — |
| F9.12 | Loop closure + σ comparison | S07 p.3 | Verified | text p.3 | — |
| F9.13 | amcl ground truth; 541 m; 10 cm | S39 p.18 | Verified | quote | — |
| F9.14 | Maybeck four questions | S36 pp.5–6 | Verified | question (4) | — |
| F9.15 | Innovations white; p.101 quote | S54 pp.100–101 | Verified | p.100 "innovations process is white"; p.101 quote | — |
| F9.16 | Thm 6.1 iff; Thm 6.2 finite lags | S54 pp.132–134 | Verified | p.133 Thm 6.1; p.134 Thm 6.2 | — |
| F9.17 | Stationary, ergodic | S54 p.132 | Verified | quote | — |
| F9.18 | Correlated if gain not optimal; Mehra | S55 ESTIMATION OF W | Verified | l.119–121 | — |

### Findings §10 — Failure modes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F10.1 | Smug divergence | S38 | Verified | l.595 | — |
| F10.2 | EKF amplifies error | S41 pp.1–2 | Verified | quote | — |
| F10.3 | Oscillation | S17; S20 | Verified | configuring l.134; ekf.yaml l.103–105 | — |
| F10.4 | Unbounded yaw covariance | S15; S17 | Verified | — | — |
| F10.5 | Unmeasured variables | S25 l.1726–1752 | Verified | — | — |
| F10.6 | Condition number | S07 p.5 | Verified | — | — |
| F10.7 | Magnetometer interference; second IMU barely helped | S07 p.5 | Partly supported | text is on PDF p.4; source also says IMU 2 "stopped reporting data halfway" | p.4; add reason |
| F10.8 | Slip, vibration, RTK loss | S47 pp.1, 3–4 | Verified | — | — |
| F10.9 | Common errors list | S16 | Verified | l.100–103 | — |
| F10.10 | Driver odom→base_link broadcast | S16 Odometry 5 | Verified | l.70 | — |
| F10.11 | map jumps | S09; S18 | Verified | — | — |
| F10.12 | Gating lock-out | S52 | Verified | quote | — |
| F10.13 | Bad timestamps | S25 | Verified | — | — |
| F10.14 | Missing tf → skip offset correction with error | S26 l.551–603 | Verified | l.551–556, 590–602 | — |
| F10.15 | print_diagnostics; /diagnostics_agg; debug | S15; S20 | Verified | nodes.rst l.175, l.273; ekf.yaml l.29 | — |
| F10.16 | Debug output of rejection; issue "left open pending a contributed fix" | S23 l.440–446; S53 | Partly supported | debug fields ✓; S53: "Can you reopen?" → "I'll reopen it if someone would be willing to PR the fix" (i.e. stayed closed) | reword |
| F10.17 | reset_on_time_jump | S15 | Verified | l.293 | — |
| F10.18 | Switch variables remove multipath | S48 pp.1, 5 | Verified | — | — |
| F10.19 | Divergence definition, quotes | S54 pp.142–143 | Verified | p.142–143 | — |
| F10.20 | Gain → 0, "learned the wrong state" | S54 p.143 | Verified | quote (source adds "not always encountered, and not a guaranteed indicator") | — |
| F10.21 | Autoware diagnostics | S61 Diagnostics | Verified | WARN/ERROR lists | — |
| F10.22 | "buried in the ground" | S61 Features | Verified | l.22 | — |
| F10.23 | One yaw bias known issue | S61 | Verified | "would not make any sense" | — |
| F10.24 | Destabilising situations | S60 sections | Verified | l.105, 119, 136, 72 | — |

### Findings §11 — Reference configurations

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F11.1 | ekf.yaml template values | S20 | Verified | l.7, 12, 18, 48, 72, 193, 202 | — |
| F11.2 | Dual-EKF navsat values | S21 | Verified | yaml l.160–164 | — |
| F11.3 | Nav2 GPS demo config | S33 | Verified | yaml | — |
| F11.4 | imu0_differential comment | S33 | Verified | l.32 | — |
| F11.5 | Nav2 smoothing guide single EKF | S31 | Verified | intro l.3; §Configuring | — |
| F11.6 | Clearpath A200: 50 Hz, 2-D, odom x,y,yaw,vx,vy,vyaw | S34 | Verified | yaml | — |
| F11.7 | 1.21, 0.26 m; 777 s; 110 m | S07 p.3 | Verified | text p.3 | — |
| F11.8 | Rovitis 0.005 ± 0.220 m; 0.6° ± 3.5° | S47 p.1 | Verified | abstract | — |
| F11.9 | Comparison **indoor**, 30/20 Hz, 3.7× | S39 pp.18–19 | Partly supported | Table V 30 Hz / 20 Hz, 3.7x ✓; "indoor" not stated ("commercial environment") | fix |
| F11.10 | Autoware architecture; 50Hz~; 2-D model + yaw bias | S60; S61 | Verified | S60 l.187–229; S61 Kalman Filter Model | — |

### Findings §12 — robot_localization in Humble

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F12.1 | 3.5.4 2025-08-29; #854 (3.5.3); #834 (3.5.2) | S27 | Verified | changelog | — |
| F12.2 | #942, #920 | S27 3.5.4 | Verified | — | — |
| F12.3 | #835 | S27 3.5.2 | Verified | — | — |
| F12.4 | broadcast_utm_transform deprecated; use_local_cartesian | S26 l.102, 109–134, 337 | Verified | l.112–115 warning; l.337 "local_enu" | — |
| F12.5 | Trailing-underscore parameter name | S26 l.123; S19 | Verified | `"broadcast_utm_transform_as_parent_frame_"` | — |
| F12.6 | publish_filtered_gps default true | S26 l.99 | Verified | code | — |
| F12.7 | print_diagnostics default false | S25 l.762 | Verified | code | — |
| F12.8 | sensor_timeout 1/frequency | S25 l.877; S20 | Verified | — | — |
| F12.9 | ROS 1 details; Nav2 links melodic/jade/noetic | S15; S31; S32 | Verified | `tcpNoDelay`, `isSimTime`; S31 l.13/99; S32 l.54/165 | — |
| F12.10 | Garmin 18x quote | S18 GPS Data | Verified | l.60 | — |
| F12.11 | INS time-correlated outputs (labelled general principle) | S38; S35 p.2 | Verified | inference is labelled | — |

### Recommended practice

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| RP-1 | REP-103/105, check signs | S16; S08 | Verified | Odometry item 4 | — |
| RP-2 | Static tf to every sensor | S30; S16 | Verified | — | — |
| RP-3 | Honest covariances, use _config | S16 | Verified | — | — |
| RP-4 | Fuse velocities incl. zero ẏ; orientation **from the best source only** | S17; S16 item 1; S39 | Partly supported | S16 note: fusing both orientations is "safe" if covariances accurate; best-source-only only when one under-reports | add condition |
| RP-5 | Reference every dimension, minimum set | S39 p.18 | Partly supported | on PDF p.17 | p.17 |
| RP-6 | Dual filter; GPS _differential false | S18 | Verified | — | — |
| RP-7 | ENU heading, declination, yaw_offset, delay/datum | S19; S22; S32 | Verified | — | — |
| RP-8 | two_d_mode only planar | S15; S39 p.17 | Verified | — | — |
| RP-9 | Thresholds only where needed; chi-square | S20; S45 p.7 | Verified | — | — |
| RP-10 | smooth_lagged_data + history_length | S15 | Verified | — | — |
| RP-11 | Tune Q, P0; raise Q | S20 | Verified | — | — |
| RP-12 | NIS / NEES / ATE-RPE | S43; S50 | Verified | — | — |
| RP-13 | print_diagnostics | S20 | Verified | S20 says `/diagnostics_agg` | — |
| RP-14 | Joseph form | S37 | Verified | — | — |
| RP-15 | Innovation whiteness check | S54; S55 | Verified | — | — |
| RP-16 | Raise / adapt Q on divergence | S54 p.144 | Verified | — | — |
| RP-17 | CI for unknown correlation | S56 p.1; S38 | Verified | — | — |
| RP-18 | Monitor ellipse / missing updates | S61 | Verified | — | — |

### Key numbers

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| KN-1 | 15 | S39 p.17 | Verified | Eq.8 | — |
| KN-2 | 12 | S07 p.2 | Verified | — | — |
| KN-3 | Default Q diagonal | S23 l.110–124 | Verified | — | — |
| KN-4 | P0 1e-9 | S20; S23 l.91 | Verified | — | — |
| KN-5 | Min variance 1e-9 | S24 l.140–146 | Verified | — | — |
| KN-6 | 1e-6 two_d_mode | S25 l.371–377 | Verified | — | — |
| KN-7 | Threshold max() | S15; S25 | Verified | — | — |
| KN-8 | sensor_timeout 1/frequency | S25 l.877 | Verified | — | — |
| KN-9 | 12.592 | S45 p.10 | Verified | — | — |
| KN-10 | delta_sec × Q | S24 l.431 | Verified | l.430–431 | — |
| KN-11 | NMEA EPE | S51 l.59–64 | Verified | — | — |
| KN-12 | 1–2 m / 10 m | S32 | Verified | — | — |
| KN-13 | 1 cm | S32 | Verified | — | — |
| KN-14 | 83 km | S09 | Verified | — | — |
| KN-15 | Q 1e-3 / 1.0 | S21 | Verified | — | — |
| KN-16 | 1.21, 0.26 m | S07 p.3 | Verified | — | — |
| KN-17 | 69.65, 160.33 m | S07 p.3 | Verified | — | — |
| KN-18 | 3.7×, "541 m **indoor** route" | S39 p.18 | Partly supported | "commercial environment" | fix |
| KN-19 | Autoware gate values | S61 | Verified | — | — |
| KN-20 | ~10 m / ~10 cm | S60 | Verified | — | — |
| KN-21 | 10⁻⁷–10⁻⁹ | S57 p.5 | Verified | — | — |
| KN-22 | 50 Hz~ | S60 | Verified | — | — |
| KN-23 | 10 m in one minute | S58 p.1 | Verified | — | — |

### How it is tested

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| T-1 | NEES test | S43 pp.2–3; S14 p.11 | Verified | — | — |
| T-2 | NIS test | S43 p.2 | Verified | — | — |
| T-3 | Residual vs 3σ | S38 | Verified | l.498 | — |
| T-4 | Loop closure | S07 p.3 | Verified | — | — |
| T-5 | ATE/RPE; errors above GT accuracy | S50 pp.2, 5 | Verified | p.5 "errors significantly above these values" | — |
| T-6 | Infrequent GPS | S07 p.5 | Verified | — | — |
| T-7 | reset_on_time_jump | S15 | Verified | — | — |
| T-8 | Innovation whiteness | S54; S55 | Verified | — | — |
| T-9 | Design vs actual innovations | S54 p.143 | Verified | — | — |
| T-10 | Autoware diagnostics | S61 | Verified | — | — |
| T-11 | PL vs AL; Stanford needs true error | S57 pp.4–5 | Verified | p.5 "the true position error" | — |

### Common mistakes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| CM-1 | Inflating covariances | S16 | Verified | — | — |
| CM-2 | Zero covariances "replaced **silently**" | S16; S24 l.140–146 | Partly supported | replacement ✓; ros_filter.cpp l.2501–2509 issues a WARN diagnostic | drop "silently" |
| CM-3 | Duplicate information | S17 | Verified | — | — |
| CM-4 | Non-overlapping yaw sources | S17 | Verified | — | — |
| CM-5 | Only yaw source differential | S17 | Verified | — | — |
| CM-6 | GPS in odom filter | S15; S18 | Verified | — | — |
| CM-7 | GPS _differential true | S18 | Verified | — | — |
| CM-8 | NED / north-zero heading | S16; S19 | Verified | — | — |
| CM-9 | Double odom→base_link | S16 | Verified | — | — |
| CM-10 | IMU Ÿ | S17 | Verified | — | — |
| CM-11 | Large P0 on velocity-only pose | S15 | Verified | — | — |
| CM-12 | Filtered output as white | S38 | Verified | — | — |
| CM-13 | Short-form covariance update | S37 | Verified | — | — |
| CM-14 | Low Q → divergence **and** gain → 0 → wrong state | S54 pp.142–143 | Partly supported | p.143 "typically, but not always" linked to low noise ✓; gain→0 is a separate "not guaranteed" indicator; causal chain not stated | label inference |
| CM-15 | Receive-time stamps | S61 §0 | Verified | — | — |
| CM-16 | Narrow gate | S61 §3 | Verified | — | — |

### Disagreements

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| DIS-1 | 1e-6 docs vs 1e-9 code | S16; S24 | Verified | — | — |
| DIS-2 | UKF cost | S15; S03 p.6 | Verified | — | — |
| DIS-3 | publish_filtered_gps: docs **and** YAML false vs code true | S19; S22; S26 l.99 | Partly supported | S22 "Defaults to false" ✓; S19 section gives no default | cite S22 only |
| DIS-4 | Comment "large values" vs 1e-9 | S23 l.86–91; S20 | Verified | — | — |
| DIS-5 | Wheel pose fusion docs vs Clearpath "(level C vs level B)" | S17; S34 | Partly supported | content ✓; S17 = B, S34 = C, so order reversed | "(level B vs level C)" |
| DIS-6 | IMU yaw example vs Nav2 demo | S21; S33 | Verified | — | — |
| DIS-7 | Gate value choice | S45; S38; S53 | Verified | — | — |
| DIS-8 | "sufficient for most mobile robotics applications" vs Rovitis | S39 p.17; S47 | Verified | p.17 quote | — |
| DIS-9 | 12 vs 15; "gained acceleration states after the paper" | S07; S39 | Partly supported | counts ✓; the timing statement is an unlabelled inference | label |
| DIS-10 | odom needed? | S09; S59; S60 | Verified | — | — |
| DIS-11 | base_link position | S09; S59 | Verified | — | — |
| DIS-12 | Low-rate updates | S07 p.5; S61 | Verified | — | — |
| DIS-13 | Gate width | S61; S38; S20 | Verified | — | — |

### Open questions (cited items)

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| OQ-1 | S05 unavailable; whiteness covered by S54/S55 | S05, S54, S55 | Verified | — | — |
| OQ-2 | Mehra/ALS only via S55 | S11; S55 | Verified | — | — |
| OQ-3 | Autoware frames covered | S59; S60 | Verified | — | — |
| OQ-4 | two_d_mode on slopes only qualitative | S61 | Verified | — | — |
| OQ-5 | Smooth update, integrity covered | S61; S57 | Verified | — | — |
| OQ-6 | Principle + CI covered; CI original not downloaded | S38; S56 | Verified | — | — |
| OQ-7 | Fitzgerald cited in S54 | S54 | Verified | ref [7] "Divergence of the Kalman Filter"; index "Fitzgerald, R. J., 133, 163" | — |

## Uncited factual statements

- Open questions bullet 7: "(it only rejects NO_FIX)" — no citation; true per L4-S26 lines 619–622 (also rejects NaN lat/lon/alt). Add citation.
- Foundational references "Why it is foundational" column is uncited; the priority/usage claims for S02, S04 and S13 are flagged above.
- §6 F10.16: "no topic publishes the Mahalanobis distance" is inferred from the issue thread, not from a code search; acceptable but could cite S53 explicitly as the basis.

## Corrections applied (2026-09-28)

Editor pass applying this claim review and `SOURCE_AUDIT.md`. Each source was re-opened at the cited location before rewriting.

### Claims

| # | Change |
|---|---|
| SUM-3 | "optimal only when" → "the best filter of any conceivable form" *when* linear/white/Gaussian (Maybeck p. 10 wording). |
| SUM-4 | Warning now cited to L4-S25 lines 2501–2509 (WARN diagnostic); the L4-S24 `FB_DEBUG` citation dropped from this bullet. |
| SUM-6 | p. 101 quoted as "a filter is optimal if"; "if and only if" now cited to Theorem 6.1, PDF p. 133. |
| FR-S02 | Not supported → "Early description of the unscented filter; it cites an earlier 1995 ACC paper". |
| FR-S04 | "First" removed. |
| FR-S13 | "Widely used in robotics" removed; replaced with what the paper does (compares methods, derives extrapolation with optimal gain). |
| F1.17 | The two navsat fallbacks described separately (lines 518–559: assume origin; 566–603: "Will not remove offset", pose left at identity). |
| F1.22 (and DIS-10) | L4-S60 location changed to "Output", "Kinematics Fusion Filter" and "TF tree". |
| F2.18, F11.9, KN-18 | "indoor" → "route through a commercial environment". |
| F4.6, F4.7, F4.8, RP-5 | PDF p. 18 → PDF p. 17. |
| F4.20 | Unsourced parenthetical "(general principle, applies to any device…)" removed (§12 already states the general KF principle with L4-S35). |
| F4.21 | Split: text list (x and yaw velocity) cited to PDF p. 2; Table I (odometry ẋ, ẏ, ż, yaw rate; GPS x, y, z) to PDF p. 3; results to Table II. |
| F6.13 | First quote relocated to "Detecting and Rejecting Bad Measurement"; second stays in "Gating and Data Association Strategies". |
| F7.9 | Added reset cases: `set_datum` service (line 365) and runtime `magnetic_declination_radians` change (line 921). |
| F7.20 | "copied unchanged into the output" → copied regardless of fix status, then rotated into the world frame (lines 803–820 added). |
| F7.22 | "differ only in default position error" → defaults differ, unless GST sentences supply receiver error estimates. |
| F9.9 | Reworded: "biased and inconsistent" attributed to the paper's own polar example; Lerro radar case stated as "can become inconsistent" below 1° bearing σ. |
| F10.7 | Added the second reason (IMU 2 stopped reporting halfway). Page kept at PDF p. 5: re-check with `pdftotext -f 5 -l 5` shows the interference paragraph on p. 5 (p. 4 contains only the figure caption), so the reviewer's p. 4 was not applied. |
| F10.16 | "left open pending a contributed fix" → issue stayed closed; maintainer would reopen "if someone would be willing to PR the fix". |
| RP-4 | Added the condition from L4-S16 item 1 note: fuse both orientations if covariances are accurate; best source only if one under-reports. |
| CM-2 | "silently" removed; WARN diagnostic cited (L4-S25 lines 2501–2509). |
| CM-14 | Split into two sourced statements (low Q "typically, but not always" linked to divergence; gain → 0 as a separate sign); causal chain removed. |
| DIS-3 | Default "false" cited to L4-S22 only; L4-S19 noted as giving no default. |
| DIS-5 | "(level C vs level B)" → "(level B vs level C)". |
| DIS-9 | Unsourced "gained acceleration states after the paper" removed; now states 15 variables including linear accelerations [L4-S39, p. 17]. |
| Open questions | "(it only rejects NO_FIX)" now cited: NO_FIX or NaN fixes [L4-S26, lines 619–622]. |
| §2 fuse bullet | Fixed-lag smoother claim now cited to L4-S39 only; L4-S40 cited for its framework description and its "work in progress … not expected to work" note. |
| L4-S55 citations | Section-name locations replaced by PDF pages: "PREVIOUS WORK" → pp. 2–3 / p. 3; "IDENTIFIABILITY OF Q AND R" → pp. 6, 9–12; "ESTIMATION OF W" → pp. 12–13 (each re-checked in the PDF). |

### Sources

| ID | Change |
|---|---|
| L4-S01 | Citation notes the file is a retyped reproduction; PDF pages ≠ printed pp. 35–45. |
| L4-S02 | DOI 10.1117/12.280797 added (confirmed via Crossref). |
| L4-S28 | Author P. Bovbel added; REP status "Draft" stated. |
| L4-S40 | Pinned to release tag 1.3.4 (commit 40e31ec is tag 1.3.4, confirmed with `git ls-remote`); WIP status stated in the citation. |
| L4-S49 | Level A → C (ESA ASTRA symposium, abstract-based selection). |
| L4-S55 | Converted-text `.md` deleted and replaced with the open-access PDF (PMC author manuscript NIHMS1724120, CC BY, 75 pp., from the PMC open-data bucket; `file`: PDF document). Same source ID. IEEE and Europe PMC render links returned bot-check pages. |
| L4-S60 | Marked as the legacy *architecture v1* design doc. |
| L4-S63, L4-S64, L4-S65 | Added to Foundational references and Sources as *not downloaded* (no open copy): Groves 2013; Julier & Uhlmann 1997 CI (ACC, doi:10.1109/ACC.1997.609105); Fitzgerald 1971 (IEEE TAC 16(6), 736–747, doi:10.1109/TAC.1971.1099836, confirmed via Crossref). Open questions now refer to these IDs. |
| SCOPE.md | REP-103 authors corrected to Foote & Purvis. |

### Structure

- §3 and §4 reordered so general principles come first (Dissanayake constraints, Q discretisation, scalar variance growth; LC/TC coupling, nonholonomic constraints, filtered-output principle), then robot_localization specifics.
- No failing sources; no claims left Partly supported or Not supported. README Status set to **Verified**.

### Final mechanical check

- `file` on all 59 files in `sources/`: every `.pdf` is "PDF document"; no text file is an HTML/error/bot-check page (the "exported SGML", "LaTeX" and "Python script" labels on `.md` files are content guesses, as noted in the audit).
- Every file in `sources/` is in the Sources table; every table file exists.
- Sources table IDs run L4-S01 – L4-S65 with no gaps or duplicates; every ID cited in the README exists in the table, and every table ID is cited.
- Counts: 311 cited bullets + 117 table rows (Foundational 18, Key numbers 23, How it is tested 11, Sources 65) = 428; 65 sources.
