# L1 — Wheel odometry: verification (claims)

| | |
|---|---|
| **Topic** | L1 — Wheel odometry (`README.md`, sources L1-S01 … L1-S42) |
| **Date** | 2026-09-27 |
| **Reviewer** | independent — claims |
| **Method** | Every cited item in Summary, Foundational references, Findings, Recommended practice, Key numbers, How it is tested, Common mistakes, Disagreements and cited Open questions was checked against the cited file and location. PDFs were read with `pdftotext -layout`; "p." means the page of the downloaded PDF file (the README's convention), confirmed page by page. Code, YAML, RST and Markdown sources were read directly (line numbers are file lines). Rules: STANDARDS.md §5. Sources themselves are not graded here (see SOURCE_AUDIT.md). README.md was not edited. |

## Summary of counts

| Section | Checked | Verified | Partly supported | Not supported |
|---|---|---|---|---|
| Summary | 6 | 1 | 5 | 0 |
| Foundational references | 10 | 10 | 0 | 0 |
| Findings §1 Dead-reckoning principles | 9 | 8 | 1 | 0 |
| Findings §2 Error sources | 12 | 12 | 0 | 0 |
| Findings §3 Error modelling | 22 | 21 | 1 | 0 |
| Findings §4 Calibration | 21 | 20 | 1 | 0 |
| Findings §5 Encoders, velocity, timing | 12 | 10 | 1 | 1 |
| Findings §6 Skid-steer / tracked | 14 | 14 | 0 | 0 |
| Findings §7 Error magnitudes | 11 | 11 | 0 | 0 |
| Findings §8 Slip detection | 16 | 16 | 0 | 0 |
| Findings §9 Testing | 13 | 12 | 1 | 0 |
| Findings §10 ROS reporting | 11 | 11 | 0 | 0 |
| Findings §11 Reference implementations | 6 | 6 | 0 | 0 |
| Findings: REV SPARK MAX | 4 | 4 | 0 | 0 |
| Recommended practice | 12 | 11 | 1 | 0 |
| Key numbers | 25 | 24 | 1 | 0 |
| How it is tested | 12 | 11 | 1 | 0 |
| Common mistakes | 16 | 16 | 0 | 0 |
| Disagreements | 5 | 5 | 0 | 0 |
| Open questions (cited) | 9 | 7 | 2 | 0 |
| **Total** | **246** | **230** | **15** | **1** |

### Items needing correction (Partly supported / Not supported)

| # | Status | Problem in one line |
|---|---|---|
| S2 | Partly | "can be calibrated out … which cannot" is not stated at L1-S02 p. 3 / L1-S03 pp. 130–131. |
| S3 | Partly | Linear/cubic variance growth holds for a **straight path** with **identical encoder noise statistics**; both conditions missing. |
| S4 | Partly | Doh's PC-method does not use arbitrary paths; it needs the same path driven forward then backward. |
| S5 | Partly | "the usual fix" is not stated in the cited locations. |
| S6 | Partly | "6×6" is not in L1-S11; "velocities rather than pose" omits robot_localization's condition (all from the same encoders) and L1-S26's advice to fuse orientation over angular velocity. |
| 1.3 | Partly | "second-order Runge–Kutta" label is not in L1-S07 (it is a code comment in L1-S30). |
| 3.4 | Partly | Kelly's result needs identical encoder noise statistics; condition missing. |
| 4.10 | Partly | "~12 m each": source says start and end points were ~12 m apart, not trajectory length. |
| 5.5 | **Not supported** | The 54 % / 92 % gain is for Merry's **skip option** compared with plain time-stamping, not for time-stamping itself. |
| 5.11 | Partly | CTRE procedure: raise the **period** until granular, then the **window** until smooth; README merges the two. |
| 9.2 | Partly | Tape measure for return errors is not at L1-S02 p. 4 or L1-S01 p. 20 (it is at L1-S01 p. 23 / L1-S03 p. 143). |
| R10 | Partly | Same condition problem as S6. |
| K10 | Partly | Same "~12 m" problem as 4.10. |
| T9 | Partly | "per metre" is not in L1-S39 (Sturm gives per frame / per second). |
| O1 | Partly | "Endo's CV-04 was tested indoors" is not stated; the source says P-tile, plywood and artificial turf under a fixed motion-capture camera. |
| O5 | Partly | List of summarising sources is incomplete (Thrun via L1-S36–S38, Martínez via L1-S19, Ward & Iagnemma model-based via L1-S41). |

Uncited factual statements: none of substance. Open questions 4 (timestamping, wrap-around, resets) and 11 (agricultural figures) are uncited "not found" statements, which is acceptable for open questions. Editorial remarks ("used here only for comparison", "the newer results are the better guide", "level A is followed") are judgements, not source claims, and are clearly worded as such.

## Detailed results

### Summary

| # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|
| S1 | Error grows without bound; heading errors turn into growing lateral errors | L1-S03 p. 130; L1-S15 §1.1 | Verified | S03 p. 130: "accumulation of orientation errors will cause large position errors which increase proportionally with the distance"; S15 p. 2 §1.1: "once incurred, orientation errors grow without bound into lateral position errors" | — |
| S2 | Systematic vs non-systematic lists; systematic "can be calibrated out", non-systematic "cannot" | L1-S02 p. 3; L1-S03 pp. 130–131 | Partly supported | Lists match (S02 p. 3; S03 pp. 130–131). Calibratability is not stated there; closest: S03 p. 131 "magnitude of non-systematic errors is unpredictable". It is stated in S15 p. 5 ("robot cannot be calibrated to compensate for non-systematic errors"), S06 p. 1 ("can be removed by calibration"), S03 p. 139. | Add [L1-S15, p. 5] / [L1-S03, p. 139] for the calibration part, or reword. |
| S3 | Heading/along-track variance linear, cross-track cubic with distance | L1-S04 §9.1.3 p. 25; L1-S06 p. 7 | Partly supported | S04 p. 25: "When the encoders have identical noises … Heading variance and alongtrack variance are linear in distance whereas crosstrack variance is cubic" (straight trajectory, §9.1). S06 p. 7 is also for straight-line motion. | Add "on a straight line, with equal noise statistics on both wheels". |
| S4 | UMBmark identifies E_d, E_b; 10–22-fold reduction; later methods use arbitrary paths (Kelly, Doh, Censi, Seegmiller) | L1-S01 pp. 22–23; L1-S04 §10.4; L1-S13 pp. 9–10; L1-S08 p. 1; L1-S14 pp. 18–19 | Partly supported | S01 p. 22 Table I: 10- to 22-fold ✓. S04 §10.4 "Calibration from Arbitrary Trajectories" ✓; S08 p. 1 "does not require … particular trajectories" ✓; S14 p. 19, 28 trajectories ✓. Doh's PC-method drives a path "forward and then backward" (S13 p. 6); only Kelly's technique is called "applicable … in any arbitrary path" (S13 p. 9). | Describe Doh as "from a path driven forward and back" instead of "arbitrary paths". |
| S5 | Skid-steer must slip to turn; pure rolling over-predicts yaw rate; "the usual fix" is enlarged effective track width, plus slip detection | L1-S19 pp. 3, 5; L1-S22 p. 7; L1-S21 §II; L1-S18 p. 3 | Partly supported | S22 p. 7: differential-drive kinematics "greatly overestimates yaw rate" ✓; S19 p. 3 ICR model ✓; S21 §II and S18 p. 3 slip indicators ✓. No cited location says this is "the usual" fix (S20 p. 1 says extended differential-drive models are "commonly deployed", but S20 is not cited here). | Say "a common fix" and add [L1-S20, p. 1], or drop "usual". |
| S6 | `odom` continuous and may drift; Odometry pose/twist frames with "6×6 covariances"; robot_localization advises fusing velocities rather than pose | L1-S09 §odom; L1-S11; L1-S27 odom0 item 1 | Partly supported | S09 lines 52–58 ✓; S11 lines 2–3 frames ✓, but the file only names `PoseWithCovariance`/`TwistWithCovariance` (no "6×6"). S27 line 54: "its velocity, heading, and position data are all generated from the same source … it's best to just use the velocities" — conditional; S26 item 1 says "If the odometry provides both orientation and angular velocity, fuse the orientation." | Drop "6×6" or cite L1-S30 controller.cpp line 436 (NUM_DIMENSIONS = 6); add "when pose, heading and velocity all come from the same encoders". |

### Foundational references (descriptions of why each is foundational)

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F1 | S01 defines E_d, E_b, UMBmark and correction procedure | L1-S01 | Verified | p. 6 eqs. 2.3–2.4; pp. 17–20 correction | — |
| F2 | S02 original benchmark; taxonomy; extended UMBmark | L1-S02 | Verified | p. 3 lists; pp. 6–7 extended UMBmark | — |
| F3 | S03 survey; odometry equations ch. 1, ch. 5, tracked vehicles | L1-S03 | Verified | p. 20 eqs. 1.2–1.7; p. 130 ch. 5; p. 28 §1.3.8 | — |
| F4 | S04 general linearised solution for systematic and random error | L1-S04 | Verified | p. 2 abstract "general solution of linearized propagation dynamics of both systematic and random errors" | — |
| F5 | S05 per-wheel noise ∝ distance, closed-form for lines/arcs/turns, experiments | L1-S05 | Verified | p. 1 abstract; pp. 6–7 eq. 4; p. 17 experiments | — |
| F6 | S06 closed-form derivation behind S05 | L1-S06 | Verified | p. 1 abstract: closed form for straight lines, arcs, turns | — |
| F7 | S07 textbook odometry update and (k_r, k_l) model | L1-S07 | Verified | pp. 7–8 eqs. 5.6–5.9 | — |
| F8 | S08 closed-form ML calibration, no special path, reviews families | L1-S08 | Verified | p. 1 abstract and §I-A | — |
| F9 | S18 encoder/gyro/current slip indicators | L1-S18 | Verified | p. 3 EI, GI, CI | — |
| F10 | S09/S11 official ROS definitions | L1-S09, L1-S11 | Verified | S09 §odom; S11 message file | — |

### Findings §1 — Dead-reckoning principles and integration methods

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 1.1 | c_m = πD_n/(nC_e); centre displacement = mean; Δθ = (ΔU_R − ΔU_L)/b | L1-S03 p. 20 eqs. 1.2–1.5 | Verified | p. 20 eqs. 1.2–1.5 | — |
| 1.2 | x_i = x_{i−1} + ΔU cos θ_i, y likewise | L1-S03 p. 20 eqs. 1.6–1.7 | Verified | p. 20 eqs. 1.7a/b | — |
| 1.3 | Textbook uses mid-point heading θ + Δθ/2 "(a second-order Runge–Kutta form)" | L1-S07 §5.2.4 p. 7 | Partly supported | p. 7 eqs. 5.2–5.3 use θ + Δθ/2 ✓; S07 does not call it Runge–Kutta. The label is in L1-S30 odometry.cpp line 139 ("Runge-Kutta 2nd order integration"). | Cite [L1-S30, odometry.cpp lines 135–143] for the RK2 label, or mark it as an inference. |
| 1.4 | diff_drive_controller exact arc integration, RK2 fallback when |Δθ| < 1e-6 | L1-S30 odometry.cpp 135–160 | Verified | lines 145–159 | — |
| 1.5 | Circular arcs give "relatively small" benefits (Larsson) | L1-S03 p. 138 | Verified | p. 138 "The benefits of this approach are relatively small." | — |
| 1.6 | Kelly: "forced dynamics"; integral vs algebraic triangulation | L1-S04 pp. 2–3 | Verified | p. 2 "Triangulation errors are algebraic … Odometry … integrals"; p. 3 "forced dynamics" | — |
| 1.7 | Error stops when motion stops; some systematic errors cancel on closed paths; some reversible by driving back | L1-S04 p. 2; p. 40 | Verified | p. 40 "error propagation also stops when motion stops … certain systematic errors cancel on closed trajectories … some are reversible"; p. 2 "driving backward over the path" | — |
| 1.8 | Heading errors grow without bound; lateral error d·sin Δθ | L1-S15 §1.1; L1-S07 §5.2.3 p. 6 | Verified | S15 p. 2; S07 p. 6 "d sin Δθ" | — |
| 1.9 | Zoë 3D model: <.25 m and 2.3° after up to 201.5 m, no gyro; 2D yaw spikes at ramps | L1-S22 p. 6 | Verified | p. 6 quote exact; yaw spikes: p. 7 Fig. 5 caption (and p. 6 text) | — |

### Findings §2 — Error sources and taxonomy

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 2.1 | Systematic sources list | L1-S02 p. 3; L1-S03 pp. 130–131 | Verified | S02 p. 3 items a–f | — |
| 2.2 | Non-systematic sources list | L1-S02 p. 3; L1-S03 p. 131 | Verified | S02 p. 3 items a–c | — |
| 2.3 | Smooth indoor: systematic dominate; rough: non-systematic dominate | L1-S03 p. 131 | Verified | p. 131 "non-systematic errors are dominant" | — |
| 2.4 | E_d, E_b "notorious"; wheelbase uncertainty ~1% | L1-S02 p. 3; L1-S01 p. 6 | Verified | S02 p. 3; S01 p. 6 eqs. 2.3–2.4 | — |
| 2.5 | E_s measurable with tape to 0.3–0.5% of full scale; corrected first | L1-S02 p. 4 | Verified | p. 4 "accuracy of 0.3-0.5% of full scale" | — |
| 2.6 | E_d curves legs, E_b mis-turns; one-direction test can cancel | L1-S02 pp. 4–5; L1-S03 pp. 133–134 | Verified | S02 p. 5; S03 pp. 133–134 | — |
| 2.7 | Siegwart range/turn/drift; turn+drift "far outweigh" range | L1-S07 §5.2.3 p. 6 | Verified | p. 6 | — |
| 2.8 | 10 mm bump ≈0.6°; larger bumps 0.5°–0.8° | L1-S02 p. 6; L1-S15 §2 p. 4 | Verified | S02 p. 6; S15 p. 4 "on the order of 0.5° - 0.8° for the larger bumps" | — |
| 2.9 | Small wheelbase; knife-edge wheels; loaded castors slip on reversing | L1-S03 pp. 137–138 | Verified | p. 137 wheelbase; p. 138 castors, knife-edge | — |
| 2.10 | Boyden & Velinsky: kinematic model accurate only up to 0.3 m/s in tight turn; limit turn speed/accel | L1-S03 p. 138 | Verified | p. 138 | — |
| 2.11 | Over-constrained vehicles slip on wheel-speed mismatch | L1-S16 p. 6; L1-S18 p. 2 | Verified | S16 p. 6 "any momentary mismatch between wheel velocities will force the wheels to skid or slip"; definition p. 2 | — |
| 2.12 | Slip/bumps over-count; skid gives fewer pulses | L1-S16 pp. 4–6; L1-S18 p. 1 | Verified | S16 p. 4 "encoders 'over-count'"; p. 5 "skidding wheels always produce fewer encoder pulses"; S18 p. 1 | — |

### Findings §3 — Error modelling and covariance propagation

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 3.1 | σ_L² = k_L²d_L, σ_R² = k_R²d_R, zero-mean white, uncorrelated wheels | L1-S05 pp. 6–7 eq. 4; L1-S06 p. 3 eq. 1 | Verified | S05 p. 6 "zero mean, and white"; "uncorrelated"; p. 7 eq. 4; S06 p. 3 eq. 1 | — |
| 3.2 | Analytic integration over segments; line, arc, turn about axle; short arcs for other paths | L1-S06 p. 1; L1-S05 p. 1 | Verified | both abstracts | — |
| 3.3 | Straight line: σ_θ, σ_along ∝ √d; σ_cross ∝ d^1.5 | L1-S06 p. 7 | Verified | p. 7 "square root of distance … power 1.5" | — |
| 3.4 | Kelly: same pattern for differential-heading odometry on straight path | L1-S04 §9.1.3 p. 25 | Partly supported | p. 25: pattern holds "When the encoders have identical noises" (otherwise a σ_vω term remains) | Add "when both encoders have identical noise statistics". |
| 3.5 | Biases time-dependent, scale errors motion-dependent; scale error cancels on closed paths; random grows | L1-S04 p. 40 | Verified | p. 40 §13.3 | — |
| 3.6 | Lateral wheel slip acts like angular-velocity scale error (like gyro scale error) | L1-S04 p. 30 | Verified | p. 30 "lateral wheel slip acts like a scale error in angular velocity … apply to lateral wheel slip and gyro scale errors" | — |
| 3.7 | "good gyros will tend to outperform differential heading odometry" | L1-S04 p. 40 | Verified | p. 40 | — |
| 3.8 | Linearised within 1 cm or 3% on 210 s trajectory; Monte Carlo matched | L1-S04 p. 39 | Verified | p. 39 "never exceeds 1 cm or 3%"; "total time is 210 s"; Fig. 15 | — |
| 3.9 | Σ_Δ = diag(k_r|Δs_r|, k_l|Δs_l|); propagation law; k "experimentally established" | L1-S07 §5.2.4 p. 8 | Verified | p. 8 eqs. 5.8–5.9 | — |
| 3.10 | Perpendicular uncertainty grows faster; ellipse axis not perpendicular on arcs | L1-S07 p. 10 Figs. 5.4–5.5 | Verified | p. 10 captions | — |
| 3.11 | k = 0.05 m^1/2 over ~10 m "marginally good"; beyond, second-order model | L1-S05 p. 15 | Verified | p. 15 | — |
| 3.12 | k_L = 0.00040, k_R = 0.00058 m^1/2 on parquet; floor-dependent | L1-S05 p. 17 | Verified | p. 17 | — |
| 3.13 | Could not fit all six covariance elements; cross-track sensitive; bumps not modelled | L1-S05 pp. 17, 19 | Verified | p. 17; p. 19 "cannot account for … hitting a bump" | — |
| 3.14 | Error-ellipse methods include only systematic behaviour | L1-S03 p. 131 | Verified | p. 131 | — |
| 3.15 | Martinelli & Siegwart: AKF for systematic + Observable Filter for non-systematic; simulation | L1-S12 pp. 1–5 | Verified | p. 1 abstract and intro; p. 4–5 simulations | — |
| 3.16 | AMCL implements sample_motion_odometry (Prob. Rob. p. 136); rot1/trans/rot2 | L1-S37 lines 50–70 | Verified | line 50 comment; lines 57–70 | — |
| 3.17 | Noise variances α1·rot² + α2·trans²; α3·trans² + α4·(rot1² + rot2²) | L1-S37 lines 86–103 | Verified | lines 86–103 (sqrt passed as σ) | — |
| 3.18 | Nav2 docs name α1–α4; α5 omni only; defaults 0.2 | L1-S38 §Parameters | Verified | lines 9–35 | — |
| 3.19 | Backward motion folded to 0 or π; rot1 = 0 when translation < 0.01 | L1-S37 lines 55–80 | Verified | lines 57–61, 72–80 (quote exact) | — |
| 3.20 | Banana-shaped, "clearly not Gaussian"; Gaussian filters "inconsistent" | L1-S36 p. 1 | Verified | p. 1 abstract, Fig. 1 | — |
| 3.21 | Bailey et al.: heading σ > "one or two degrees" → EKF-SLAM maps failed | L1-S36 p. 1 | Verified | p. 1 | — |
| 3.22 | Gaussian in exponential coordinates; equal at small noise, better as uncertainty grows; closed-form for lines and arcs | L1-S36 pp. 2, 4; abstract, §VI | Verified | p. 4 "for small diffusion values … approximately the same … as the uncertainty is increased … performs better"; §VI p. 4 | — |

### Findings §4 — Odometry calibration methods

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 4.1 | UMBmark 4×4 m, 5 cw + 5 ccw, stop at corners, turn on spot, slowly | L1-S02 p. 7 | Verified | p. 7 §3.4 steps 2, 5, 6 | — |
| 4.2 | Cluster centres; E_max,syst = max(r_cw, r_ccw) | L1-S02 pp. 5–6 eqs. 2–4 | Verified | pp. 5–6 | — |
| 4.3 | α, β; R = (L/2)/sin(β/2); E_d = (R+b/2)/(R−b/2); b_actual = 90/(90−α)·b_nom | L1-S01 pp. 17–19 eqs. 4.17–4.27 | Verified | pp. 17–19 | — |
| 4.4 | c_L = 2/(E_d+1), c_R = 2/(1/E_d+1), average diameter unchanged | L1-S01 p. 19 eqs. 4.28–4.33 | Verified | p. 19 (eq. 4.33 on p. 20) | — |
| 4.5 | 8 experiments: 232–423 → 12–35 mm, 10–22-fold; one needed second pass 66 → 20 | L1-S01 p. 22 Table I | Verified | p. 22 Table I | — |
| 4.6 | Second pass helps only if E_max,syst > 3·SEM; SEM 11.2 mm | L1-S01 p. 22 | Verified | p. 22 | — |
| 4.7 | Survey: "10- to 20-fold", "about two hours" | L1-S03 pp. 142–143 | Verified | p. 142; p. 143 | — |
| 4.8 | Chong–Kleeman UMBmark: 135 → 33 mm ("4 folds"); wheelbase 0.37100 → 0.36898; residual not removed | L1-S05 p. 16 Table 8 | Verified | p. 16 | — |
| 4.9 | Kelly: endpoint residuals as linear equations, pseudo-inverse | L1-S04 §10.4 pp. 33–34 | Verified | p. 33 "linearized error integrals … constraint equations"; p. 34 "left pseudo-inverse" | — |
| 4.10 | IPEM: 28 trajectories "(~12 m each)"; ~75% worst case; low-rate poses, less timing-sensitive | L1-S14 pp. 18–19; pp. 3–4 | Partly supported | p. 19: "28 different trajectories … from the same start point to nearly the same endpoint, approximately 12 m apart"; "reduced by about 75% in the worst case"; p. 3 "sensitivity to accurate timing is also much reduced". Trajectory length is not given. | Replace "(~12 m each)" by "(start and end ~12 m apart)". |
| 4.11 | PC-method forward/backward; 5.49 / 2.29 / 1.42 / 0.82 m after ~1 km | L1-S13 p. 6; p. 9 Table 3 | Verified | p. 6; p. 9 Table 3; ~1 km on p. 10 | — |
| 4.12 | Augmented KF approaches (Larsen, Martinelli) | L1-S08 p. 1; L1-S12 p. 1 | Verified | S08 p. 1; S12 p. 1 "as it operates" | — |
| 4.13 | Antonelli "exactly linear", LLS with camera; Censi closed-form, no special trajectory, no external sensor | L1-S08 p. 1 | Verified | p. 1 | — |
| 4.14 | UMBmark/EKF assume known nominal values; EKF outliers from wheel slip | L1-S08 p. 2 | Verified | p. 2 | — |
| 4.15 | Systematic errors change slowly with wear/load; periodic recalibration | L1-S03 p. 139; L1-S01 p. 24 | Verified | S03 p. 139; S01 p. 24 | — |
| 4.16 | Kümmerle: radii, separation, laser pose in hyper-graph SLAM, online, rough initial guess, no prior map | L1-S42 p. 1; pp. 3, 5–6 | Verified | p. 1 abstract; p. 3 hyper-graph | — |
| 4.17 | "when the robot carries a load … carpet to concrete" | L1-S42 p. 3 | Verified | p. 3 | — |
| 4.18 | PowerBot ~40 kg on left; radii values; "severe drift" | L1-S42 pp. 11–12 Fig. 5 | Verified | p. 11; p. 12 Fig. 5 caption | — |
| 4.19 | Symmetric skid-steer model from pure rotation (ICR) + straight run (α) | L1-S19 pp. 3–4 | Verified | p. 3 eq. 11; p. 4 eq. 12 | — |
| 4.20 | 5-parameter asymmetric ICR by GA vs RTK-GPS segments; quote | L1-S19 p. 4 | Verified | p. 4 quote exact; RTK on p. 5 | Optional: add p. 5 for RTK. |
| 4.21 | Nav2 BT: 2 m CCW square ×3 at 0.2 m/s, "primitive experiment" | L1-S29 | Verified | lines 4–6 | — |

### Findings §5 — Encoder measurement, velocity estimation and timing

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 5.1 | Up to half-count quantisation; differentiation amplifies | L1-S24 pp. 1–2 | Verified | p. 1 abstract; p. 2 "maximally half an encoder count" | — |
| 5.2 | Fixed-time: absolute error speed-independent; % intolerable at low speed | L1-S25 pp. 2–3 | Verified | p. 3 | — |
| 5.3 | Longer windows / LP / MA filters add lag | L1-S25 p. 3 | Verified | p. 3 "additional lag time introduced by the filter degrade the performance of the speed control loop" | — |
| 5.4 | Period better at low speed, frequency at high; mixed mode | L1-S25 pp. 3–4 | Verified | p. 4 | — |
| 5.5 | "Time-stamping each encoder event … improved velocity 54% and acceleration 92%" | L1-S24 p. 1 | **Not supported** | p. 1 abstract: "we propose a method to extend the observation interval … using a skip operation. Experiments … show that the velocity estimation is improved by 54% and the acceleration estimation by 92%"; p. 6: "Compared to time stamping without skip". | Reword: "Adding a 'skip' option to event time-stamping improved velocity estimates by 54% and acceleration by 92% compared with time-stamping without skip". |
| 5.6 | Encoder imperfections add errors | L1-S24 p. 3 | Verified | p. 3 | — |
| 5.7 | position_feedback default true; velocity × radius × period; open_loop | L1-S30 controller.cpp 147–184; yaml | Verified | lines 147–184; yaml lines 84–93 | — |
| 5.8 | Position path: pose from differences; rolling mean window 10 | L1-S30 odometry.cpp 48–99; yaml | Verified | lines 48–99; yaml line 111 | — |
| 5.9 | Odometry msg and TF stamped with `time` | L1-S30 controller.cpp 211, 226 | Verified | lines 211, 226 | — |
| 5.10 | Master adds update_from_pos/update_from_vel with explicit dt | L1-S31 lines 80–149, 236–256 | Verified | lines 80–97, 130–150, 236–259 | — |
| 5.11 | Talon SRX defaults 100 ms + 64 samples (1 ms); "start both at 1 and increase until smooth 'but still responsive enough'" | L1-S35 §"Velocity Measurement Filter" | Partly supported | lines 398–401 defaults ✓. Procedure (lines 427–432): set both to 1, "Increase the sampling period until the measured velocity is sufficiently granular", then "Increase the rolling average window until … smooth, but still responsive enough". | Say the period is raised until granular, then the window until smooth. |
| 5.12 | Kelly optimal update rate; Seegmiller delay + first-order lag | L1-S04 p. 2; L1-S14 p. 20 | Verified | S04 p. 2; S14 p. 20 | — |

### Findings §6 — Skid-steer and tracked vehicle odometry

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 6.1 | Skid-steer relies on slip; poor dead reckoning; tracked "virtually impossible" | L1-S03 pp. 28, 139 | Verified | p. 28; p. 139 | — |
| 6.2 | ICR per tread; χ = 1 with no slip | L1-S19 p. 3 | Verified | p. 3 eq. 9 | — |
| 6.3 | P3-AT L = 0.4 m; χ 0.69–0.76 asphalt, 0.71–0.75 concrete; α 0.90–0.95 | L1-S19 p. 5 Tables I–II | Verified | Tables I–II p. 5 (L = 0.4 m is on p. 4) | Optional: add p. 4 for L. |
| 6.4 | Asphalt lower χ "due to greater friction"; higher pressure α closer to 1 | L1-S19 p. 5 | Verified | p. 5 | — |
| 6.5 | Factory ICR 0.3 m vs non-slip 0.2 m | L1-S19 p. 5 | Verified | p. 5 | — |
| 6.6 | Δy MSE 0.00468 → 0.00011 | L1-S19 p. 6 Table III | Verified | p. 6 | — |
| 6.7 | "greatly overestimates yaw rate" | L1-S22 p. 7 | Verified | p. 7 | — |
| 6.8 | Baril: models compared on 590 kg robot, >2 km, snow/concrete | L1-S20 p. 1 | Verified | p. 1 abstract | — |
| 6.9 | Symmetric extended model best angular, recommended; trained "vastly better" | L1-S20 pp. 3, 6–7 | Verified | p. 6; p. 7 "the best model in our case is the extended differential drive symmetric"; Fig. 5 caption | — |
| 6.10 | Rotates more on snow; ideal DD better on snow | L1-S20 pp. 6–7 | Verified | p. 6; p. 7 Fig. 7 caption | — |
| 6.11 | Error peak at ~2:1 command ratio; nonlinearity; translational error independent | L1-S20 p. 7 | Verified | p. 7 | — |
| 6.12 | Slip ratios (1 − a); gyro gives one more relation (SCOG) | L1-S21 §II-B–D | Verified | p. 2 Fig. 1, eqs.; §II-D | — |
| 6.13 | n = 0.4811 / 0.6213 / 0.5094; ≈0.5 | L1-S21 §III-B p. 3 | Verified | p. 3 | — |
| 6.14 | Pentzer EKF ICR tracking, 118 kg robot (via S20) | L1-S20 p. 2 | Verified | p. 2 | — |

### Findings §7 — Typical odometry error magnitudes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 7.1 | Uncalibrated LabMate 310 mm, 0.2 m/s | L1-S02 p. 8 | Verified | p. 8 | — |
| 7.2 | Calibrated 12–35 mm; 33 mm low-cost | L1-S01 p. 22; L1-S05 pp. 16–17 | Verified | S01 Table I; S05 Table 8–9 | — |
| 7.3 | Hongo 350 kg, <200 mm over 50 m | L1-S03 p. 138 | Verified | p. 138 (on a "well-paved" road) | — |
| 7.4 | Nomad Scout 25.7 m → 4.4 m after 719.1 m | L1-S13 p. 9 | Verified | p. 9 | — |
| 7.5 | LabMate odometry limited to ~10 m | L1-S03 pp. 137–138 | Verified | pp. 137–138 | — |
| 7.6 | Pioneer AT sand track 12 m: 56 mm (PID), 71 mm (CCC) | L1-S16 pp. 9, 14 | Verified | p. 9; p. 14 | — |
| 7.7 | FLEXnav <1% typical, "may become huge"; iComp "well under 1%", up to 10× | L1-S17 pp. 1, 12 | Verified | p. 1; p. 12 | — |
| 7.8 | LandTamer 10 m: 1.717/1.465/1.774 m; 0.291/0.277/0.493 rad; 64–96% reduction | L1-S22 p. 8 Tables III–IV | Verified | p. 8; p. 7 "up to 2.5 m/s" | — |
| 7.9 | CV-04 path: (22, 188, 2.02) / (13, −6, 3.15) / (2, −3, 3.15) | L1-S21 §V-C p. 5 | Verified | p. 5; P-tile floor p. 5 §V-B | — |
| 7.10 | MER 10%/100 m goal; 19 m drive, 1.6 m under, ~5 m; slips to 125% | L1-S23 pp. 2, 10, 11 | Verified | p. 2; p. 11; p. 10 | — |
| 7.11 | All-wheel slippage; earlier method needed one wheel gripping | L1-S17 p. 1; L1-S18 p. 2 | Verified | S17 p. 1; S18 p. 2 | — |

### Findings §8 — Slip detection and compensation

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 8.1 | Gyrodometry switch on threshold; ~18× vs odometry, ~8× vs gyro | L1-S15 §3–4 pp. 5–6 | Verified | p. 5 pseudo-code; p. 6 | — |
| 8.2 | Bump errors act "a fraction of a second for each encounter" | L1-S15 abstract | Verified | p. 1 | — |
| 8.3 | Fewest Pulses + 0.5-count correction | L1-S16 pp. 4–5 | Verified | pp. 4–5 | — |
| 8.4 | CCC reduces mismatch slip; "not substantial" on sand | L1-S16 pp. 6, 12 | Verified | p. 6; p. 12 | — |
| 8.5 | Expert rule D_L = D_R − ω_z·T·B | L1-S16 pp. 8–9 eqs. 3–4 | Verified | p. 9 | — |
| 8.6 | EI, GI, CI indicators; current ∝ torque | L1-S18 p. 3 | Verified | p. 3 | — |
| 8.7 | 31/56/91/94% (1% FP); 25/38/18/61% (12% FP) | L1-S18 pp. 8, 9 | Verified | p. 8; p. 9 | — |
| 8.8 | iComp linear slip–current; three tuning methods; longitudinal only | L1-S17 pp. 1–2 | Verified | p. 1 abstract; p. 2 | — |
| 8.9 | SCOG: encoders + gyro + one exponent | L1-S21 abstract §II-D | Verified | p. 1 abstract; §II-D | — |
| 8.10 | MER VO Slip Checks; wheel+IMU fail on slopes/sand | L1-S23 pp. 2, 10–11 | Verified | p. 2; p. 11 "insufficient progress … Slip Check" | — |
| 8.11 | Ward: SVM, 0.5 s windows, N = 50 at 100 Hz, four features | L1-S41 pp. 2–3 | Verified | pp. 2–3; Fig. 3 caption p. 4 | — |
| 8.12 | Feature-four rationale | L1-S41 p. 3 | Verified | p. 3 | — |
| 8.13 | 94.7% total, 98.1% normal, 75% immobilised; 92.0% IMU-only | L1-S41 p. 3 | Verified | p. 3 | — |
| 8.14 | Accelerometer-integrated speed diverges (random walk) | L1-S41 p. 4 | Verified | p. 4 | — |
| 8.15 | Fusion with dynamic-model detector removed false positives | L1-S41 pp. 5–6 | Verified | p. 5 "Both fusion techniques eliminated these false positives" | — |
| 8.16 | MER slip 98.9–99.5%, sols 463–483 | L1-S23 p. 15 | Verified | p. 15 | — |

### Findings §9 — Testing and evaluating odometry

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 9.1 | One-direction square and figure-8 hide errors | L1-S02 pp. 4–5; L1-S03 pp. 133–134 | Verified | S02 p. 5; S03 p. 134 | — |
| 9.2 | Return errors via L-shaped wall and tape measure (or sonar ±2 mm, ±0.4°) | L1-S02 p. 4; L1-S01 p. 20 | Partly supported | S02 p. 4: walls as reference ✓; S01 p. 20: L-shaped corner, sonar "within ±2 millimeters … ±0.4°" ✓. Tape measure for return errors is not at either location (S02 p. 4 mentions a tape measure only for E_s). | Add [L1-S01, p. 23] or [L1-S03, p. 143] for the tape measure. |
| 9.3 | Cluster spread = non-systematic, limited value (floor-dependent) | L1-S02 p. 6 | Verified | p. 6 | — |
| 9.4 | Extended UMBmark: ~10 mm cable ×10, inside wheel, first leg; orientation error | L1-S02 pp. 6–7 | Verified | pp. 6–7 | — |
| 9.5 | 60 runs 10 m out-and-back with sonar walls | L1-S05 p. 17 | Verified | p. 17 | — |
| 9.6 | Doh 10 paths ~100 m; t-tests | L1-S13 pp. 9–10 | Verified | p. 10 | — |
| 9.7 | Ground truth: RTK <1 cm; total station; mocap 9 mm; lidar ICP | L1-S19 p. 5; L1-S22 p. 7; L1-S21 Table I; L1-S20 p. 4 | Verified | each location | — |
| 9.8 | Baril ε_t (%), ε_θ (deg/m); training horizon effect | L1-S20 pp. 5–6 | Verified | p. 5 | — |
| 9.9 | RPE = drift; RMS; Δ = 30 at 30 Hz = per second | L1-S39 pp. 6–7 | Verified | p. 6; p. 7 | — |
| 9.10 | Start-to-end "a common (but poor) choice"; KITTI same point | L1-S39 p. 7; L1-S40 p. 5 | Verified | S39 p. 7; S40 p. 5 | — |
| 9.11 | ATE: Horn alignment, RMS; global consistency; "strongly correlated" | L1-S39 p. 7 | Verified | p. 7 | — |
| 9.12 | KITTI %, deg/m over sub-sequences; best 2.2%, 0.016 deg/m | L1-S40 pp. 5–6 | Verified | p. 6 | — |
| 9.13 | Closed/symmetric paths "may or may not be good choices" | L1-S04 p. 40 | Verified | p. 40 | — |

### Findings §10 — Reporting odometry in ROS

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 10.1 | odom drift/continuity/short-term reference | L1-S09 §odom | Verified | lines 52–65 | — |
| 10.2 | Frame authorities | L1-S09 | Verified | lines 207–214 | — |
| 10.3 | Metres, radians; right-handed; x fwd, y left, z up | L1-S10 | Verified | lines 57, 76, 97, 107–109 | — |
| 10.4 | Pose in header.frame_id, twist in child_frame_id, both with covariance | L1-S11 | Verified | lines 2–3, 12, 15 | — |
| 10.5 | robot_localization frame transformations | L1-S26 §Coordinate Frames | Verified | "Coordinate Frames" bullet on nav_msgs/Odometry | — |
| 10.6 | Position+velocity → fuse velocity; same encoders → "best to just use the velocities" | L1-S26 §Odometry 1; L1-S27 item 1 | Verified | S26 item 1 (position/linear velocity); S27 line 54 | — |
| 10.7 | Zero lateral velocity valid measurement | L1-S27 item 2 | Verified | line 70 | — |
| 10.8 | Covariances "matter"; 1e3 inflation detrimental; use config vector | L1-S26 §Odometry 3, §Common errors | Verified | item 3; Common errors | — |
| 10.9 | Zero variance → epsilon 1e−6 | L1-S26 §Common errors | Verified | "will add a small epsilon value (1e-6)" | — |
| 10.10 | Disable driver TF if EKF publishes odom→base_link | L1-S26 §Odometry 5 | Verified | item 5 | — |
| 10.11 | Nav2 needs TF + Odometry; IMU drift over time, encoders over distance | L1-S28 | Verified | lines 15, 21 | — |

### Findings §11 — Reference implementations

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 11.1 | Defaults: position_feedback, open_loop, enable_odom_tf, 50 Hz, window 10, zero covariances with suggested values | L1-S30 parameters.yaml | Verified | yaml lines 74–123 | — |
| 11.2 | Fixed covariance diagonals, not distance-dependent | L1-S30 controller.cpp 437–443 | Verified | lines 436–443 (set once at configure) | — |
| 11.3 | Multipliers used for commands and odometry | L1-S30 controller.cpp 143–145, 298–302 | Verified | lines 143–145, 257–260, 298–302 | — |
| 11.4 | Several wheels per side averaged | L1-S30 controller.cpp 153–173 | Verified | lines 153–173 | — |
| 11.5 | Clearpath: 1.875 (0.555 m), 1.5 (0.37559 m); covariances; 50 Hz; enable_odom_tf false | L1-S32 | Verified | a200 and j100 yaml | — |
| 11.6 | Nav2 guide: linear/angular formulas; wheel vel from joint position changes; recommends diff_drive_controller | L1-S28 | Verified | lines 50–56 | — |

### Findings — REV SPARK MAX / NEO

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 12.1 | Quadrature: period 1–100 ms (100 default), depth 1–64 (64) | L1-S33 | Verified | lines 118–141 | — |
| 12.2 | Hall: 8–64 ms (32), depth 1/2/4/8 (8) | L1-S33 | Verified | lines 146–183 | — |
| 12.3 | Rotations, RPM × conversion factor | L1-S33 | Verified | lines 94–113 | — |
| 12.4 | (8 − 1)/2 × 32 = 112 ms; REV wants to reduce | L1-S34 | Verified | file body | — |

### Recommended practice

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| R1 | Correct scale error with tape first | L1-S02 p. 4 | Verified | p. 4 | — |
| R2 | Bidirectional UMBmark, 5 each way, slowly, stop to turn | L1-S02 p. 7; L1-S01 pp. 17–19 | Verified | p. 7 | — |
| R3 | Repeat if E_max,syst > 3·SEM | L1-S01 p. 22 | Verified | p. 22 | — |
| R4 | Skid-steer: ICR from rotation + straight run, per terrain/speed; or full ICR fit | L1-S19 pp. 3–5; L1-S20 pp. 6–7 | Verified | S19 pp. 3–4 | — |
| R5 | Per-wheel variance ∝ distance; identify k on surface | L1-S05 pp. 7, 17; L1-S07 p. 8 | Verified | as cited | — |
| R6 | Re-calibrate as loads/tyres change | L1-S01 p. 24; L1-S03 p. 139 | Verified | as cited | — |
| R7 | Keep turn speeds/accelerations low | L1-S03 p. 138 | Verified | p. 138 | — |
| R8 | Gyro for short heading disturbances | L1-S15 §3; L1-S16 pp. 8–9 | Verified | as cited | — |
| R9 | Continuous odom; REP-105/103; non-zero covariances | L1-S09; L1-S26 §Odometry | Verified | S26 items 3–4, Common errors | — |
| R10 | Fuse wheel-odometry velocities; one odom→base_link broadcaster | L1-S26 items 1, 5; L1-S27 item 1 | Partly supported | Item 5 ✓. S27 advice is conditional ("velocity, heading, and position data are all generated from the same source"); S26 item 1 says fuse orientation when orientation and angular velocity are both provided. | Add "when pose, heading and velocity all come from the same encoders". |
| R11 | Re-estimate radii on load/surface change or online | L1-S42 pp. 3, 11–12 | Verified | as cited | — |
| R12 | Report relative errors over many intervals | L1-S39 pp. 6–7; L1-S40 p. 5 | Verified | as cited | — |

### Key numbers

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| K1 | Wheelbase uncertainty ~1% | L1-S02 p. 3 | Verified | p. 3 | — |
| K2 | E_s 0.3–0.5% | L1-S02 p. 4 | Verified | p. 4 | — |
| K3 | 10–22-fold; 232–423 → 12–35 mm; 0.2 m/s | L1-S01 p. 22 | Verified | p. 22; 0.2 m/s p. 20 | — |
| K4 | About 2 h | L1-S03 p. 143 | Verified | p. 143 | — |
| K5 | 10 mm bump ≈0.6° | L1-S02 p. 6 | Verified | p. 6 | — |
| K6 | √d and d^1.5 growth | L1-S06 p. 7 | Verified | p. 7 (straight line, as the row says) | — |
| K7 | k_L, k_R values | L1-S05 p. 17 | Verified | p. 17 | — |
| K8 | ≤1 cm or 3%, 210 s | L1-S04 p. 39 | Verified | p. 39 | — |
| K9 | 0.82 / 2.29 / 1.42 m, raw 5.49 m | L1-S13 p. 9 | Verified | p. 9 | — |
| K10 | IPEM −75%; "28 trajectories, ~12 m" | L1-S14 p. 19 | Partly supported | p. 19: endpoints "approximately 12 m apart" | Write "start–end ~12 m apart". |
| K11 | χ 0.69–0.76 | L1-S19 p. 5 | Verified | p. 5 | — |
| K12 | n 0.48–0.62 (≈0.5) | L1-S21 p. 3 | Verified | p. 3 | — |
| K13 | 1.47–1.77 m, 0.28–0.49 rad | L1-S22 p. 8 | Verified | p. 8 | — |
| K14 | 64–96% | L1-S22 p. 8 | Verified | p. 8 Table IV (10 m) | — |
| K15 | <0.25 m, 2.3°, 201.5 m | L1-S22 p. 6 | Verified | p. 6 | — |
| K16 | Gyrodometry ~18× / ~8×; 15 bumps, 10 cm/s | L1-S15 pp. 5–6 | Verified | p. 4 (15 bumps, 10 cm/s); p. 6 | — |
| K17 | 94% (1%) / 61% (12%) | L1-S18 pp. 8–9 | Verified | pp. 8–9 | — |
| K18 | MER ≤10% of 100 m | L1-S23 p. 2 | Verified | p. 2 | — |
| K19 | diff_drive defaults 50 Hz, window 10, covariance 0 | L1-S30 | Verified | yaml | — |
| K20 | Clearpath [0.001×5, 0.01] | L1-S32 | Verified | yaml | — |
| K21 | SPARK MAX quadrature 100 ms, 64 | L1-S33 | Verified | lines 118–141 | — |
| K22 | AMCL α1–α4 0.2 | L1-S38 | Verified | lines 9–29 | — |
| K23 | Radius change under load | L1-S42 p. 11 | Verified | p. 11 | — |
| K24 | 94.7% / 98.1% / 75%; 92.0% IMU-only | L1-S41 p. 3 | Verified | p. 3 | — |
| K25 | Hall 32 ms, 8 samples; ≈112 ms | L1-S33; L1-S34 | Verified | as cited | — |

### How it is tested

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| T1 | UMBmark; second pass if > 3·SEM | L1-S02 pp. 5–7; L1-S01 p. 22 | Verified | as cited | — |
| T2 | Extended UMBmark; average absolute orientation error | L1-S02 pp. 6–7 | Verified | pp. 6–7 | — |
| T3 | Nav2 BT; no pass criterion | L1-S29 | Verified | file | — |
| T4 | 60 runs; 95% ellipses | L1-S05 p. 17 | Verified | p. 17 | — |
| T5 | PC-method; ~100 m validation paths | L1-S13 pp. 6, 9–10 | Verified | as cited | — |
| T6 | Arbitrary trajectories with known end poses | L1-S04 §10.4; L1-S14 pp. 18–19 | Verified | as cited | — |
| T7 | Segment-wise vs RTK / total station; 10/20 m horizons | L1-S19 p. 6; L1-S22 p. 8 | Verified | as cited | — |
| T8 | ε_t, ε_θ; median and quartiles | L1-S20 pp. 5–6 | Verified | p. 6 | — |
| T9 | RPE "e.g. per second or per metre"; start-to-end "poor" | L1-S39 pp. 6–7 | Partly supported | p. 7 gives drift "per frame" (Δ = 1) and "per second" (Δ = 30 at 30 Hz); "per metre" is not in S39 (it is KITTI's deg/m, S40). | Drop "per metre" or cite L1-S40. |
| T10 | ATE after rigid alignment; RMSE | L1-S39 p. 7 | Verified | p. 7 | — |
| T11 | KITTI metric | L1-S40 pp. 5–6 | Verified | as cited | — |
| T12 | Slip indicators vs ground-truth flag | L1-S18 pp. 8–9 | Verified | as cited | — |

### Common mistakes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| C1 | One-direction square gives "excellent" result | L1-S02 pp. 4–5; L1-S03 pp. 133–134 | Verified | S02 p. 5 | — |
| C2 | Correcting only b hides diameter error | L1-S02 p. 5 | Verified | p. 5 | — |
| C3 | Running calibration fast | L1-S02 p. 7 | Verified | p. 7 | — |
| C4 | Ideal DD on skid-steer | L1-S22 p. 7; L1-S20 p. 6 | Verified | as cited | — |
| C5 | Averaging redundant encoders | L1-S16 pp. 4, 14 | Verified | p. 14 Table II (365/266 vs 56/71 mm) | — |
| C6 | Lowest-count without truncation correction underestimates | L1-S16 p. 5 | Verified | p. 5 | — |
| C7 | Inflating covariances to 1e3 | L1-S26 §Odometry 3 | Verified | item 3 | — |
| C8 | Zero covariances; 1e−6; diff_drive defaults zeros | L1-S26; L1-S30 yaml | Verified | as cited | — |
| C9 | Fusing duplicate encoder info | L1-S27 item 1 | Verified | line 54 | — |
| C10 | Two odom→base_link broadcasters | L1-S26 item 5 | Verified | item 5 | — |
| C11 | Wrong signs | L1-S26 item 4 | Verified | item 4 | — |
| C12 | Only start-to-end error | L1-S39 p. 7; L1-S40 p. 5 | Verified | as cited | — |
| C13 | One radius set across load/surface | L1-S42 pp. 3, 11–12 | Verified | as cited | — |
| C14 | Ellipse in (x, y, θ) at long range | L1-S36 p. 1 | Verified | p. 1 | — |
| C15 | Slip via integrated accelerometer speed | L1-S41 p. 4 | Verified | p. 4 | — |
| C16 | Filtering velocity too heavily adds lag | L1-S25 p. 3; L1-S35 §Recommended Procedure | Verified | S25 p. 3; S35 line 432 "still responsive enough" | — |

### Disagreements between sources

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| D1 | Survey: tracked odometry "virtually impossible", skid steer "tele-operated"; later work usable | L1-S03 pp. 28, 139; L1-S19 p. 6; L1-S21 §V-C; L1-S22 p. 8 | Verified | all locations | — |
| D2 | Nav2 CCW only vs bidirectional requirement | L1-S29; L1-S02 pp. 4–5 | Verified | as cited | — |
| D3 | "inversely" vs "directly proportional"; same formula | L1-S01 pp. 18–19; L1-S03 p. 141 | Verified | S01 p. 18; S03 p. 141 | — |
| D4 | Asphalt lower χ; snow ideal DD better but more rotation | L1-S19 p. 5; L1-S20 pp. 6–7 | Verified | as cited | — |
| D5 | Distance-growing covariance vs fixed diagonals | L1-S05 p. 7; L1-S07 p. 8; L1-S30; L1-S32 | Verified | as cited | — |

### Open questions (cited)

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| O1 | No grass figures for rubber tracks; LandTamer wheeled; "Endo's CV-04 was tested indoors" | L1-S22; L1-S21 | Partly supported | S22 p. 7 Fig. 6 "six-wheeled skidsteer" ✓. S21 does not say "indoors"; it lists P-tile, plywood and artificial turf (p. 3) under a fixed 280 cm motion-capture camera (Table I). | Replace "indoors" with "on P-tile, plywood and artificial turf". |
| O2 | Baril nonlinearity at ratio ≈2 | L1-S20 p. 7 | Verified | p. 7 | — |
| O3 | Lag described, pose effect not quantified | L1-S25; L1-S34; L1-S30 | Verified | none of these quantifies pose error | — |
| O5 | Unread works known only through L1-S08, S12, S14, S20 | L1-S08; L1-S12; L1-S14; L1-S20 | Partly supported | Antonelli: S08 p. 1, S14 p. 17 ✓; Martinelli 2007: S14 p. 18 ✓; Wang: S12 p. 1 ✓; Pentzer: S20 p. 2 ✓. Thrun is described through L1-S36–S38, Martínez 2005 through L1-S19 (ref. 17), Ward & Iagnemma model-based through L1-S41. | Add L1-S19, L1-S36–S38 and L1-S41 to the list. |
| O6 | ros2_controllers applies same multiplier to both | L1-S30 | Verified | controller.cpp 143–145, 257–260, 298–302 | — |
| O7 | Second-order model at large k; banana; AMCL sampling; 1–2° is EKF-SLAM | L1-S05 p. 15; L1-S36; L1-S37 | Verified | as cited | — |
| O8 | Integration schemes described, not quantified | L1-S03 pp. 20, 138; L1-S07 p. 7; L1-S30 | Verified | as cited | — |
| O9 | 3D beyond Seegmiller | L1-S22 | Verified | S22 3D model | — |
| O10 | α / k defaults or indoor values only | L1-S38; L1-S05 | Verified | as cited | — |

## Corrections applied (2026-09-27)

Applied by the topic editor after this claims review and SOURCE_AUDIT.md. Every flagged location was re-read in the source before rewording.

### Claims
| # | Change |
|---|---|
| S2 | Calibratability now cited where it is stated: systematic errors reducible by vehicle-specific calibration [L1-S03, p. 139]; non-systematic errors cannot be calibrated out [L1-S15, p. 5]. Lists stay on L1-S02 p. 3 / L1-S03 pp. 130–131. |
| S3 | Added "on a straight path" and "identical noise statistics on both wheels" (L1-S04 p. 25). |
| S4 | Doh's PC-method now described as one path driven forward then backward [L1-S13, p. 6]; "arbitrary paths" kept only for Kelly, Censi, Seegmiller. |
| S5 | "the usual fix" replaced by "one fix"; added L1-S20 p. 1 ("commonly deployed" models); yaw-rate over-prediction cited to L1-S22 p. 7. |
| S6 | "6×6" dropped; robot_localization advice now conditional on position, heading and velocity coming from the same encoders (L1-S27 line 54). |
| 1.3 | Runge–Kutta label moved to [L1-S30, odometry.cpp lines 135–143] (quoted code comment). |
| 3.4 | Added the identical-encoder-noise condition. |
| 4.10, K10 | "~12 m each" → start and end points approximately 12 m apart (L1-S14 p. 19). |
| 5.5 (Not supported) | Rewritten: the 54 % / 92 % gains are for Merry's skip option compared with time-stamping without skip [L1-S24, pp. 1, 6]. |
| 5.11 | CTRE procedure split: raise sampling period until granular, then rolling-average window until smooth (L1-S35 lines 427–432). |
| 9.2 | Tape measure now cited to L1-S03 p. 143 and L1-S01 p. 23; sonar calibrator stays on L1-S01 p. 20. |
| R10 | Added the same-encoders condition. |
| T9 | "per metre" → "per frame or per second" (L1-S39 p. 7). |
| O1 | "tested indoors" → "tested on P-tile, plywood and artificial turf under a fixed motion-capture camera" [L1-S21, p. 3, Table I]; LandTamer cited to L1-S22 p. 7. |
| O5 | Summarising-source list completed: L1-S08, S12, S14, S19, S20, S36–S38, S41. |
| New open question | Jung & Chung heading-error calibration and other bidirectional paths (SCOPE 4b) recorded as not covered. |

### Sources
| ID | Change |
|---|---|
| L1-S01 | Issue corrected 12(5) → 12(6), Dec. 1996, DOI added (confirmed via Crossref); also corrected in SCOPE.md. |
| L1-S02 | Added Proc. SPIE 2591, pp. 113–124, DOI 10.1117/12.228968 (Crossref). |
| L1-S03, L1-S06 | Levels made consistent: both A under a rule stated above the Sources table (standard university-lab technical reports); L1-S06 was B. Publisher note (ORNL/DOE) added to L1-S03. |
| L1-S05 | Foundational row now names the peer-reviewed ICRA 1997 venue, with the tech report as the file. |
| L1-S11 | File renamed `ros2_2022_…` → `ros2_2020_nav_msgs_odometry.msg` to match the 2020 file change; citation reworded. |
| L1-S20 | Published venue applied: Proc. 17th CRV, pp. 198–205, 2020, DOI 10.1109/CRV50864.2020.00034; arXiv file kept as the open copy (published-layout PDF not substituted, so cited page numbers stay valid). |
| L1-S21 | Page range still not available (not in Crossref/OpenAlex); stated in the citation. |
| L1-S35 | "c. 2022" → undated page on unversioned "stable" docs. |
| L1-S41 | Pages 2730–2735 and DOI confirmed via Crossref. |
| Foundational section | Completed: REP-103 (L1-S10) added to the ROS row; each not-downloaded foundational work (Wang, Antonelli, Martinelli 2007, Siegwart 2nd ed., Thrun, Martínez) now has its own row with why it is foundational and which source describes it. |
| Not-downloaded table | Renamed "References not downloaded"; added Jung & Chung 2011, Kelly 2013 textbook, Iagnemma & Dubowsky 2004 (supporting, not cited). |

No source failed the audit; no files were deleted. Mechanical check: `file` output matches every extension; all 47 files appear in the Sources table; every cited ID exists; IDs L1-S01…L1-S42 have no gaps or duplicates. README Status set to Verified.
