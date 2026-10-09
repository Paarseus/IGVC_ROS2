# C1 — Motor velocity control: claim verification

| | |
|---|---|
| **Topic** | C1 — Motor velocity control (`README.md`, 495 lines, sources C1-S01 to C1-S51) |
| **Date** | 2026-09-28 |
| **Reviewer** | independent — claims |
| **Rules** | `research/STANDARDS.md` section 5 (step 4, claims only; sources are audited separately in `SOURCE_AUDIT.md`) |
| **Method** | Every cited item in Summary, Foundational references, Findings, Recommended practice, Key numbers, How it is tested, Common mistakes, Disagreements and cited Open questions was checked at its cited location. PDFs were read with `pdftotext -layout` (non-layout for `astrom_2002_ch6_pid_control.pdf`); the three scanned PDFs with no text layer (Ziegler & Nichols 1942, Åström & Hägglund 1984, Canudas de Wit et al. 1995) were rendered with `pdftoppm` and read page by page; Peng et al. 1996 was read from the decoded text copy. Page numbers are the printed ones (report page numbers for C1-S43/S44, preprint/author-version numbers where the README says so). Markdown/RST/Java/C++ sources were read directly and cited line numbers confirmed with `sed -n`. This file replaces the 2026-09-27 verification, which predates the gap-check additions (C1-S38 to C1-S51). |

## Counts

| Section | Checked | Verified | Partly supported | Not supported |
|---|---|---|---|---|
| Summary | 6 | 5 | 1 | 0 |
| Foundational references | 12 | 8 | 4 | 0 |
| Findings: Loop structure: cascades, feedforward and two-degree-of-freedom control (general) | 6 | 5 | 1 | 0 |
| Findings: PI or PID for a velocity loop, and tuning from a delay model (general) | 11 | 10 | 1 | 0 |
| Findings: Integrator windup and anti-windup (general) | 13 | 13 | 0 | 0 |
| Findings: Delay, sampling and measurement filtering limit the loop (general) | 10 | 10 | 0 | 0 |
| Findings: System identification of motor drives (general) | 17 | 17 | 0 | 0 |
| Findings: Disturbance observers: robust speed control under model–plant mismatch (general) | 5 | 5 | 0 | 0 |
| Findings: Friction modelling and compensation (general) | 15 | 15 | 0 | 0 |
| Findings: Actuator limits outside FRC: thermal current limiting and supply-voltage compensation (general) | 10 | 9 | 1 | 0 |
| Findings: Wheel and track velocity control on other ground vehicles (general) | 13 | 13 | 0 | 0 |
| Findings: How the controller-side velocity loop works | 17 | 16 | 1 | 0 |
| Findings: SPARK MAX gain units and the firmware 25/26 changes | 16 | 15 | 1 | 0 |
| Findings: Measuring feedforward (system identification) | 21 | 21 | 0 | 0 |
| Findings: Tuning order and what limits the feedback gains | 20 | 20 | 0 | 0 |
| Findings: Velocity measurement filtering | 10 | 10 | 0 | 0 |
| Findings: Current limits, voltage compensation and ramps | 13 | 13 | 0 | 0 |
| Findings: Low-speed precision | 12 | 12 | 0 | 0 |
| Recommended practice | 21 | 20 | 1 | 0 |
| Key numbers | 48 | 47 | 1 | 0 |
| How it is tested | 21 | 21 | 0 | 0 |
| Common mistakes | 23 | 23 | 0 | 0 |
| Disagreements between sources | 11 | 11 | 0 | 0 |
| Open questions | 8 | 8 | 0 | 0 |
| **Total** | **359** | **347** | **12** | **0** |

Not graded: the five Foundational-references rows marked "—" (references that were not downloaded and are not cited), and the five Open-question bullets with no citation (README lines 433–436 and 440; they are statements that no source was found). No uncited factual statement was found elsewhere in the graded sections apart from the evaluative wording in the Foundational-references rows flagged below (item-by-item rows 8, 9, 12 and 18).

## Items needing correction (all non-Verified items)

| # | README line | Item | Status | Problem | Correction needed |
|---|---|---|---|---|---|
| 1 | 15 | Summary: drives "identified from open-loop tests: in general as first-order-plus-delay models", citing the LandTamer τc = 0.81 s, τd = 0.16 s | Partly supported | Seegmiller et al. estimated τc and τd "online using an extended Kalman filter" while the vehicle "drove in circles" (C1-S35 pp. 20–21), not from an open-loop test; "in general" rests on two examples. | "Drives are identified from measured data as first-order-plus-delay models, e.g. from open-loop steps (Agribot) or by online EKF fitting while driving (LandTamer)". |
| 2 | 23 | Foundational: C1-S28 "by the leading PID authority" | Partly supported | Uncited evaluative claim. | Drop "the leading PID authority" or cite a source for it. |
| 3 | 24 | Foundational: C1-S29 "Most-cited analytic PI/PID tuning rules" | Partly supported | Uncited superlative. | Drop "Most-cited" or cite a citation count. |
| 4 | 27 | Foundational: C1-S38 "First published PID tuning rules" | Partly supported | The paper does not claim to be first, and the claim is uncited (earlier controller-setting work exists, e.g. Callender, Hartree & Porter, 1936). | "Classic early PID tuning rules (ultimate-gain and reaction-curve methods)". |
| 5 | 33 | Foundational: C1-S45 "the most common robust motion-control tool in drives" | Partly supported | Not stated in the source, which says practitioners "have widely adopted" the DOb (p. 9). | "a widely adopted robust motion-control tool". |
| 6 | 47 | Setpoint weighting cited to "eq. 6.5" | Partly supported | The weighted controller and the two-degree-of-freedom remark are at eq. (6.4); (6.5) is the reference-to-control transfer function. | Cite "section 6.3, eq. 6.4 (and 6.5), Fig. 6.4". |
| 7 | 60 | Relay test: "the loop then oscillates at the critical period" | Partly supported | Source: a system "with a phase lag of at least π at high frequencies **may** oscillate with period tc under relay control" (p. 646), and the period is approximate (describing function, p. 647). | Add the condition and hedge: "for a process with at least 180° phase lag at high frequency, the loop may oscillate at approximately the critical period". |
| 8 | 134 | "A common thermal model of a motor has two heat capacities …" | Partly supported | The source presents this as its "Basic Thermal Model" and says it is "applicable to" brushless and brushed DC motors; "common" is not in the source. | "A two-node thermal model (Kawaharazuka et al.) …". |
| 9 | 164 | "A closed-loop controller uses sensor feedback to reduce the difference ('error') between the setpoint and the measured value" | Partly supported | The cited REV section says only "uses feedback to improve the accuracy of its outputs"; it does not define error. | Use the source's wording, or cite a source that defines error (e.g. C1-S04 "multiplied by the error"). |
| 10 | 188 | REV units table "is written for position control" | Partly supported | The table gives per-rotation units affected by the position conversion factor but does not say it is for position control; this is an unlabelled inference. | "(its per-rotation units suggest the position form)". |
| 11 | 307 | Recommended practice 18: "raise P until just before oscillation" | Partly supported | CTRE says increase kP "until the output starts to oscillate around the setpoint"; none of the three cited sources (C1-S22, S16, S17) says to stop just before oscillation. REV (C1-S04 step 9) says to decrease P if it oscillates but is not cited here. | Add C1-S04 "Tuning" step 9 to the citation, or reword to CTRE's "until the output starts to oscillate, then back off". |
| 12 | 336 | Key numbers: kP/kI/kD units "REV units table (position form)" | Partly supported | Same unlabelled inference as #10. | "(per-rotation form; inferred to be position form)". |

No item was found Not supported. The eight problems found in the 2026-09-27 review have all been corrected in the current README (SIMC τ1 ≤ 8θ condition, "can be modelled adequately", TurtleBot PD "assumed", RSD 3.05–4.08%, NEO locked-rotor trend removed, open-question wording, deleted Hall-sensor bullet).

## Item-by-item check

Status column: Verified / Partly supported / Not supported. "Line" is the README line number. Citations are copied from the README (S-numbers in the evidence column abbreviate C1-Sxx).

### Summary

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 1 | 12 | Structure: a velocity loop works best as feedforward (model-based voltage for friction, speed a… | C1-S16, "Choice of Control Strategies"; C1-S15, "The Permanent-Magnet DC Motor Feedforward Equation"; C1-S24, section 11.6, pp. 341–342 | Verified | S16 "Choice of Control Strategies": FB "must be combined with a feedforward"; S15 eq. V = kS·sgn + kV·v + kA·a; S24 p. 341 "10 rad/s (10 times that of the outer loop)", p. 342 "may not be valid ... we must verify this" | — |
| 2 | 13 | Delay sets the achievable gain and bandwidth: sampling adds about h/2 (often several periods), … | C1-S28, section 6.7; C1-S29, section 2.1 and Table 2; C1-S37, section 2.2; C1-S20, `FeedbackControllerPreset.hpp` lines 131–139; C1-S18, `analyzing-gains.rst`, "Measurement Delays" | Verified | S28 §6.7 "average delay ... h/2 ... often several sampling periods"; S29 Table 2 PM 61.4, ωc 0.50 (τ1 ≤ 8θ column); S37 p. 898 "PI is preferred over PID"; S20 L138–139 `112_ms`; S18 "can cause the calculated gains ... to be unstable" | — |
| 3 | 14 | Any controller with integral action winds up when the actuator saturates; back-calculation (tra… | C1-S28, section 6.5; C1-S30, section II; C1-S04, "Tuning"; C1-S17, "Integral Term Windup"; C1-S22, "Closed-Loop Overview" | Verified | S28 §6.5 "Tt should be larger than Td and smaller than Ti ... √(TiTd)", incremental: "inhibiting integration whenever the output saturates"; S04 "I ... not often recommended in FRC"; S22 kS → kV → kP order | — |
| 4 | 15 | Drives are identified from open-loop tests: in general as first-order-plus-delay models (e.g. τ… | C1-S37, section 3.1; C1-S35, section 4; C1-S18, "Creating an Identification Routine" and `loading-data.rst`; C1-S20, `Drive.java` lines 61–74 | Partly supported | S37 p. 902 "open-loop tests" → 49.3/(0.15s+1)e^−0.2s. S35 p. 20–21: τc, τd "estimated online using an extended Kalman filter" while the LandTamer "drove in circles" | The LandTamer values come from online EKF calibration during normal driving, not an open-loop test; "in general" rests on two examples. Reword: "identified from measured data, e.g. open-loop steps (Agribot) or online EKF fitting while driving (LandTamer)". |
| 5 | 16 | Friction is usually modelled as Coulomb plus viscous, with stiction and Stribeck effects at low… | C1-S31, sections II and IV; C1-S32, sections 4.1.3–4.1.6; C1-S26, ch. 2.1; C1-S25, abstract and section 6; C1-S12, "Motor Specifications" | Verified | S31 §II Coulomb + viscous, stiction, Stribeck; S32 §4.1.6 p = 0.0014 (direction), §4.1.9 36% (method); S31 §IV "no method ... intrinsically superior"; S26 §2.1 limit cycling; S12 "42 counts per rev." | — |
| 6 | 17 | Product-specific: on SPARK firmware/REVLib 2026 the feedforward terms are in **volts** (kS in V… | C1-S02, table "Default Units"; C1-S09, line 256; C1-S11, 2025 line 72 / 2026 lines 74–75 | Verified | S02 table: kS Volts, kV Volts per RPM, kA Volts per RPM/s, kP "Duty cycle per rotation"; S09 L256 "`kV` (formerly `kF`)"; S11 2025 L72 `.velocityFF(1.0 / 5767`, 2026 L74–75 comment + `.kV(12.0 / 5767` | — |

### Foundational references

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 7 | 22 | C1-S24 — Standard control textbook: cascade design, delay limits on bandwidth,  | C1-S24 | Verified | Textbook content matches: §11.6 cascade, §11.5 delay limits, §10.4 windup, §10.5 derivative filter | — |
| 8 | 23 | C1-S28 — PID structure, setpoint weighting, anti-windup and sampling/discretiza | C1-S28 | Partly supported | Chapter covers structure, set-point weighting (§6.3), windup (§6.5), sampling (§6.7) | "by the leading PID authority" is an uncited evaluative claim; drop it or cite. |
| 9 | 24 | C1-S29 — Most-cited analytic PI/PID tuning rules from a first-order-plus-delay  | C1-S29 | Partly supported | Paper gives SIMC rules and "half rule" (abstract, §2.1, §3.3) | "Most-cited" is an uncited superlative; drop it or cite a citation count. |
| 10 | 25 | C1-S31 — Survey of friction models and compensation classes | C1-S31 | Verified | Survey of models (§II) and compensation classes A–D (§III) | — |
| 11 | 26 | C1-S12 — Primary manufacturer specification of the motor | C1-S12 | Verified | REV NEO V1.1 page, "Motor Specifications" table | — |
| 12 | 27 | C1-S38 — First published PID tuning rules (ultimate-gain and reaction-curve met | C1-S38 | Partly supported | Ultimate-sensitivity and reaction-curve methods on pp. 762–765 | "First published PID tuning rules" is uncited and not stated in the paper (earlier settings exist, e.g. Callender, Hartree & Porter 1936). Reword to "classic early tuning rules" or cite a history source. |
| 13 | 28 | C1-S39 — Introduced relay-feedback estimation of the critical point used by aut | C1-S39 | Verified | p. 646: "Another method for automatic determination ... is therefore proposed" (relay) | — |
| 14 | 29 | C1-S40 — Original dynamic friction model (Stribeck, hysteresis, stiction, varyi | C1-S40 | Verified | Abstract p. 419: "Stribeck effect, hysteresis, spring-like characteristics for stiction, and varying break-away force"; §V-A hunting | — |
| 15 | 30 | C1-S42 — Open summary by the authors of the standard computer-control text; sam | C1-S42 | Verified | pp. 39–40 "Selection of Sampling Interval and Antialiasing Filters" | — |
| 16 | 31 | C1-S43 — Overview by the author of the standard identification textbook: estima | C1-S43 | Verified | §2 "The Core": estimation, model fit, complexity, validation | — |
| 17 | 32 | C1-S44 — Standard reference on identification from data taken under feedback | C1-S44 | Verified | §1 closed-loop identification problems | — |
| 18 | 33 | C1-S45 — Survey of the disturbance observer, the most common robust motion-cont | C1-S45 | Partly supported | Overview of DOb; p. 9 practitioners "have widely adopted this robust control technique" | "the most common robust motion-control tool in drives" is an uncited superlative not stated in the source; soften to "widely adopted". |

### Findings: Loop structure: cascades, feedforward and two-degree-of-freedom control (general)

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 19 | 43 | In an inner/outer ("cascade") design the inner loop is designed first to give "fast and accurat… | C1-S24, section 11.6, p. 341 | Verified | p. 341: inner loop gives "fast and accurate control of the roll angle"; "Under the assumption that the dynamics of the roll controller are fast relative to the desired bandwidth" | — |
| 20 | 44 | In Åström & Murray's worked cascade example the inner-loop bandwidth was chosen as 10 rad/s, "1… | C1-S24, section 11.6, pp. 341–343, Figs. 11.17–11.19 | Verified | p. 341 "10 rad/s (10 times that of the outer loop)"; p. 342 "/Lo/ < 0.1 for ω > 10 rad/s"; p. 343 Fig. caption "phase margin of 68° and a gain margin of 6.2" | — |
| 21 | 45 | The "inner loop is perfect" approximation "may not be valid", so it must be checked once the fu… | C1-S24, section 11.6, p. 342 | Verified | p. 342 "this approximation may not be valid, and so we must verify this when we complete our design" | — |
| 22 | 46 | PID is typically the lowest level of a hierarchy, with higher-level controllers (e.g. model pre… | C1-S28, section 6.1 | Verified | §6.1 "PID control is used at the lowest level"; "more than 95% ... most loops are actually PI control" | — |
| 23 | 47 | Setpoint weighting (a weight b on the reference in the proportional term and c in the derivativ… | C1-S28, section 6.3, eq. 6.5 and Fig. 6.4 | Partly supported | §6.3: "The controller given by (6.4) has a structure with two degrees of freedom because the signal path from y to u is different from that from r to u" | The weighted controller is eq. (6.4); (6.5) is the r-to-u transfer function. Cite "eq. 6.4 (and 6.5)". |
| 24 | 48 | Model-based friction compensation can be applied as feedforward, using reference positions/velo… | C1-S31, section IV | Verified | §IV: FF version "with a significantly reduced on-line computational burden" | — |

### Findings: PI or PID for a velocity loop, and tuning from a delay model (general)

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 25 | 51 | The SIMC rules for a first-order-plus-delay model k·e^(−θs)/(τ1·s + 1) are Kc = (1/k)·τ1/(τc + … | C1-S29, section 3.3, eqs. 23–25, p. 296 | Verified | p. 296 eqs. 23–25; "PID-control ... is primarily recommended for processes with dominant second order dynamics (with τ2 > θ, approximately)" | — |
| 26 | 52 | The recommended choice τc = θ ("fast response with good robustness") gives, for a first-order d… | C1-S29, eq. 28, p. 296; section 4.1.1 and Table 2, p. 297 | Verified | eq. 28 p. 296; Table 2 p. 297: GM 3.14/2.96, PM 61.4/46.9, Ms 1.59/1.70, ωc 0.50; §4.1.1 "much better than ... GM > 1.7 and PM > 30" | — |
| 27 | 53 | A larger τc detunes the loop: it lowers the controller gain, which reduces input usage and sens… | C1-S29, section 6.1, p. 303 | Verified | §6.1 p. 303: "to reduce the manipulated input usage, reduce measurement noise sensitivity and generally make operation smoother, we may want detune"; larger τc "decreases the controller gain" | — |
| 28 | 54 | If measurement noise is a problem with SIMC settings, the source suggests, in order: filter the… | C1-S29, section 6.2, pp. 303–304 | Verified | §6.2 pp. 303–304: filter τF "up to about 0.5θ, without a large affect on performance and robustness"; 2. remove derivative; 3. detune | — |
| 29 | 55 | For a DC motor speed model of first order with transport delay L and time constant τ, one agric… | C1-S37, section 2.2, p. 898 | Verified | p. 898 "L < τ/2 ... L < 2τ; otherwise, the dead-time rule ... PI is preferred over PID to keep stability" | — |
| 30 | 56 | In that study the Cohen–Coon tuning gave a higher proportional gain with "significant overshoot… | C1-S37, section 4.1, p. 904 | Verified | p. 904 "significant overshoot and slow damping"; "could be problematic when the sample time is long"; Fig. 7 "Ts = 150 ms" | — |
| 31 | 57 | Ziegler and Nichols' ultimate-gain method finds the "ultimate sensitivity" Su (the proportional… | C1-S38, p. 762 and "Summary of Controller Adjustments", p. 765 | Verified | p. 762 "Sensitivity = 0.45Su, Reset rate = 1.2/Pu"; p. 765 summary: P 0.5Su; PI 0.45Su, 1.2/Pu; PID 0.6Su, 2/Pu, Pu/8 (bracketed modern-term gloss is the README's) | — |
| 32 | 58 | The same paper gives equivalent settings from the open-loop "reaction curve" (process reaction … | C1-S38, "Optimum Settings From Reaction Curve", p. 764, and p. 765 | Verified | p. 764 "Sensitivity = 0.9/(R1L) psi per in., Reset rate = 0.3/L per min"; repeated in p. 765 summary | — |
| 33 | 59 | Ziegler and Nichols note that the proportional-only rule (half the ultimate sensitivity, with a… | C1-S38, pp. 761 and 763 | Verified | p. 761 "25 per cent amplitude ratio ... must be modified in some cases"; p. 763 "not generally useful on pressure- or flow-control applications ... used most widely on temperature-control applications" | — |
| 34 | 60 | Åström and Hägglund replaced the manual ultimate-gain experiment, which they describe as diffic… | C1-S39, section 2, p. 646, eq. 1 | Partly supported | p. 646: ZN experiment "difficult to automatize ... amplitude of the oscillation is kept under control"; eq. 1 kc = 4d/(πa). But: "a system with a phase lag of at least π at high frequencies may oscillate with period tc under relay control" | Add the condition and hedge: "for a process with at least 180° phase lag at high frequency the loop may oscillate at (approximately) the critical period". |
| 35 | 61 | The relay experiment generates its own test signal with strong content at the critical frequenc… | C1-S39, abstract, p. 645; section 2, p. 646; section 3, p. 647 | Verified | Abstract "not sensitive to modelling errors and disturbances"; p. 646 "significant frequency content at ωc", hysteresis "less sensitive to measurement noise"; p. 647 "error of a few per cent" | — |

### Findings: Integrator windup and anti-windup (general)

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 36 | 64 | Windup happens because "all actuators have limitations": when the control signal hits a limit t… | C1-S28, section 6.5 | Verified | §6.5 "All actuators have limitations ... feedback loop is broken ... error has opposite sign for a long period ... any controller with integral action may give large transients" | — |
| 37 | 65 | Limiting setpoint changes so the actuator never saturates "frequently leads to conservative bou… | C1-S28, section 6.5, "Setpoint Limitation" | Verified | §6.5 Setpoint Limitation: "frequently leads to conservative bounds and poor performance ... does not avoid windup caused by disturbances" | — |
| 38 | 66 | In incremental ("velocity") PID algorithms, windup is avoided by inhibiting integration wheneve… | C1-S28, section 6.5, "Incremental Algorithms" | Verified | §6.5 Incremental: "avoid windup by inhibiting integration whenever the output saturates. This method is equivalent to back-calculation" | — |
| 39 | 67 | Back-calculation feeds the difference between the controller output and the actual (measured or… | C1-S28, section 6.5, "Back-Calculation and Tracking" and Fig. 6.7 | Verified | §6.5, Fig. 6.7: es fed "through gain 1/Tt. The signal is zero when there is no saturation" | — |
| 40 | 68 | The tracking time constant Tt "should be larger than Td and smaller than Ti", with a suggested … | C1-S28, section 6.5, "Back-Calculation and Tracking" | Verified | §6.5: "Tt should be larger than Td and smaller than Ti. A rule of thumb ... √(TiTd)"; "spurious errors can cause saturation ... accidentally resets the integrator" | — |
| 41 | 69 | Modern anti-windup design states two goals: "small signal preservation" (no change to the respo… | C1-S30, section II-A, p. 307 | Verified | §II-A p. 307: "(small signal preservation)", "(large signal recovery)" | — |
| 42 | 70 | Anti-windup design always assumes that the unconstrained (unsaturated) closed loop is asymptoti… | C1-S30, section II-B, p. 307 | Verified | §II-B p. 307: "in anti-windup design it is always assumed that the unconstrained controller guarantees closed-loop ... stability" | — |
| 43 | 71 | Saturation can cause loss of performance "or even stability", and the tutorial argues for accou… | C1-S30, section I, p. 306 | Verified | §I p. 306: "performance, or even stability, loss"; "available in the actuators, rather than oversizing them" | — |
| 44 | 72 | Peng, Vrančić and Hanus define windup as the performance loss when "the real plant input is tem… | C1-S41, abstract and introduction | Verified | Decoded extract: "a motor driven actuator has a limited speed"; "most suitable anti-windup strategy for usual applications is the conditioning technique, using ... the realizable reference. The exception is ... too restrictive" | — |
| 45 | 73 | The same review says it is a misconception that anti-windup aims to reduce the step-response ov… | C1-S41, introduction | Verified | Introduction: "many authors think that anti-windup is aimed at reducing the output overshoot ... or that anti-windup is the synonym of bumpless transfer ... These thoughts need to be corrected" | — |
| 46 | 74 | A rate limit (a cap on how fast the actuator input can change, which is what an output ramp doe… | C1-S30, section III-E, item 3, pp. 316–317 | Verified | §III-E item 3 pp. 316–317: "inertia in various components ... very fast"; model = saturation "with an integrator and gain and enclosing this within a feedback loop"; [95] "magnitude and rate saturation" | — |
| 47 | 75 | Rate-limited actuators have been linked to pilot-induced oscillations and the loss of several a… | C1-S30, section III-E, item 3, p. 317 | Verified | p. 317: "pilot-induced-oscillations (PIOs) and the subsequent untimely demise of several aircraft due to rate-limited actuators" | — |
| 48 | 76 | In incremental (velocity-form) PID algorithms "it is also easy to limit the rate of change of t… | C1-S28, section 6.5, "Incremental Algorithms" | Verified | §6.5 Incremental: "It is also easy to limit the rate of change of the control signal" | — |

### Findings: Delay, sampling and measurement filtering limit the loop (general)

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 49 | 79 | Digital implementation adds dead time: if the input is read with sampling period h and the outp… | C1-S28, section 6.7, "Sampling" | Verified | §6.7 Sampling: "average delay of the measurement signal is h/2 ... most controllers ... do not organize the calculation in this way ... often several sampling periods" | — |
| 50 | 80 | Sampling can alias high-frequency disturbances into low-frequency signals inside the loop bandw… | C1-S28, section 6.7, "Aliasing", "Prefiltering" and Example 6.3 | Verified | §6.7 Prefiltering: high frequencies "appear as low-frequency signals in the bandwidth"; Ex. 6.3 "(ωs/2ωb)² = 16 ... ωb = ωs/8"; "should be accounted for in the control design" | — |
| 51 | 81 | With a backward-difference derivative the filter coefficient Td/(Td + N·h) always lies between … | C1-S28, section 6.7, "Discretization", eq. 6.17 | Verified | §6.7 eq. 6.17: "Td/(Td + Nh) is in the range of 0 to 1 ... guarantees that the difference equation is stable" | — |
| 52 | 82 | The "half rule" folds fast dynamics into an effective delay: the largest neglected time constan… | C1-S29, section 2.1, eqs. 10–11, p. 293 | Verified | p. 293 half rule: "largest neglected ... time constant ... distributed evenly to the effective delay and the smallest retained time constant"; "sampling period h ... approximately h/2" | — |
| 53 | 83 | The effect of a delay on control performance "is worse than that of a lag of equal magnitude". | C1-S29, section 2.1, p. 293 | Verified | p. 293: "the effect of a delay on control performance is worse than that of a lag of equal magnitude" | — |
| 54 | 84 | A zero-order hold behaves, for small sampling periods, like a time delay of half a sampling int… | C1-S42, "Selection of Sampling Interval and Antialiasing Filters", pp. 39–40, eq. 36 | Verified | pp. 39–40: hold ≈ "time delay of half a sampling interval"; "phase margin can be decreased by 5° to 15°"; eq. 36 "hωc ≈ 0.05 to 0.14"; "Nyquist frequency ... about 23 to 70 times higher than the crossover frequency" (assumes ζf = 0.707, gN = 0.1) | — |
| 55 | 85 | The same brief states that "Antialiasing filters are important in all cases". | C1-S42, p. 40, summary list | Verified | p. 40 summary bullet: "Antialiasing filters are important in all cases." | — |
| 56 | 86 | Low-resolution Hall sensors split each 360° electrical cycle into six 60° sectors. | C1-S36, section 2.1, p. 2 | Verified | §2.1 p. 2: "six sections of 60° each" | — |
| 57 | 87 | The simplest Hall speed estimate (average speed over the previous sector) assumes constant spee… | C1-S36, section 3.1, pp. 4–5, and Table 1 | Verified | §3.1 pp. 4–5 and Table 1; p. 5 "considerable delays in the estimation results" | — |
| 58 | 88 | Observer-based Hall speed estimation has its own trade-off: back-EMF observers work poorly at l… | C1-S36, section 3.3, pp. 8–9 | Verified | §3.3 p. 8: back-EMF "difficult" at low speed; VTO "High controller bandwidth may lead to considerable fluctuations ... low bandwidth can undermine observer stability"; p. 9 "extending the speed loop bandwidth at low speeds remains problematic" | — |

### Findings: System identification of motor drives (general)

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 59 | 91 | Neglecting inductance and friction, the DC motor speed response to voltage reduces to a first-o… | C1-S37, section 2.2, eqs. 12–15, p. 898 | Verified | p. 898 eqs. 12–15; parameters "can be obtained by means of open-loop tests" (§2.2) | — |
| 60 | 92 | On a differential-drive agricultural robot ("Agribot"), open-loop step tests gave K = 49.3 rad/… | C1-S37, section 3.1, eq. 35, p. 902 | Verified | p. 902 eq. 35 49.3/(0.15s+1)e^−0.2s; "static gain from the data sheet ... K = 52.77 rad/s/V. The time constant and transport delay must be obtained from measurements because they depend on the utilization conditions" | — |
| 61 | 93 | The same drive had a dead zone: the motor started only when the control signal exceeded ±20% of… | C1-S37, section 3.1, p. 902 | Verified | p. 902 "motor starts only when the controller's control signal exceeds ±20% of its nominal value" | — |
| 62 | 94 | The response of many field-robot powertrains "can be modeled adequately" as a time delay plus f… | C1-S35, section 4, p. 20, eq. 50 | Verified | p. 20: "No powertrain can instantaneously drive wheels"; "response of many powertrains can be modeled adequately by a time delay and a first-order"; "Most WMR motion models in related work omit powertrain dynamics"; MPC "future wheel velocities must be predicted" | — |
| 63 | 95 | τc and τd can be identified online with an extended Kalman filter that compares predicted and e… | C1-S35, section 4, pp. 20–21, Fig. 5 | Verified | p. 20–21: "estimated online using an extended Kalman filter"; LandTamer "time constant = 0.81, delay = 0.16 sec" | — |
| 64 | 96 | Fitting the integrated prediction (wheel velocity) was "more accurate and robust to noise than … | C1-S35, section 4, p. 21 | Verified | p. 21: "IPEM calibration was more accurate and robust to noise than calibration to ω̇ residuals"; "instrumental in removing large prediction errors associated with transients" | — |
| 65 | 97 | Constant-voltage (step) tests at many voltage levels in both directions give steady-state speed… | C1-S32, sections 3.1–3.3 and 4.1.5 | Verified | §3.1 constant torque, §3.2 ramp (slope/intercept m, b), §3.3 inertia from transient; §4.1.5 | — |
| 66 | 98 | In those experiments viscous friction agreed within 0.43% between the step and ramp methods, bu… | C1-S32, sections 4.1.3 and 4.1.9, Tables 4 and 10 | Verified | §4.1.3 Table 4: fc 0.5141 vs 0.3293 N·m; §4.1.9 "only 0.43% relative difference" (viscous), "36% inter-method difference" | — |
| 67 | 99 | Coulomb friction was significantly direction-dependent (ANOVA p = 0.0014) while viscous frictio… | C1-S32, section 4.1.6 | Verified | §4.1.6: Coulomb "F(3,16)=8.42 and p=0.0014"; viscous "p=0.067"; "identification across multiple operational quadrants and employing averaged parameter values" | — |
| 68 | 100 | The inertia estimate depended on which friction values were used (0.1346 vs 0.1178 kg·m², a 12.… | C1-S32, sections 4.1.5 and 4.1.8, Tables 6 and 9 | Verified | Table 6/9: J 0.1346 vs 0.1178 kg·m²; §4.1.8 "The 12.5% relative difference" | — |
| 69 | 101 | The 12 V test motor needed about 2.3 V before it began rotating. | C1-S32, section 4.1.2 | Verified | §4.1.2 "minimum voltage of approximately 2.3 V to overcome static friction" | — |
| 70 | 102 | For a differential-drive AGV (TurtleBot 2), offline Levenberg–Marquardt identification used to … | C1-S33, Table 4 | Verified | Table 4: ev 0.001 (RLS + L-M), 0.023 (L-M), 0.011 (RLS); tr 1.6, 2.9, 7.5 s (laboratory columns) | — |
| 71 | 103 | Offline identification is "sufficient only" for robots whose payload does not change; carrying … | C1-S33, section 2 | Verified | §2: offline identification "is sufficient only in the case of robots that will not be expanded ... or will not transport loads" | — |
| 72 | 104 | Commercial mobile robots are usually commanded with linear and angular velocity, and a dynamic … | C1-S33, section 3.1 | Verified | §3.1: "A much better solution is to use a dynamics model that takes the linear and angular velocities ... This is how commercially available mobile robots are usually controlled" | — |
| 73 | 105 | General identification theory rests on a few concepts: a model, the information in the data, es… | C1-S43, section 2 "The Core", report pp. 3–4 | Verified | Report pp. 3–4 §2: concepts Model, Information, Estimation, Validation; "(1) ... good agreement with the estimation data. (2) The model should not be too complex" | — |
| 74 | 106 | Identifying a system from data recorded while a feedback controller is running is a special pro… | C1-S44, section 1, report pp. 2–3 | Verified | Report p. 2–3 §1: "correlation between the un-measurable noise and the input ... estimate will typically be biased"; "spectral analysis, instrumental variable methods and the subspace methods give erroneous results when applied directly to closed-loop data" | — |
| 75 | 107 | Closed-loop experiments are still used when the plant is unstable, must stay under control for … | C1-S44, section 1, report p. 4 | Verified | Report p. 4: "identification for control"; "the plant is unstable, or that it has to be controlled for production economic or safety reasons" | — |

### Findings: Disturbance observers: robust speed control under model–plant mismatch (general)

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 76 | 110 | A disturbance observer (DOb, proposed by Ohnishi in 1983) lumps plant uncertainty (e.g. wrong i… | C1-S45, abstract, p. 1; section IV-A, eq. 1, p. 5 | Verified | Abstract p. 1 "proposed by K. Ohnishi in 1983"; p. 2 "fictitious disturbance"; §IV-A p. 5 eq. 1: "if the bandwidth of DOb is large enough ... performance controller can be designed by considering only the nominal plant dynamics" | — |
| 77 | 111 | The observer bandwidth and filter order are limited by measurement noise (which depends on enco… | C1-S45, section IV-A, p. 5 | Verified | §IV-A p. 5: bandwidth and order "limited by ... noise and the waterbed effect, respectively ... resolution of an encoder and sampling time ... RHP zeros and poles"; "becomes more noise sensitive" | — |
| 78 | 112 | DOb-based control gives a two-degree-of-freedom structure: Ohnishi's 1985 work showed "theoreti… | C1-S45, section II-B, p. 2 | Verified | p. 2: Ohnishi 1985 "theoretically and experimentally proven that the robustness and performance ... can be independently adjusted" | — |
| 79 | 113 | Commercial motor drives embed disturbance observers, e.g. Panasonic MINAS-A5 drivers use one "t… | C1-S45, section III-C, p. 4 | Verified | §III-C p. 4: "DOb was embedded in the Panasonic's MINAS-A5 ... torque, reduce vibration, and offset any speed decline" | — |
| 80 | 114 | Tuning of the nominal inertia is still "an open problem": experiments show robust stability and… | C1-S45, section VI, p. 9 | Verified | §VI p. 9: "tuning the parameters of the nominal inertia matrix is still an open problem"; "may significantly deteriorate when the nominal inertia matrix changes"; "generally tuned by trial and error" | — |

### Findings: Friction modelling and compensation (general)

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 81 | 117 | The simplest static friction model is Coulomb plus viscous, F(v) = Fc·sgn(v) + β·v; stiction ad… | C1-S31, section II | Verified | §II: Coulomb + viscous static model, stiction breakaway, Stribeck | — |
| 82 | 118 | Poor or absent friction compensation leads to tracking errors (especially at low velocity), sti… | C1-S31, section I | Verified | §I: tracking errors, stick-slip, hunting, limit cycles; "friction can cause 50% error in some heavy industrial manipulators" | — |
| 83 | 119 | Identifying a dynamic friction model such as LuGre is hard: its internal state cannot be measur… | C1-S31, section II | Verified | §II: dynamic models hard to identify (unmeasurable state, sensitivity, high-precision sensors) | — |
| 84 | 120 | Compensation methods fall into four classes: (A) fixed model-based terms identified offline, (B… | C1-S31, section III | Verified | §III classes A–D | — |
| 85 | 121 | Fixed (class A) compensation is the cheapest to run but requires accurate offline identificatio… | C1-S31, section III | Verified | §III: fixed compensation "cannot, obviously, account for friction variations" | — |
| 86 | 122 | A friction model with zero force at zero velocity cannot compensate static friction; that part … | C1-S31, section III-A | Verified | §III-A: static friction then compensated by integral action | — |
| 87 | 123 | A large integral gain that would destabilise the loop during sliding can be used in the final s… | C1-S31, section III-D | Verified | §III-D: "a large integral action, that could not be applied in the" sliding phase | — |
| 88 | 124 | In a two-joint direct-drive arm, a static LuGre-based compensation gave 0.5 mm RMS error at low… | C1-S31, section IV | Verified | §IV: LuGre "rms value passes from 0.5 mm to 3.5 mm"; polynomial "1.8 mm in the LV test and 2.9 mm" | — |
| 89 | 125 | The reviewers conclude that no compensation method is "intrinsically superior"; the choice depe… | C1-S31, section IV | Verified | §IV p. 4365: "no method among the reviewed ones can be considered as intrinsically superior" | — |
| 90 | 126 | On a small geared DC motor, a two-parameter Coulomb–viscous model explained more than 98% of th… | C1-S32, section 4.1.4, Table 5 | Verified | §4.1.4 Table 5: Coulomb–viscous RMSE 0.29, R² 0.98; Stribeck 0.18; LuGre 0.20; "more than 98% of the variance"; "(/ω/>5rad/s), all models exhibit nearly identical behavior" | — |
| 91 | 127 | Temperature drift of ±1 °C was estimated to contribute about 1.5% to friction-coefficient varia… | C1-S32, section 4.1.9 | Verified | §4.1.9: "Temperature drift of ±1 °C ... contributes approximately 1.5%" | — |
| 92 | 128 | The LuGre model represents friction as the average deflection z of elastic "bristles" between t… | C1-S40, section II, eqs. 1–4, p. 420 | Verified | §II p. 420 eqs. 1–4; "characterized by six parameters σ0, σ1, σ2, FC, FS, and vs"; g "need not be symmetrical. Direction dependent behavior can therefore be captured" | — |
| 93 | 129 | Its authors state that the model captures "the Stribeck effect, hysteresis, spring-like charact… | C1-S40, abstract, p. 419; section IV-B, Fig. 3, p. 422 | Verified | Abstract p. 419 (quote matches); §IV-B p. 422 Fig. 3 "the highest frequency shows the widest hysteresis loop" | — |
| 94 | 130 | Friction "may give rise to limit cycles in servo drives where the controller has integral actio… | C1-S40, section V-A, Figs. 7–8, p. 423 | Verified | §V-A p. 423: "friction may give rise to limit cycles in servo drives where the controller has integral action ... hunting"; Figs. 7–8 "clearly predicts limit cycles" | — |
| 95 | 131 | An observer-based friction compensator (estimating the unmeasurable bristle state) is also give… | C1-S40, section V-B, eq. 15 and Theorem 2, p. 424 | Verified | §V-B p. 424 eq. 15, Theorem 2; "To assume that the friction model and its parameters are known exactly is of course a strong assumption ... The accuracy required in the velocity measurement is a similar problem" | — |

### Findings: Actuator limits outside FRC: thermal current limiting and supply-voltage compensation (general)

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 96 | 134 | A common thermal model of a motor has two heat capacities (winding "core" and housing) and two … | C1-S46, section II-A, eqs. 1–3, p. 2 | Partly supported | §II-A p. 2 eqs. 1–3 (C1, C2, R1, R2; Q = Re·i²); "applicable to various classical motors such as brushless and brushed DC motors" | The source calls it its "Basic Thermal Model"; "a common thermal model" is an uncited characterization. Reword to "a two-node thermal model (Kawaharazuka et al.)". |
| 97 | 135 | Real thermal parameters differ from datasheet values, attributed to "heat dissipation to the at… | C1-S46, section II-A, p. 2, Fig. 3 | Verified | §II-A p. 2: "heat dissipation to the attached metal parts, error of the ambient temperature, deterioration or burnout"; new vs "old motor used for over half a year ... different by about 15 °C in 90 seconds"; "thermal model should always be updated" | — |
| 98 | 136 | Earlier software methods that cap current or torque at a fixed value that is safe for a set tim… | C1-S46, section I, p. 1 | Verified | §I p. 1: [6] "considers an extremely short amount of time"; [7] "a value lower than the actual possible value" | — |
| 99 | 137 | maxon's drives (ESCON, EPOS) protect the winding with an "I2t" algorithm that measures motor cu… | C1-S47, section 2 | Verified | §2: "I2t algorithm measures the motor current during each current control cycle (EPOS4: 40us)"; based on "Nominal current" and "Thermal time constant winding"; limited to nominal, "no(!) abrupt shutdown" | — |
| 100 | 138 | With that algorithm a current of twice nominal is limited to nominal after 0.3 thermal time con… | C1-S47, section 2, "Example" | Verified | §2 Example: "twice ... after 0.3 times", "four times ... after 0.1 times"; overcurrent time "automatically adapted"; "Cyclic aspects ... properly taken into account" | — |
| 101 | 139 | maxon contrasts this with the fixed-overload model of AC frequency inverters ("110% or 160%", s… | C1-S47, section 1 | Verified | §1: "overload ratio of 110% or 160%. The overload time period might be configurable"; motors "often operated many times above the specified nominal data for a short period" | — |
| 102 | 140 | A typical "Maximum output current" is 2–3 times the nominal current, and it must also respect t… | C1-S47, section 3 | Verified | §3: "rule of thumb ... 2-3 times of the motor's Nominal current"; must not exceed "mechanical components (e.g. gearboxes" | — |
| 103 | 141 | Supply (bus) voltage compensation outside FRC: a Freescale controller for mains-fed three-phase… | C1-S48, section 2.2 ("Dynamic Bus Ripple Cancellation"), p. 6; section 3.10, eqs. 16–20, pp. 46–47 | Verified | p. 6 §2.2 feature list "Dynamic Bus Ripple Cancellation"; §3.10 pp. 46–47 eqs. 16–20, Vnorm/Vbus(t); "correction should be applied only to the terms containing the modulation index ... leaving the bias term untouched"; clipping at low VBus | — |
| 104 | 142 | In that controller a new bus-voltage reading is taken every PWM interrupt, and the note states … | C1-S48, section 2.2, p. 6; section 3.10, p. 47 | Verified | p. 47: "A new VBus reading is taken every time the PWM ISR is executed"; p. 6: "compensation for line frequency ripple, as well as slower bus voltage changes resulting from regeneration or brown out" | — |
| 105 | 143 | On a skid-steered wheeled robot, a trajectory can be impossible even though total vehicle power… | C1-S51, section 5.2, pp. 310–311 | Verified | §5.2 pp. 310–311: "51 w ... 102 w"; outer motor "approximately 58 w ... cannot be achieved"; total "well below the 102 w"; Fig. 18 lab vinyl, 0.7 m/s, 1.2 m | — |

### Findings: Wheel and track velocity control on other ground vehicles (general)

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 106 | 146 | Before its traction-control update, the Curiosity Mars rover turned each wheel at a constant sp… | C1-S34, section 1, p. 700 | Verified | §1 p. 700: constant wheel speed assuming flat terrain; slip when climbing rocks | — |
| 107 | 147 | The update ("TRCTL") instead commands "idealized, no-slip wheel angular rates" computed from a … | C1-S34, abstract and section 1, p. 700 | Verified | Abstract/§1 p. 700: "idealized, no-slip wheel angular rates"; no force sensors; VO slip ~once per metre | — |
| 108 | 148 | Wheel speeds are capped, so to speed up the wheel crossing an obstacle the other wheels are slo… | C1-S34, section 1, p. 700 | Verified | §1 p. 700: wheel speeds capped; other wheels slowed | — |
| 109 | 149 | The wheel-rate function runs at 8 Hz inside the flight software. | C1-S34, section 4.1 | Verified | §4.1: "evaluate drive wheel rates at 8 Hz" | — |
| 110 | 150 | Results: ground tests showed wheel forces reduced by 19% (leading wheels) and 11% (middle leadi… | C1-S34, abstract and section 4.2 | Verified | Abstract: "reduced by 19% for leading wheels and 11% for middle leading"; "3.6 km and 149 drives ... wheel current, correlated with wheel torque, of 18.7%"; "up to 25%" vs "only a 10% increase"; §4.2 "less than 3% to the total CPU usage" | — |
| 111 | 151 | In the agricultural-robot study each drive motor ran a discrete PI speed controller with sampli… | C1-S37, sections 3.2 and 4.1, pp. 903–904 | Verified | §3.2/§4.1 pp. 903–904: "sample period ... Ts = 150 ms"; "fine-tuning from Ziegler–Nichols rule initial tuning" | — |
| 112 | 152 | The dynamic model that Siwek et al. identify for the TurtleBot 2 AGV assumes a low-level PD con… | C1-S33, Appendix A | Verified | Appendix A: "equations of the PD controller implemented to regulate the supply voltage of motors [60]" | — |
| 113 | 153 | On a tracked field robot driving in wet, bumpy sorghum fields, the track motor speeds were held… | C1-S49, abstract, p. 1; section 2.1, p. 4 | Verified | Abstract p. 1 (wet, bumpy field); §2.1 p. 4: Kangaroo x2 "two channel self-tuning PID controller" at "50-Hz"; sampling "5-Hz ... maximum update rate of the GNSS"; encoders "accuracy of 0.035 m/s" | — |
| 114 | 154 | That robot's model scales commanded speed and yaw rate by two traction parameters μ and κ betwe… | C1-S49, section 2.2, p. 5; section 5, p. 15; abstract | Verified | §2.2 pp. 5–6 eq. 2: µν, κω, "between zero and one", slips "1 − µ and 1 − κ", estimated each iteration; §5 p. 15 "performance of the low-level controller ... is sufficient"; abstract 0.0423 m | — |
| 115 | 155 | A tracked skid-steer robot (tests: straight line at a constant 0.75 m/s, then turning at 45 deg… | C1-S50, section 4.1, p. 7; section 4.2, Table 2, p. 8 | Verified | §4.1 p. 7: "straight line at constant speed of 0.75 m/s ... 45 deg/s"; Table 2 p. 8: sand 26.17, gravel 21.77, mud 24.01, asphalt 21.23 A | — |
| 116 | 156 | On that robot the currents showed wide and narrow peaks on asphalt and gravel but an almost fla… | C1-S50, section 4.2, p. 8 | Verified | §4.2 p. 8: "wide and narrow current peaks over the asphalt and gravel and almost a flat trend on mud and sand"; "larger contact area"; "offset ... due to different intrinsic motor characteristics and to power dissipation" | — |
| 117 | 157 | The same paper uses motor current as a proxy for traction effort, since tractive effort, thrust… | C1-S50, section 3.3, eq. 18, p. 6 | Verified | §3.3 p. 6 eq. 18: "tractive effort, the thrust, and the torque can be considered as roughly proportional to the DC motor current" | — |
| 118 | 158 | For their skid-steered test vehicle (a modified Pioneer 3-AT), Yu et al. replaced the manufactu… | C1-S51, section 5, p. 309 | Verified | §5 p. 309: "nontransparent, speed controller ... replaced by a PID controller and motor controller"; "control sampling rate of 1KHz"; "Two current sensors" | — |

### Findings: How the controller-side velocity loop works

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 119 | 164 | A closed-loop controller uses sensor feedback to reduce the difference ("error") between the se… | C1-S01, section "Closed-Loop Control Basics" | Partly supported | "Closed-Loop Control Basics": "a process that uses feedback to improve the accuracy of its outputs" | The page does not mention "error" or the setpoint–measurement difference. Quote it as "uses feedback to improve the accuracy of its outputs". |
| 120 | 165 | The SPARK MAX and SPARK Flex run their PID loop on the motor controller and update it every 1 m… | C1-S01, section "Closed-Loop Control with SPARK Motor Controllers" | Verified | "the loop is updated every 1ms"; "Both the SPARK MAX and SPARK Flex ..." | — |
| 121 | 166 | The SPARK loop is a standard PID algorithm with several feedforward terms added; an optional "a… | C1-S01, section "Closed-Loop Control with SPARK Motor Controllers" | Verified | "follows a standard PID algorithm and incorporates several feedforward terms"; arbitrary FF "added to the output ... after all calculations ... either voltage or duty cycle" | — |
| 122 | 167 | In Velocity Control mode the setpoint is a speed in RPM, or in the units set by the velocity co… | C1-S05, "Velocity Control Mode" | Verified | "run the motor at a set speed in RPM (or configured conversion factor units)" | — |
| 123 | 168 | REV notes that velocity-loop constants "are often of a very low magnitude" and suggests decreas… | C1-S05, hint box | Verified | hint: "Velocity Loop constants are often of a very low magnitude ... try decreasing your gains" | — |
| 124 | 169 | Each SPARK has 4 closed-loop gain "slots" (numbered 0–3), each with its own set of constants; t… | C1-S07, section "Slots" | Verified | "Slots": "4 closed-loop slots, each with their own set of constants ... numbered 0-3" | — |
| 125 | 170 | P multiplies the error and "does the heavy lifting"; I accumulates error over time to remove st… | C1-S04, section "The Constants" | Verified | "The Constants": P "does the heavy lifting"; I accumulates error; D "resists motion" and dampens oscillation | — |
| 126 | 171 | The permanent-magnet DC motor feedforward ("voltage balance") equation is V = kS·sgn(ḋ) + kV·ḋ … | C1-S15, section "The Permanent-Magnet DC Motor Feedforward Equation" | Verified | eq. V = Ks·sgn(ḋ) + Kv·ḋ + Ka·d̈ | — |
| 127 | 172 | kS is the voltage needed to overcome static friction (applied in the direction of motion); kV i… | C1-S15, same section | Verified | kS static friction (signum); kV "hold (or cruise) ... counter-electromotive force ... friction that increases with speed"; kA per acceleration | — |
| 128 | 173 | The voltage–speed and voltage–acceleration relationships are "almost entirely linear" for perma… | C1-S15, same section | Verified | "almost entirely linear (for FRC-legal components)" | — |
| 129 | 174 | For velocity control, the feedforward velocity comes directly from the setpoint, and accelerati… | C1-S15, section "Using the Feedforward" | Verified | "Using the Feedforward": velocity from setpoint; acceleration "can often be omitted"; from difference of setpoints | — |
| 130 | 175 | A pure-feedback (PID-only) velocity controller is flawed because non-zero effort is needed just… | C1-S16, section "Issues with Feedback Control Alone" | Verified | "a non-zero amount of control effort is required to keep the flywheel spinning ... this feedback-only strategy is flawed" | — |
| 131 | 176 | A pure feedforward velocity controller "works reasonably well" but cannot reject disturbances. | C1-S16, section "Pure Feedforward Control" | Verified | "works reasonably well ... cannot reject disturbances" | — |
| 132 | 177 | kD "is not useful for velocity control with a constant setpoint - it is only necessary when the… | C1-S16, section "Choice of Control Strategies" | Verified | quote exact, "Choice of Control Strategies" | — |
| 133 | 178 | For an ideal first-order motor/flywheel model a P controller alone can place the single closed-… | C1-S27, section 6.10.7 "Do flywheels need PD control?", pp. 71–73 | Verified | §6.10.7 pp. 71–73: "one P controller to place that pole anywhere on the real axis. A derivative term is unnecessary on an ideal flywheel" | — |
| 134 | 179 | In a velocity loop, overshoot "typically occurs ... only as a result of loop delay". | C1-S16, section "Velocity and Position Control" | Verified | "overshoot typically occurs in velocity controllers only as a result of loop delay" | — |
| 135 | 180 | CTRE Phoenix 6 (a comparable smart controller) documents velocity-loop gains as: kS = output to… | C1-S22, `ctre_2026_phoenix6_basic_pid_control.md`, section "Velocity Control" | Verified | Velocity Control: Ks "output to overcome static friction", Kv "output/rps", Ka "unused"; kP/kI/kD units in same list | — |

### Findings: SPARK MAX gain units and the firmware 25/26 changes

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 136 | 183 | REVLib 2025 requires SPARK firmware v25.0.0 or newer. | C1-S09, line 343 | Verified | L343 (revlib-2025.0.0): "Requires non-prerelease versions of SPARK and Servo Hub firmware v25.0.0 or higher" | — |
| 137 | 184 | REVLib 2026.0.0 added feedforward parameters "`kV` (formerly `kF`), `kA`, `kS`, `kG`, `kCos`, a… | C1-S09, line 256 | Verified | L256 (revlib-2026.0.0): quote exact | — |
| 138 | 185 | SPARK MAX firmware 26.1.0 "Adds expanded feedforward support to all PID modes", reworks MAXMoti… | C1-S09, lines 65–79 | Verified | L65–79 (sm-26.1.0): "Adds expanded feedforward support to all PID modes", "Reworks MAXMotion", "Removes SmartMotion and SmartVelocity" | — |
| 139 | 186 | SPARK MAX firmware 26.1.0 "Fixes the I PID component not limiting in the negative direction whe… | C1-S09, line 68 | Verified | L68: quote exact | — |
| 140 | 187 | Default units on current REV docs: velocity in RPM, applied output in duty cycle, kS in volts, … | C1-S02, table "Default Units" | Verified | Default Units table: RPM, Duty Cycle, kS Volts, kV Volts per RPM, kA Volts per RPM/s | — |
| 141 | 188 | The same units table gives kP as "Duty cycle per rotation", kI as "Duty cycle per (rotation*ms)… | C1-S02, table "Default Units" | Partly supported | Table: kP "Duty cycle per rotation", kI "Duty cycle per (rotation*ms)", kD "(Duty cycle*ms) per rotation", "Affected by: Position Conversion Factor" | The table does not say it is for position control; "i.e. the table is written for position control" is an inference. Label it: "(the per-rotation units suggest the position form)". |
| 142 | 189 | kV and kA scale with the velocity conversion factor; the position and velocity conversion facto… | C1-S02, table "Default Units" and section "Velocity Conversion Factor" | Verified | kV/kA "Affected by Velocity Conversion Factor"; "velocity conversion factor is completely independent ... both need to be set" | — |
| 143 | 190 | REV states kV units are "Volts per velocity as measured by the feedback sensor, after the conve… | C1-S03, section "kV - Velocity Gain" | Verified | "kV - Velocity Gain": quote exact | — |
| 144 | 191 | In REVLib 2026 the Java doc for `kV()` reads "The kV gain in Volts per velocity"; for `kA()` "V… | C1-S10, `FeedForwardConfig.java` lines 65, 81, 97 | Verified | FeedForwardConfig.java L65 "kS gain in Volts", L81 "kV gain in Volts per velocity", L97 "kA gain in Volts per velocity per second" | — |
| 145 | 192 | kV is "not applied in Position control mode" and kA "is only applied in MAXMotion control modes… | C1-S10, `FeedForwardConfig.java` lines 76, 92; C1-S03, section "Feed Forward Constant Terms" | Verified | L76 "not applied in Position control mode"; L92 "only applied in MAXMotion control modes"; S03 chart: Velocity Control kS true, kV true, kA false | — |
| 146 | 193 | In REVLib 2026 the old `velocityFF()` and `pidf()` methods are deprecated "for removal" and wri… | C1-S10, `ClosedLoopConfig.java` lines 104, 245–260; `SparkParameters.java` line 52 | Verified | ClosedLoopConfig.java L104 `@Deprecated(forRemoval = true)` pidf; L245–260 velocityFF → `kV_0`; SparkParameters.java L52 `kV_0(16, Type.FLOAT)` | — |
| 147 | 194 | In REVLib 2025.0.3, `velocityFF()` wrote to a parameter named `kF_0`, also ID 16. | C1-S10, `rev_2025_revlib_ClosedLoopConfig.java` line 280; `rev_2025_revlib_SparkParameters.java` line 49 | Verified | 2025 ClosedLoopConfig L280 `SparkParameter.kF_0`; 2025 SparkParameters L49 `kF_0(16)` | — |
| 148 | 195 | The SPARK MAX parameter reference page lists ID 16 as "kF_0", "Feed Forward gain constant for g… | C1-S08, row kF_0 | Verified | Row ID 16: "kF_0 ... Feed Forward gain constant for gain slot 0", no unit | — |
| 149 | 196 | REV's 2025 velocity example used `.velocityFF(1.0 / 5767)`; the 2026 version of the same exampl… | C1-S11, 2025 closed-loop example line 72; 2026 closed-loop example lines 74–75 | Verified | 2025 L72 `.velocityFF(1.0 / 5767, ...)`; 2026 L74–75 comment + `.kV(12.0 / 5767, ...)` | — |
| 150 | 197 | REV's MAXMotion velocity tip says decent performance is possible "with only kV/kA and no PID at… | C1-S06, section "Tips for Smooth Motions" | Verified | "Tips for Smooth Motions": "decent performance with only kV/kA and no PID at all"; "If the underlying velocity PID outruns the acceleration target, the motion may seem jittery" | — |
| 151 | 198 | MAXMotion Velocity mode honours the maximum acceleration but not the cruise velocity, so any to… | C1-S07, section "MAXMotion Parameters" | Verified | S07 L174 cruise velocity "does not honor it ... top-speed clamping ... before you send the setpoint"; S06 "Honoring the maximum acceleration" | — |

### Findings: Measuring feedforward (system identification)

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 152 | 201 | System identification fits a model to measured input–output data; real data contain measurement… | C1-S18, `introduction.rst`, "What is System Identification?" | Verified | introduction.rst: "measurement noise (e.g. timing errors, encoder resolution limitations) and system noise (unmodeled forces ... like vibrations)" | — |
| 153 | 202 | The SysId "simple motor" fit is V = kS·sgn(ḋ) + kV·ḋ + kA·d̈. | C1-S18, `introduction.rst`, "Simple Motor Identification" | Verified | introduction.rst "Simple Motor Identification" equation | — |
| 154 | 203 | A standard routine has a quasistatic test (slow voltage ramp so the acceleration term is neglig… | C1-S18, `creating-routine.rst`, "Types of Tests" | Verified | creating-routine.rst: quasistatic/dynamic; "run both forwards and backwards, for four tests in total" | — |
| 155 | 204 | Default test settings are a ramp of 1 volt per second and a step of 7 volts, with a 10 s safety… | C1-S18, `creating-routine.rst`, "Routine Config" | Verified | creating-routine.rst L22 "1 volt per second ... 7 volts"; L26 timeout "10 seconds" | — |
| 156 | 205 | The drive identification needs at least 10 ft of space, ideally close to 20 ft, and "can not be… | C1-S18, `running-routine.rst`, note | Verified | running-routine.rst L7 "at least 10' of space, ideally closer to 20' ... can not be accurately characterized while on blocks" | — |
| 157 | 206 | In WPILib's example drivetrain routine the same voltage is sent to both sides, but the left and… | C1-S20, `Drive.java` lines 54–74 | Verified | Drive.java L54–74: same `voltage` to leftMotor/rightMotor; `log.motor("drive-left")`, `log.motor("drive-right")` | — |
| 158 | 207 | SysId analyses one motor's position, velocity and voltage entries at a time, chosen by the moto… | C1-S18, `loading-data.rst` | Verified | loading-data.rst: entries "containing the motor name you set in the log callback" | — |
| 159 | 208 | SysId regresses acceleration on velocity, voltage and sgn(velocity) (ordinary least squares) an… | C1-S20, `FeedforwardAnalysis.cpp` lines 25–41, 223–240 | Verified | FeedforwardAnalysis.cpp L25–41 "regress acceleration"; L223–240 Ks = −γ/β, Kv = −α/β, Ka = 1/β | — |
| 160 | 209 | SysId's analysis code has "Drivetrain" and "DrivetrainAngular" model types that use the same fi… | C1-S20, `FeedforwardAnalysis.cpp` lines 25–30 | Verified | L27: "Simple, Drivetrain, DrivetrainAngular:" same equation | — |
| 161 | 210 | A good dataset needs "at least 2 steady-state velocity events to separate Ks from Kv" and "at l… | C1-S20, `FeedforwardAnalysis.cpp` lines 166–171 | Verified | L166–171 quotes exact | — |
| 162 | 211 | Fit-quality guides: acceleration r² rarely exceeds 0.5 even on good data and below about 0.2 kA… | C1-S18, `viewing-diagnostics.rst`, "Goodness-of-Fit Metrics" | Verified | viewing-diagnostics.rst: "rarely goes above 0.5"; "below around 0.2, the kA gain will be of dubious quality"; sim r² "north of .9" | — |
| 163 | 212 | A successful quasistatic plot is nearly linear and a successful dynamic plot is an approximatel… | C1-S18, `viewing-diagnostics.rst`, "Time-Domain Plots" | Verified | "very nearly linear ... approximately exponential approach of the steady-speed" | — |
| 164 | 213 | A velocity threshold set too low lets pre-motion samples into the fit (a "leading tail"); too h… | C1-S18, `viewing-diagnostics.rst`, "Improperly Set Motion Threshold" | Verified | "leading tail" = threshold too low; too high → "gap" in acceleration-versus-velocity plot | — |
| 165 | 214 | If the mechanism is light relative to motor power, kA tends toward zero and can be ignored for … | C1-S18, `viewing-diagnostics.rst`, "Acceleration-Velocity Plot" | Verified | "kA will tend towards zero and can be ignored ... calculated feedback gains are likely to be inaccurate" | — |
| 166 | 215 | kS "is nearly impossible to model, and must be measured empirically"; kV, kA and kG can be esti… | C1-S19, section "The WPILib Feedforward Classes" | Verified | feedforward.rst: "kS is nearly impossible to model, and must be measured empirically"; kG, kV, kA from computation / ReCalc | — |
| 167 | 216 | WPILib gain units are kS in volts, kV in volts·seconds/distance and kA in volts·seconds²/distan… | C1-S19, note under "SimpleMotorFeedforward" | Verified | SimpleMotorFeedforward note: volts, volts * seconds / distance, volts * seconds^2 / distance | — |
| 168 | 217 | Manual kS measurement: slowly increase voltage until the mechanism starts to move; kS is the la… | C1-S16, section "Feedforward Simplifications" | Verified | "Feedforward Simplifications": "slowly increase the voltage ... Ks is the largest voltage applied before the mechanism begins to move" | — |
| 169 | 218 | REV's manual kS method: find the smallest output that makes the mechanism move, then reduce it … | C1-S03, section "kS - Static Gain" | Verified | "kS - Static Gain": "find the smallest output that causes the mechanism to move slightly, then decrease it slightly" | — |
| 170 | 219 | For a mechanism with a constant load (REV's elevator example), measure the upper and lower volt… | C1-S03, section "kG - Static (Elevator) Gravity Gain" | Verified | kG section: V1/V2 edges of holding region; "kS = (V1 - V2) / 2", "kG = V2 + kS" | — |
| 171 | 220 | A drivetrain can be modelled with separate linear and angular gains (Kv,lin, Ka,lin, Kv,ang, Ka… | C1-S27, section 14.3 "Drivetrain left/right velocity state-space model" | Verified | §14.3 Theorem 14.3.1; "If Kv and Ka are the same for both the linear and angular cases ... left and right sides are decoupled" | — |
| 172 | 221 | The SysId "Spark Max" preset assumes the controller is already set to the analysis units throug… | C1-S18, `analyzing-gains.rst`, note in "Enter Controller Parameters" | Verified | analyzing-gains.rst L36 note: "Spark Max preset assumes ... units of analysis with the SPARK MAX API's position/velocity scaling factor" | — |

### Findings: Tuning order and what limits the feedback gains

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 173 | 224 | REV's order: set all constants to 0; check motor direction; set up feedforwards; set P very sma… | C1-S04, section "Tuning", steps 1–10 | Verified | "Tuning" steps 1–10 match | — |
| 174 | 225 | CTRE's manual order: all gains zero; kS "until just before the motor moves"; kV until output ve… | C1-S22, `ctre_2026_phoenix6_closed_loop_requests.md`, "Closed-Loop Overview" | Verified | Closed-Loop Overview: "Increase Ks until just before the motor moves", Kv "closely matches", Kp "until the output starts to oscillate", Kd "as much as possible without introducing jittering" | — |
| 175 | 226 | WPILib's flywheel tutorial: tune the feedforward first, then the PID; "PID portion of the contr… | C1-S16, section "Combined Feedforward and Feedback Control" | Verified | "Combined Feedforward and Feedback Control" quote exact | — |
| 176 | 227 | REV: "I ... is not often recommended in FRC"; feedforward is recommended instead to remove stea… | C1-S04, section "I - Integral Gain" | Verified | "I - Integral Gain": quote exact; "use a limited to prevent I windup"; "Feedforward gains are recommended" | — |
| 177 | 228 | WPILib: "systems that seem to require integral control to respond well probably have an inaccur… | C1-S17, "Integral Term Windup" | Verified | "Integral Term Windup" important box: quote exact | — |
| 178 | 229 | Integrator windup: when the actuator saturates the loop is effectively open, the integral keeps… | C1-S24, section 10.4, pp. 306–308 | Verified | §10.4 pp. 306–308: "feedback loop is broken ... integral term will also build up ... large transients"; anti-windup in same section | — |
| 179 | 230 | Windup mitigations named by WPILib: lower kI, reset the integrator when far from the setpoint (… | C1-S17, "Integral Term Windup" | Verified | "Integral Term Windup" items 1–3 | — |
| 180 | 231 | On the SPARK, `iZone` limits where the integrator accumulates and `iMaxAccum` caps the I accumu… | C1-S08, row kIZone_0; C1-S10, `ClosedLoopConfig.java` lines 393–402 | Verified | Row kIZone_0: "integrator will only accumulate while the setpoint is within IZone"; ClosedLoopConfig L393–402 iMaxAccum "constrain the I accumulator" | — |
| 181 | 232 | Delay limits gain: a time delay τ behaves like a right-half-plane zero at 2/τ, and such a zero … | C1-S24, section 11.5, pp. 332–333, eq. 11.16 and Padé approximation | Verified | §11.5 pp. 332–333: eq. 11.16, "With ϕl = π/3 we get ωgc < 0.6 z"; Padé "slow right half-plane zero z = 2/τ" | — |
| 182 | 233 | Aggressive (LQR) gains can become unstable when sensor measurements are delayed too long. | C1-S27, section B.5 "Latency compensation" | Verified | §B.5: "If sensor measurements are delayed too long, the LQR may be unstable" | — |
| 183 | 234 | Smart motor controllers "apply substantial low-pass filtering to their encoder velocity measure… | C1-S18, `analyzing-gains.rst`, "Measurement Delays" | Verified | "Measurement Delays": quotes exact | — |
| 184 | 235 | Delay of a moving-average filter with N taps and sample period T is (N − 1)·T/2; delay of a bac… | C1-S18, `analyzing-gains.rst`, "Measurement Delays"; C1-S27, Theorems B.5.1–B.5.2; C1-S20, `FeedbackControllerPreset.hpp` lines 79–107 | Verified | analyzing-gains.rst d = T(n − 1)/2; Theorems B.5.1–B.5.2; .hpp L79–107 "(N - 1)/2 T", "average delay is T / 2" | — |
| 185 | 236 | SysId presets: REV NEO built-in sensor 112 ms delay; REV non-NEO (quadrature) 81.5 ms; CTRE Pho… | C1-S20, `FeedbackControllerPreset.hpp` lines 109–152 | Verified | .hpp L109–152: REV_NEO_BUILT_IN 112_ms, REV_NON_NEO 81.5_ms, CTRE_V5 81.5_ms, CTRE_V6 1_ms "Kalman filters default-tuned to lowest latency" | — |
| 186 | 237 | SysId's Controller Period field should be 1 ms for most "smart controllers", because their onbo… | C1-S18, `analyzing-gains.rst`, "Controller Period"; C1-S20, `FeedbackControllerPreset.hpp` lines 119–158 | Verified | analyzing-gains.rst "run at 1Khz, or a period of 0.001s"; presets use `1_ms` | — |
| 187 | 238 | SysId feedback gains are "educated guesses" and "a starting point for further tuning". | C1-S18, `analyzing-gains.rst`, "Feedback Analysis" | Verified | analyzing-gains.rst L25 quotes exact | — |
| 188 | 239 | The "Max Acceptable Control Effort" in SysId should never exceed 12 V and ideally be lower; sma… | C1-S18, `analyzing-gains.rst`, "Specify Optimality Criteria" | Verified | "should never exceed 12V ... ideally ... somewhat lower"; "smaller values for Max Acceptable Error and larger values for Max Acceptable Control Effort will result in larger gains" | — |
| 189 | 240 | If gains ask for more than the actuator can deliver, the mechanism saturates and "behave[s] as … | C1-S17, "Actuator Saturation" | Verified | "Actuator Saturation": quote exact | — |
| 190 | 241 | Derivative action amplifies high-frequency noise; a filtered derivative kd·s/(1 + s·Tf) with Tf… | C1-S24, section 10.5 "Filtering the Derivative", p. 308 | Verified | §10.5 p. 308: "replacing the term kd s by kd s/(1 + sTf) ... Tf = (kd/kp)/N, with N in the range 2–20" | — |
| 191 | 242 | The SPARK exposes a per-slot derivative filter parameter (`kDFilter`). | C1-S08, row kDFilter_0; C1-S10, `ClosedLoopConfig.java`, `dFilter()` | Verified | Row kDFilter_0 "PIDF derivative filter constant for gain slot 0"; ClosedLoopConfig `dFilter()` L268–273 | — |
| 192 | 243 | REV MAXMotion tip: a loop tuned for high speed can look "wobbly" at low speed and vice versa; t… | C1-S06, "Tips for Smooth Motions" | Verified | "Tips for Smooth Motions": "wobbly or inconsistent if the loop has been tuned for higher speeds or vice versa ... separate PIDs and switching between slots" | — |

### Findings: Velocity measurement filtering

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 193 | 246 | NEO hall sensor (REVLib 2026): `uvwMeasurementPeriod` is in ms, range [8, 64], default 32 ms; `… | C1-S10, `EncoderConfig.java` lines 146–176 | Verified | EncoderConfig.java: uvwMeasurementPeriod "milliseconds ... range [8, 64]. The default value is 32ms"; uvwAverageDepth "1, 2, 4, or 8 (default)" | — |
| 194 | 247 | Quadrature encoder (REVLib 2026): `quadratureMeasurementPeriod` range [1, 100] ms, default 100 … | C1-S10, `EncoderConfig.java` lines 118–136 | Verified | L118–136: quadratureMeasurementPeriod "[1, 100] ... default ... 100ms"; quadratureAverageDepth "[1, 64] ... default ... 64" | — |
| 195 | 248 | The SPARK MAX parameter table describes the quadrature velocity as the difference between the c… | C1-S08, rows kEncoderAverageDepth and kEncoderSampleDelta | Verified | Rows kEncoderSampleDelta ("current sample, and the sample x * 500μs behind") and kEncoderAverageDepth ("between 1 and 64") | — |
| 196 | 249 | Per REV support (quoted in a SysId issue), the NEO hall sensor "is sampled every 32ms, and the … | C1-S21 | Verified | Issue comment: "sampled every 32ms, and the sampling window is 8 samples ... does not use a backward finite difference ... (8 - 1)/2 * 32 = 112ms" | — |
| 197 | 250 | Hall-sensor velocity measurement settings were added in SPARK MAX firmware 1.6.0. | C1-S09, line 709 | Verified | L709 (sparkmax-1.6.1 notes, "Version 1.6.0"): "Adds new parameters for configuring hall sensor velocity measurement" | — |
| 198 | 251 | REVLib 2027 alpha 7 (September 2026) "Removes hall sensor velocity averaging configurations in … | C1-S09, lines 473–474; C1-S08, row kEncoderSampleDelta | Verified | L473–474 under revlib-2027.0.0-alpha-7 (2026-09-11): quotes exact; S08 "in 500μs increments" → 20 × 0.5 ms = 10 ms (arithmetic) | — |
| 199 | 252 | CTRE Phoenix 5 comparison: a velocity sample is taken every 1 ms as the position change over th… | C1-S23, section "Velocity Measurement Filter" | Verified | "Velocity Measurement Filter": "Every 1ms ... position sampled 100ms-prior ... rolling average is sized for 64 samples ... default" | — |
| 200 | 253 | CTRE's procedure for choosing these filters: start with period 1 ms and window 1, sweep the mot… | C1-S23, section "Changing Velocity Measurement Parameters" | Verified | "Recommended Procedure": set to 1, sweep, increase period until not stair-stepping, then window "sufficiently smooth, but still responsive enough" | — |
| 201 | 254 | CTRE notes that unless sensor velocity is "hundreds of sensor units per sampling period" the me… | C1-S23, same section | Verified | "Unless the sensor velocity is considerably fast (hundreds of sensor units per sampling period) the measurement will be very coarse" | — |
| 202 | 255 | Poorly installed encoders and inappropriate filter settings produce noisy velocity signals that… | C1-S18, `viewing-diagnostics.rst`, "Noisy Velocity Signals" | Verified | "Noisy Velocity Signals": "poorly-installed encoders ... as can inappropriate filtering settings"; "problematic for robot code much the same way" | — |

### Findings: Current limits, voltage compensation and ramps

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 203 | 258 | Smart Current Limit: the SPARK reduces its output voltage to keep the motor phase current at th… | C1-S10, `SparkBaseConfig.java` lines 213–228; C1-S13, section "Limiting Current" | Verified | SparkBaseConfig L213–228: "reduce the controller voltage output ... enabled by default and used for brushless only ... highly recommended when using the NEO ... low internal resistance"; S13 "maintain a constant phase current" | — |
| 204 | 259 | The Smart Current Limit can scale linearly with speed (stall limit below `limitRpm`, reducing t… | C1-S10, `SparkBaseConfig.java` lines 244–278 | Verified | L244–278: "limit the current based on the RPM ... linear fashion to help with controllability in closed loop control. For a response that is linear the entire RPM range leave limit RPM at 0" | — |
| 205 | 260 | Default Smart Current Limit is 80 A; REV suggests 40–60 A for the NEO. | C1-S13, section "Suggested Current Limits"; C1-S08, row kSmartCurrentStallLimit | Verified | S13 "default setting is 80A"; table NEO "40A - 60A"; S08 kSmartCurrentStallLimit default 80A | — |
| 206 | 261 | The secondary current limit is a simple on/off limit that briefly disables the output (in 50 µs… | C1-S10, `SparkBaseConfig.java` lines 286–306 | Verified | L286–306: "disable the output ... briefly ... simplified on/off controller ... enabled by default but is set higher than the default Smart Current Limit"; "PWM cycles (20kHz)" (= 50 µs; S08 kCurrentChopCycles "PWM period (50μs)") | — |
| 207 | 262 | Motor torque is proportional to phase current, not the controller's input current; average inpu… | C1-S14, section "Locked-rotor Testing with SPARK MAX Smart Current Limit" | Verified | S14: "Motor torque is proportional to phase current, not the input current"; "Average Input Current = Average Phase Current x Duty Cycle" | — |
| 208 | 263 | REV locked-rotor tests (motor held stalled at a fixed Smart Current Limit): the NEO 550 survive… | C1-S14, NEO and NEO 550 tabs, "Time to Failure Summary" | Verified | NEO 550 "Time to Failure Summary": 20A survived 220s; 40A ~27s; 60A ~5.5s; 80A ~2.0s; NEO tab has only image graphs at 40/50/60/80 A | — |
| 209 | 264 | Battery voltage sag changes mechanism performance; "voltage compensation" keeps the control-loo… | C1-S17, "Voltage Sag" | Verified | "Voltage Sag": "keep the output voltage of the control loops constant despite changes in the bus voltage"; "cannot increase the voltage ... beyond what is available on the bus" | — |
| 210 | 265 | Voltage compensation rescales the command as V = Vcmd·Vnominal/Vrail. | C1-S27, section 6.10.6 "Voltage compensation", eq. 6.23 | Verified | §6.10.6 eq. 6.23 V = Vcmd·Vnominal/Vrail | — |
| 211 | 266 | REVLib `voltageCompensation(nominalVoltage)` "Sets the voltage compensation setting for all mod… | C1-S10, `SparkBaseConfig.java` lines 389–399 | Verified | L389–399: javadoc quote exact; writes kCompensatedNominalVoltage and kVoltageCompensationMode = 2 | — |
| 212 | 267 | CTRE: duty-cycle output is affected by battery voltage, while voltage output "often results in … | C1-S22, `ctre_2026_phoenix6_closed_loop_requests.md`, "Choosing Output Type" | Verified | "Choosing Output Type": DutyCycle affected by battery voltage; Voltage "often results in more stable and reproducible behavior"; torque "Kv is generally unnecessary ... torque request is directly proportional to acceleration" | — |
| 213 | 268 | Closed-loop ramp rate is "the maximum rate at which the motor controller's output is allowed to… | C1-S10, `SparkBaseConfig.java` lines 372–386; C1-S08, rows kRampRate and kClosedLoopRampRate | Verified | L372–386: "maximum rate at which the motor controller's output is allowed to change"; "Time in seconds to go from 0 to full throttle"; `rate = 1.0 / rate`; S08 kClosedLoopRampRate "0 DC/sec", kRampRate "0 disables" | — |
| 214 | 269 | MAXMotion Velocity honours a configured maximum acceleration and uses an internal velocity loop… | C1-S06, "MAXMotion Velocity Control" | Verified | S06: "Honoring the maximum acceleration ... in a controlled way, reducing power draw and increasing consistency"; "internal velocity closed-loop controller" | — |
| 215 | 270 | The idle-mode parameter sets the half-bridge state (coast or brake) when the controller command… | C1-S08, row kIdleMode | Verified | Row kIdleMode: "State of the half bridge when the motor controller commands zero output or is disabled" | — |

### Findings: Low-speed precision

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 216 | 273 | NEO V1.1: 42 counts per revolution hall-sensor encoder, free speed 5676 RPM, motor Kv "473 Kv" … | C1-S12, table "Motor Specifications" | Verified | Spec table: 42 counts per rev., 5676 RPM, "473 Kv", 2.6 Nm, 105 A | — |
| 217 | 274 | Reading an incremental encoder at a fixed sample rate gives a position quantization error of up… | C1-S25, abstract and section 2 | Verified | Abstract "numerical differentiation largely amplify the quantization errors"; §2 "half an encoder count" | — |
| 218 | 275 | Time stamping estimates velocity by fitting a polynomial through recent encoder edges and their… | C1-S25, abstract and sections 5–6 | Verified | Abstract: "velocity estimation is improved by 54% and the acceleration estimation by 92%"; §6 "r = 3 ... 92% more accurate than with r = 0" | — |
| 219 | 276 | Friction can cause steady-state errors, limit cycling and hunting. | C1-S26, Introduction, p. 3 | Verified | Introduction p. 3: "steady-state errors, limit cycling and hunting" | — |
| 220 | 277 | Basic friction models combine Coulomb friction (constant, opposing motion), viscous friction (p… | C1-S26, sections 1.1–1.1.1, pp. 4–5 | Verified | §1.1–1.1.1 pp. 4–5 Coulomb, viscous, Stribeck; friction at zero velocity not a function of velocity only | — |
| 221 | 278 | With PD control, stick-slip may occur at low velocity; removing it with high gains risks instab… | C1-S26, section 2.1, p. 14 | Verified | §2.1 p. 14: PD stick-slip at low velocity; high gains risk instability | — |
| 222 | 279 | Integral control removes steady-state error but can cause limit cycling at low or zero velocity… | C1-S26, section 2.1, p. 14 | Verified | §2.1 p. 14: integral "can cause limit cycling at low or zero velocity"; "deadband and anti-windup at velocity reversal. But unfortunately these methods" add errors | — |
| 223 | 280 | Dither (a small high-frequency added signal) can smooth the friction discontinuity at low veloc… | C1-S26, section 2.1, p. 14 | Verified | §2.1 p. 14 dither | — |
| 224 | 281 | Model-based friction feedforward uses the reference trajectory, does not affect closed-loop sta… | C1-S26, section 2.2.1, pp. 15–16 | Verified | §2.2.1 pp. 15–16: FF does not affect closed-loop stability; feedback needs "stability analysis" | — |
| 225 | 282 | The kS term cancels static friction; without it a mechanism with much static friction "will not… | C1-S16, section "Feedforward Simplifications" | Verified | "Feedforward Simplifications": "will not have a linear control voltage-velocity relationship unless ... Ks" | — |
| 226 | 283 | kS is applied in the direction of the desired velocity on SPARK; CTRE applies kS using the sign… | C1-S03, section "kS - Static Gain"; C1-S22, `ctre_2026_phoenix6_closed_loop_requests.md`, "Static Feedforward Sign" | Verified | S03 "applied in the direction of desired velocity"; S22 "Velocity Sign ... always used when running velocity closed loops"; closed-loop sign: kS too large → "dither or oscillate" | — |
| 227 | 284 | The SPARK MAX `kInputDeadband` parameter (default printed as "%0.05") is described as the "Perc… | C1-S08, row kInputDeadband | Verified | Row kInputDeadband: default "%0.05", "Percent of the input which results in zero output for PWM mode" | — |

### Recommended practice

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 228 | 288 | Identify each wheel drive with open-loop step tests as a first-order-plus-delay model (gain, ti… | C1-S37, sections 2.2 and 3.1; C1-S35, section 4 | Verified | S37 §2.2 "open-loop tests", §3.1 "must be obtained from measurements", dead zone ±20%; S35 §4 FOPDT powertrain form | — |
| 229 | 289 | Add sampling (about h/2, or more if the computation is not scheduled right after sampling) and … | C1-S28, section 6.7; C1-S29, section 2.1 | Verified | S28 §6.7 h/2, "often several sampling periods"; S29 §2.1 half rule and h/2 | — |
| 230 | 290 | Start with PI (derivative only for dominant second-order dynamics), tuned from the delay model,… | C1-S29, sections 3.3–3.4 and 6.1–6.2; C1-S37, section 2.2 | Verified | S29 §3.3 D "primarily recommended" for τ2 > θ; §3.4 τc = θ; §6.1–6.2 detune/filter; S37 §2.2 PI preferred when L > τ/2 | — |
| 231 | 291 | Make the inner velocity loop much faster than any outer loop that treats it as ideal (10× in Ås… | C1-S24, section 11.6, pp. 341–342 | Verified | S24 §11.6 pp. 341–342 (10×; verify afterwards) | — |
| 232 | 292 | Add anti-windup whenever integral action is used: back-calculation with Td < Tt < Ti (e.g. Tt =… | C1-S28, section 6.5 | Verified | S28 §6.5 back-calculation Td < Tt < Ti, √(TiTd); inhibit integration; setpoint limitation insufficient | — |
| 233 | 293 | Identify friction in both directions and at several speeds (steady-state steps plus ramps), ave… | C1-S32, sections 3.1–3.2, 4.1.4 and 4.1.6; C1-S31, section IV | Verified | S32 §§3.1–3.2, 4.1.4, 4.1.6 (quadrants, averaging); S31 §IV speed-range dependence | — |
| 234 | 294 | Apply friction compensation as feedforward from the reference where possible; it needs no extra… | C1-S26, section 2.2.1; C1-S31, section IV | Verified | S26 §2.2.1 FF "does not affect closed-loop stability"; S31 §IV reduced computational burden | — |
| 235 | 295 | Choose the controller sampling period so that h·ωc ≈ 0.05–0.14 relative to the intended crossov… | C1-S42, pp. 39–40 | Verified | S42 pp. 39–40 eq. 36 and "Antialiasing filters are important in all cases" | — |
| 236 | 296 | When identifying the drive, note that standard methods generally work well on open-loop data; i… | C1-S44, section 1; C1-S43, section 2 | Verified | S44 §1 open-loop "generally work well", closed-loop LS "typically ... biased", spectral/IV/subspace "erroneous"; S43 §2 validation data | — |
| 237 | 297 | For compact servo motors, protect the winding with a thermal (I²t-type) model based on nominal … | C1-S47, sections 1–3; C1-S46, section I | Verified | S47 §§1–3 (I²t on nominal current + thermal time constant; fixed overload unsuitable; gearbox torque); S46 §I fixed caps too conservative | — |
| 238 | 298 | If the supply voltage varies, scale the PWM command by Vnominal/Vbus measured at the PWM rate. | C1-S48, section 3.10 | Verified | S48 §3.10 eq. 20 Vnorm/Vbus(t), new reading every PWM ISR (source applies the factor to the modulation terms only, AC induction drive) | — |
| 239 | 301 | Confirm firmware/library versions and units before entering gains; on firmware 26/REVLib 2026 e… | C1-S02, table "Default Units"; C1-S09, line 256; C1-S03, "kV - Velocity Gain" | Verified | S02 table; S09 L256; S03 "Volts per RPM, prior to any gear ratio" | — |
| 240 | 302 | Set a Smart Current Limit suited to the motor (REV suggests 40–60 A for the NEO) and persist it… | C1-S13, "Suggested Current Limits" | Verified | S13 "Suggested Current Limits" NEO 40A–60A; "must be burned to flash ... to be retained" | — |
| 241 | 303 | Set all gains to zero and check motor direction so positive output moves the mechanism the inte… | C1-S04, "Tuning", steps 1–3 | Verified | S04 steps 1–3 | — |
| 242 | 304 | Identify kS, kV (and kA) with a quasistatic + dynamic test, forward and backward, on the ground… | C1-S18, `creating-routine.rst` and `running-routine.rst`; C1-S20, `Drive.java` lines 61–74 | Verified | S18 creating-routine (quasistatic + dynamic, fwd/back), running-routine (not on blocks); S20 Drive.java L61–74 per-side logs | — |
| 243 | 305 | Check the fit: simulated-velocity r² above 0.9, sensible velocity threshold, and at least two s… | C1-S18, `viewing-diagnostics.rst`; C1-S20, `FeedforwardAnalysis.cpp` lines 166–171 | Verified | S18 viewing-diagnostics r² > .9, threshold; S20 L166–171 | — |
| 244 | 306 | Enter the controller's real velocity-measurement delay (e.g. 112 ms for default NEO hall settin… | C1-S18, `analyzing-gains.rst`, "Measurement Delays"; C1-S20, `FeedbackControllerPreset.hpp` line 138 | Verified | S18 "Measurement Delays" (recalculate on custom filter settings); S20 L138 `112_ms` | — |
| 245 | 307 | Tune feedback on top of the feedforward: raise P until just before oscillation; add D only if n… | C1-S22, "Closed-Loop Overview"; C1-S16, "Combined Feedforward and Feedback Control"; C1-S17, "Integral Term Windup" | Partly supported | S22 "Increase Kp until the output starts to oscillate around the setpoint"; "Kd as much as possible without introducing jittering"; S16 FF first; S17 IZone / integrator cap | CTRE says raise kP "until the output starts to oscillate", not "until just before oscillation", and none of the three cited sources says to back off. Either cite REV (C1-S04 step 9: "decrease P") for backing off, or quote CTRE's wording. |
| 246 | 308 | Choose velocity filter settings by trading coarseness (too short a period) against lag (too lon… | C1-S23, "Changing Velocity Measurement Parameters" | Verified | S23 procedure: period vs stair-stepping, window vs responsiveness | — |
| 247 | 309 | Use voltage compensation (or voltage-based output) so gains behave the same as battery voltage … | C1-S17, "Voltage Sag"; C1-S22, "Choosing Output Type" | Verified | S17 "Voltage Sag"; S22 "Choosing Output Type" (Voltage output "more stable and reproducible") | — |
| 248 | 310 | Graph setpoint and measured value during every tuning test. | C1-S04, "Tuning" | Verified | S04 "Tuning": "setup a graph of the setpoint and that measured value" | — |

### Key numbers

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 249 | 315 | Inner vs outer loop bandwidth (worked example): 10 rad/s vs 1 rad/s (10×) | C1-S24 | Verified | S24 p. 341–342 | — |
| 250 | 316 | Average delay from sampling: h/2 (often several periods in practice) | C1-S28 | Verified | S28 §6.7 | — |
| 251 | 317 | Anti-aliasing prefilter bandwidth: ωs/8 | C1-S28 | Verified | S28 Ex. 6.3 | — |
| 252 | 318 | Tracking time constant: Td < Tt < Ti; rule of thumb √(Ti·Td) | C1-S28 | Verified | S28 §6.5 | — |
| 253 | 319 | SIMC PI: Kc = τ1/(k(τc+θ)), τI = min(τ1, 4(τc+θ)), τc = θ | C1-S29 | Verified | S29 eqs. 23–24, 28 | — |
| 254 | 320 | SIMC margins (τc = θ): GM 3.14, PM 61.4°, Ms 1.59, ωc·θ = 0.50 (GM 2.96, PM 46.9°, Ms 1.70 if τ | C1-S29 | Verified | S29 Table 2 (τ1 ≤ 8θ column; lag-dominant column 2.96/46.9/1.70) | — |
| 255 | 321 | Typical minimum margins: GM > 1.7, PM > 30° | C1-S29 | Verified | S29 §4.1.1 | — |
| 256 | 322 | Measurement filter allowed without much effect: τF up to about 0.5θ | C1-S29 | Verified | S29 §6.2 "up to about 0.5θ" | — |
| 257 | 323 | Tuning-rule applicability: ZN: L < τ/2; Cohen–Coon: L < 2τ; else dead-time rule; PI preferred w | C1-S37 | Verified | S37 p. 898 | — |
| 258 | 324 | Agricultural robot drive model: K = 49.3 rad/s/V, τ = 0.15 s, L = 0.2 s; dead zone ±20% | C1-S37 | Verified | S37 p. 902 eq. 35, ±20% | — |
| 259 | 325 | Agricultural robot speed-loop period: 150 ms | C1-S37 | Verified | S37 p. 904 Ts = 150 ms | — |
| 260 | 326 | UGV powertrain model: τc = 0.81 s, τd = 0.16 s | C1-S35 | Verified | S35 p. 21 (online EKF, LandTamer) | — |
| 261 | 327 | Coulomb–viscous friction fit: R² = 0.98, RMSE 0.29 N·m (Stribeck 0.18, LuGre 0.20) | C1-S32 | Verified | S32 Table 5 | — |
| 262 | 328 | Coulomb friction, step vs ramp method: 0.5141 vs 0.3293 N·m (36% apart); viscous within 0.43% | C1-S32 | Verified | S32 Table 4, §4.1.9 | — |
| 263 | 329 | Mobile-robot ID, best lab velocity error: 0.001 m/s (LM-initialised RLS) vs 0.023 (LM) / 0.011  | C1-S33 | Verified | S33 Table 4 lab columns | — |
| 264 | 330 | Curiosity wheel-rate update rate: 8 Hz | C1-S34 | Verified | S34 §4.1 8 Hz | — |
| 265 | 331 | Curiosity TRCTL flight effect: wheel current −18.7%, drive duration ≈ +10%, CPU < 3% | C1-S34 | Verified | S34 abstract, §4.2 | — |
| 266 | 332 | Hall sensor resolution: 6 sectors of 60° per electrical cycle | C1-S36 | Verified | S36 §2.1 | — |
| 267 | 333 | SPARK on-board PID update period: 1 ms | C1-S01 | Verified | S01 "every 1ms" | — |
| 268 | 334 | Closed-loop gain slots: 4 (0–3) | C1-S07 | Verified | S07 "Slots" | — |
| 269 | 335 | kS / kV / kA default units: V / V per RPM / V per RPM/s | C1-S02 | Verified | S02 table | — |
| 270 | 336 | kP / kI / kD default units: duty cycle per rotation / per (rotation·ms) / (duty cycle·ms) per r | C1-S02 | Partly supported | S02 table units exact | "(position form)" is an inference; the table does not say so. Write "per-rotation form (inferred position form)". |
| 271 | 337 | Old vs new velocity FF in REV example: `velocityFF(1.0/5767)` → `kV(12.0/5767)` | C1-S11 | Verified | S11 2025 L72 / 2026 L75 | — |
| 272 | 338 | NEO hall velocity measurement: 32 ms period, 8-sample average (defaults); period range 8–64 ms; | C1-S10 | Verified | S10 EncoderConfig L146–176 | — |
| 273 | 339 | NEO hall velocity delay: 112 ms | C1-S20, C1-S21 | Verified | S20 L138–139; S21 | — |
| 274 | 340 | Quadrature velocity defaults: 100 ms period, 64-sample average | C1-S10 | Verified | S10 EncoderConfig L118–136 | — |
| 275 | 341 | Quadrature (non-NEO) delay: 81.5 ms | C1-S20 | Verified | S20 REV_NON_NEO 81.5_ms | — |
| 276 | 342 | CTRE Phoenix 6 measurement delay: ≈1 ms | C1-S20 | Verified | S20 CTRE_V6 1_ms | — |
| 277 | 343 | NEO encoder resolution: 42 counts/rev | C1-S12 | Verified | S12 | — |
| 278 | 344 | NEO free speed / motor Kv: 5676 RPM / "473 Kv" | C1-S12 | Verified | S12: Nominal Operating Voltage 12 V, 5676 RPM, 473 Kv | — |
| 279 | 345 | Smart Current Limit default: 80 A | C1-S08, C1-S13 | Verified | S08 row 59 "80A"; S13 | — |
| 280 | 346 | Suggested NEO current limit: 40–60 A | C1-S13 | Verified | S13 | — |
| 281 | 347 | SysId default quasistatic ramp / dynamic step: 1 V/s / 7 V | C1-S18 | Verified | S18 creating-routine L22 | — |
| 282 | 348 | SysId space for drive test: ≥10 ft, ideally ~20 ft | C1-S18 | Verified | S18 running-routine L7 | — |
| 283 | 349 | Moving-average delay: (N − 1)·T/2 | C1-S18, C1-S27 | Verified | S18 d = T(n−1)/2; S27 Thm B.5.1 | — |
| 284 | 350 | Backward-difference delay: T/2 | C1-S27, C1-S20 | Verified | S27 Thm B.5.2; S20 .hpp "T / 2" | — |
| 285 | 351 | Derivative filter constant N: 2–20 | C1-S24 | Verified | S24 §10.5 | — |
| 286 | 352 | Crossover limit from RHP zero z: ωgc < 0.6·z (delay τ ≈ zero at 2/τ) | C1-S24 | Verified | S24 §11.5 eq. 11.16 | — |
| 287 | 353 | Ziegler–Nichols ultimate-gain PI: gain 0.45·Su, reset rate 1.2/Pu | C1-S38 | Verified | S38 pp. 762, 765 | — |
| 288 | 354 | Ziegler–Nichols ultimate-gain PID: gain 0.6·Su, reset rate 2/Pu, pre-act Pu/8 | C1-S38 | Verified | S38 pp. 763, 765 | — |
| 289 | 355 | Relay critical gain: kc = 4d/(πa) | C1-S39 | Verified | S39 p. 646 eq. 1 | — |
| 290 | 356 | Sampling-period rule: h·ωc ≈ 0.05–0.14 (Nyquist 23–70× crossover) | C1-S42 | Verified | S42 eq. 36 (ζf = 0.707, gN = 0.1) | — |
| 291 | 357 | I²t current limiting: 2× nominal → limited after 0.3 τth; 4× nominal → after 0.1 τth | C1-S47 | Verified | S47 §2 Example | — |
| 292 | 358 | Typical max output current: 2–3× nominal current | C1-S47 | Verified | S47 §3 | — |
| 293 | 359 | Bus-voltage compensation: modulation × Vnorm/Vbus(t), updated every PWM interrupt | C1-S48 | Verified | S48 §3.10 eq. 20; p. 47 "every time the PWM ISR is executed" | — |
| 294 | 360 | Tracked field robot loop rates: 50 Hz track-speed PID; 5 Hz outer controller | C1-S49 | Verified | S49 §2.1 p. 4 "50-Hz", "5-Hz" | — |
| 295 | 361 | Tracked skid-steer straight-line current: sand 26.17 A, mud 24.01 A, gravel 21.77 A, asphalt 21 | C1-S50 | Verified | S50 §4.1 0.75 m/s; Table 2 | — |
| 296 | 362 | Skid-steer per-side power limit example: outer motor ≈ 58 W needed vs 51 W per side (102 W tota | C1-S51 | Verified | S51 §5.2 pp. 310–311 (58 w predicted vs 51 w; 102 w) | — |

### How it is tested

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 297 | 367 | Open-loop voltage step: K, τ and transport delay L of the drive; dead zone | C1-S37 | Verified | S37 p. 902: "repeated step changes in the control signal ... applied to the model"; dead zone Fig. 5b | — |
| 298 | 368 | Online powertrain fit (EKF): Wheel time constant and delay | C1-S35 | Verified | S35 p. 20–21: predicted wheel velocities "closely match" encoder velocities | — |
| 299 | 369 | Constant-voltage sweep, both directions: Viscous and Coulomb friction (least squares) | C1-S32 | Verified | S32 §4.1.6: "RSD values below 3% for the viscous ... below 4% for the Coulomb", n = 5 | — |
| 300 | 370 | Voltage ramp at several slopes, both directions: Viscous and Coulomb friction from slope/interc | C1-S32 | Verified | S32 §4.1.7: viscous "RSD approximately 3.2%", Coulomb "approximately 6.5%" | — |
| 301 | 371 | Coast-down after input removal: Inertia (given friction) | C1-S32 | Verified | S32 Table 9: RSD 3.05% and 4.08%, 8 trials; §4.1.8 "below 4.5%" | — |
| 302 | 372 | Friction-model comparison: Static model adequacy | C1-S32 | Verified | S32 Table 5 R², RMSE | — |
| 303 | 373 | Closed-loop trajectory tracking after identification: Parameter quality | C1-S33 | Verified | S33 Table 4: ex, ey, eθ, ev, eω, tr | — |
| 304 | 374 | Stability margins of tuned loop: Robustness | C1-S29 | Verified | S29 Table 2, §4.1.1 GM > 1.7, PM > 30 | — |
| 305 | 375 | Before/after flight comparison: Effect of wheel-rate algorithm | C1-S34 | Verified | S34 §4.2 flight comparison (mean current, drive duration; significance test) | — |
| 306 | 376 | Quasistatic ramp (fwd/back): kS and kV (acceleration negligible) | C1-S18 (`creating-routine.rst`, `viewing-diagnostics.rst`) | Verified | S18 creating-routine, viewing-diagnostics "very nearly linear" | — |
| 307 | 377 | Dynamic step (fwd/back): kA | C1-S18 | Verified | S18 "approximately exponential approach" | — |
| 308 | 378 | SysId fit statistics: Model quality | C1-S18 (`viewing-diagnostics.rst`) | Verified | S18 viewing-diagnostics: sim r² > .9; accel r² < ~0.2 kA dubious | — |
| 309 | 379 | Manual breakaway test: kS | C1-S16, C1-S03 | Verified | S16 "largest voltage applied before the mechanism begins to move"; S03 "smallest output ... then decrease it slightly" | — |
| 310 | 380 | Setpoint vs measurement graph during P/D tuning: Tracking, oscillation | C1-S04 | Verified | S04 step 10 "quick, precise, and repeatable" | — |
| 311 | 381 | Velocity-filter sweep: Measurement coarseness vs lag | C1-S23 | Verified | S23 Recommended Procedure | — |
| 312 | 382 | Locked-rotor at fixed current limit: Thermal time to failure | C1-S14 | Verified | S14 NEO 550 Time to Failure Summary; "The intent is to show the point of failure" | — |
| 313 | 383 | Time stamping with vs without skip (r = 3 vs r = 0): Velocity and acceleration estimate error | C1-S25 | Verified | S25 §§5–6 r = 3 vs r = 0: 54% / 92% | — |
| 314 | 384 | Ziegler–Nichols ultimate-gain test: Critical gain Su and period Pu | C1-S38 | Verified | S38 pp. 760–762: ultimate sensitivity from sustained oscillation | — |
| 315 | 385 | Relay feedback test: Critical gain and period, automatically | C1-S39 | Verified | S39 p. 646: period "by measuring the times between zero-crossings", amplitude "peak-to-peak" | — |
| 316 | 386 | Model validation on fresh data: Whether an identified model generalises | C1-S43 | Verified | S43 §2 validation data | — |
| 317 | 387 | Straight-line constant-speed drive on several terrains: Motor current per side as traction-effo | C1-S50 | Verified | S50 §4.1–4.2: straight-line currents per terrain; paper uses them as terrain "fingerprint" | — |

### Common mistakes

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 318 | 390 | Assuming an inner loop is "perfect" in the outer-loop design without checking that it is fast e… | C1-S24, section 11.6, p. 342 | Verified | S24 p. 342 "may not be valid" | — |
| 319 | 391 | Using a high proportional gain with a long sampling period, giving large overshoot and slow dam… | C1-S37, section 4.1, p. 904 | Verified | S37 p. 904 "high proportional gain could be problematic when the sample time is long"; Cohen–Coon "significant overshoot and slow damping" | — |
| 320 | 392 | Tuning from datasheet motor constants and ignoring transport delay and dead zone, which "must b… | C1-S37, section 3.1, p. 902 | Verified | S37 p. 902 "must be obtained from measurements"; dead zone | — |
| 321 | 393 | Ignoring the delay added by sampling and filtering; a delay harms control more than a lag of eq… | C1-S28, section 6.7; C1-S29, section 2.1 | Verified | S28 §6.7 sampling delay; S29 p. 293 "worse than that of a lag of equal magnitude" | — |
| 322 | 394 | Relying on setpoint limits to prevent windup; this does not help with disturbance-induced windu… | C1-S28, section 6.5 | Verified | S28 §6.5 "does not avoid windup caused by disturbances" | — |
| 323 | 395 | Choosing a very small anti-windup tracking time constant in a loop with derivative action, so n… | C1-S28, section 6.5 | Verified | S28 §6.5 "spurious errors ... accidentally resets the integrator" | — |
| 324 | 396 | Identifying Coulomb friction in only one direction or with only one method; it was direction-de… | C1-S32, sections 4.1.6 and 4.1.9 | Verified | S32 §4.1.6 (p = 0.0014), §4.1.9 (36%) | — |
| 325 | 397 | Leaving powertrain dynamics out of a predictive vehicle model, which leaves "large prediction e… | C1-S35, section 4, p. 21 | Verified | S35 p. 21 quote exact | — |
| 326 | 398 | Using only offline-identified parameters on a vehicle whose payload changes. | C1-S33, section 2 | Verified | S33 §2 "sufficient only" without loads | — |
| 327 | 399 | Entering a pre-2026 `kF`/`velocityFF` value (duty cycle per RPM) as the 2026 `kV` (volts per RP… | C1-S11, 2025 line 72 vs 2026 lines 74–75 | Verified | S11 comment "kV is now in Volts, so we multiply by the nominal voltage (12V)"; 1.0/5767 → 12.0/5767 | — |
| 328 | 400 | Forgetting that kV/kA are per motor RPM by default, "prior to any gear ratio", when copying gai… | C1-S03, "kV - Velocity Gain" | Verified | S03 "By default, the units are Volts per RPM, prior to any gear ratio" | — |
| 329 | 401 | Setting only one conversion factor: the velocity factor is independent of the position factor. | C1-S02, "Velocity Conversion Factor" | Verified | S02 "completely independent ... both need to be set" | — |
| 330 | 402 | Using the default SysId feedback gains after changing the controller's velocity filter; the mea… | C1-S18, "Measurement Delays" | Verified | S18 "Measurement Delays": "the measurement delay must be recalculated"; unstable gains | — |
| 331 | 403 | Using PID-only velocity control; it cannot hold speed without a steady error. | C1-S16, "Issues with Feedback Control Alone" | Verified | S16 "Issues with Feedback Control Alone" | — |
| 332 | 404 | Relying on integral gain to hide a bad feedforward, leading to windup and overshoot. | C1-S17, "Integral Term Windup"; C1-S24, section 10.4 | Verified | S17 important box; S24 §10.4 large transients | — |
| 333 | 405 | Characterizing a drivetrain on blocks. | C1-S18, `running-routine.rst` | Verified | S18 running-routine "can not be accurately characterized while on blocks" | — |
| 334 | 406 | Running a NEO near stall at the default 80 A limit. REV suggests 40–60 A for the NEO. REV's num… | C1-S13, "Suggested Current Limits"; C1-S14, NEO 550 tab, "Time to Failure Summary" | Verified | S13 NEO 40A–60A; S14 NEO 550 27 s / 5.5 s / 2.0 s; NEO tab graphs + raw-data link only | — |
| 335 | 407 | Setting P so high in MAXMotion Velocity that the loop outruns the acceleration target, giving j… | C1-S06, "Tips for Smooth Motions" | Verified | S06 "If the underlying velocity PID outruns the acceleration target, the motion may seem jittery" | — |
| 336 | 408 | Assuming parameters set over CAN survive a power cycle without persisting/burning them to flash… | C1-S13, "Limiting Current" | Verified | S13 "must be burned to flash via code or the Hardware Client in order to be retained through a power cycle" | — |
| 337 | 409 | Fitting a drive model to data logged while the speed loop is closed with ordinary least squares… | C1-S44, section 1 | Verified | S44 report pp. 2–3: correlation of input and noise; LS "typically biased"; spectral analysis "erroneous" | — |
| 338 | 410 | Treating anti-windup as a way to reduce step-response overshoot, or as the same thing as bumple… | C1-S41, introduction | Verified | S41 introduction (misconceptions) | — |
| 339 | 411 | Using datasheet thermal parameters and a fixed ambient temperature for motor temperature estima… | C1-S46, sections I and II-A | Verified | S46 §II-A: datasheet parameters differ; "error of the ambient temperature"; old vs new motor 15 °C | — |
| 340 | 412 | Checking only total vehicle power on a skid-steered vehicle; one side's motor drive can saturat… | C1-S51, section 5.2 | Verified | S51 §5.2 58 w per side > 51 w while total 58 w < 102 w | — |

### Disagreements between sources

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 341 | 415 | **Does LuGre capture hysteresis?** Cardona Soto et al. state the LuGre model replicates "the St… | C1-S32, section 2.6; C1-S31, section II; C1-S40, abstract and section IV-B, Fig. 3, p. 422 | Verified | S32 §2.6 quote exact; S31 §II "does not take into account possible hysteresis effects"; S40 abstract and Fig. 3 p. 422 | — |
| 342 | 416 | **Is Coulomb plus viscous friction enough?** Cardona Soto et al. find it explains more than 98%… | C1-S32, section 4.1.4; C1-S31, section III-A | Verified | S32 §4.1.4 (>98%, /ω/ > 5 rad/s); S31 §III-A "cannot provide accurate tracking and positioning results" | — |
| 343 | 417 | **Ramp slope for friction identification.** Cardona Soto et al. derive that the ramp slope must… | C1-S32, section 3.2 | Verified | S32 §3.2: "in [24] is mentioned that r must be large enough ... results in an overestimation of fc" | — |
| 344 | 418 | **How much integral action?** General PID practice treats PI as the default ("most loops are ac… | C1-S28, section 6.1; C1-S29, section 3.3; C1-S04; C1-S17 | Verified | S28 §6.1; S29 §3.3 (τI always finite); S04 I "not often recommended"; S17 "probably have an inaccurate feedforward model" | — |
| 345 | 419 | **Derivative action in velocity loops (general view).** SIMC recommends D only for dominant sec… | C1-S29, section 3.3; C1-S37, section 2.2 | Verified | S29 §3.3; S37 §2.2 (the "sides with" framing is the README's own synthesis, stated as such) | — |
| 346 | 420 | **Name and units of parameter ID 16.** The SPARK MAX parameter reference page still lists ID 16… | C1-S08, row kF_0; C1-S10, `SparkParameters.java` line 52; `FeedForwardConfig.java` line 81; C1-S11 | Verified | S08 row 16 "kF_0"; S10 SparkParameters L52 `kV_0(16`, FeedForwardConfig L81 "Volts per velocity"; S11 | — |
| 347 | 421 | **Quadrature velocity defaults.** REVLib 2026 source and the parameter page say 100 ms / 64 sam… | C1-S10, lines 118–136; C1-S08; C1-S09, line 474 | Verified | S10 L118–136 (100 ms / 64); S08 kEncoderSampleDelta 200 × 500 µs, depth 64; S09 L474 "depth to 8 and sample delta to 20" | — |
| 348 | 422 | **Is D useful in a velocity loop?** REV's general procedure adds D to damp oscillation, CTRE sa… | C1-S04; C1-S22; C1-S16; C1-S27, 6.10.7 | Verified | S04 step 9; S22 Kd "as much as possible without introducing jittering"; S16 "Kd is not useful"; S27 §6.10.7 | — |
| 349 | 423 | **How to measure kS manually.** WPILib: largest voltage before motion begins; REV: smallest out… | C1-S16; C1-S03; C1-S22 | Verified | S16, S03, S22 quotes as in findings | — |
| 350 | 424 | **Which anti-windup scheme is best?** Åström recommends back-calculation with a tracking time c… | C1-S28, section 6.5; C1-S41, abstract; C1-S17; C1-S10 | Verified | S28 §6.5; S41 abstract; S17 IZone/cap; S10 iMaxAccum. README states only abstract/intro of S41 read | — |
| 351 | 425 | **Fixed overload time vs thermal model.** AC frequency inverters commonly allow a fixed overloa… | C1-S47, sections 1–2; C1-S10, `SparkBaseConfig.java` lines 213–278; C1-S14 | Verified | S47 §§1–2 ("Most frequency inverters offer an overload ratio of 110% or 160%"); S10 L213–278; S14 locked-rotor times | — |

### Open questions

| # | Line | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 352 | 428 | No open source found gives a general quantitative rule for how much faster a wheel velocity loo… | C1-S24 | Verified | S24 p. 341 is the only 10× figure among the sources (absence claim; not contradicted) | — |
| 353 | 429 | No peer-reviewed source found reports velocity-loop performance (tracking error, bandwidth) for… | C1-S49; C1-S50; C1-S51; C1-S34; C1-S33; C1-S37 | Verified | S49 wet soil, path error, no speed-tracking metric (Fig. 11 is measured vs estimated speed); S50 sand/mud/gravel/asphalt currents; S51 "lab vinyl surface"; S34, S33, S37 wheeled | — |
| 354 | 430 | The Hall-sensor review is aimed at PMSM field-oriented control and does not quantify speed-esti… | C1-S36 | Verified | S36 title/scope PMSM; Table 1 compares methods qualitatively, no error-vs-speed figures found | — |
| 355 | 431 | The peer-reviewed friction-compensation surveys Armstrong-Hélouvry et al. (1994) and Olsson et … | C1-S31; C1-S32; C1-S26; C1-S40 | Verified | S31, S32, S26 (TU/e traineeship report), S40 used; surveys not in sources/ | — |
| 356 | 432 | A peer-reviewed open survey comparing conditional integration with back-calculation on motor sp… | C1-S28; C1-S30; C1-S41 | Verified | S28 §6.5, S30, S41 (abstract and introduction only) | — |
| 357 | 437 | No open source was found for Kessler's symmetrical optimum or modulus optimum, the standard tun… | — | Verified | S22 "Choosing Output Type" torque-current output (only indirect coverage) | — |
| 358 | 438 | No general (non-vendor, non-FRC) source was found on how a supply-voltage drop below the voltag… | C1-S28; C1-S30; C1-S48 | Verified | S28 §6.5 and S30 treat voltage limits only as saturation; S48 is a mains-fed AC drive note | — |
| 359 | 439 | Physical models of PM DC/BLDC motors from a drives textbook (Leonhard, Krishnan) were not obtai… | C1-S15; C1-S27; C1-S37 | Verified | S15 feedforward derivation; S27 flywheel/drivetrain models; S37 FOPDT model | — |

## Corrections applied (2026-09-28)
Editor pass applying this claim review and `SOURCE_AUDIT.md`. All 12 Partly supported items fixed; no Not supported items existed; no source failed the audit, so no source was removed.

### Claims
| # | README location | Change |
|---|---|---|
| 1 | Summary, identification bullet | Now "identified from measured data … from open-loop tests on an agricultural robot … or by online extended-Kalman-filter fitting while a skid-steered UGV drove in circles"; "in general" dropped. |
| 2 | Foundational, C1-S28 | "by the leading PID authority" removed. |
| 3 | Foundational, C1-S29 | "Most-cited" → "Widely used". |
| 4 | Foundational, C1-S38 | "First published PID tuning rules" → "Classic early PID tuning rules". |
| 5 | Foundational, C1-S45 | "the most common robust motion-control tool in drives" → "a widely adopted robust motion-control tool". |
| 6 | Loop structure, setpoint weighting | Citation now "section 6.3, eq. 6.4 (and 6.5), Fig. 6.4". |
| 7 | Tuning, relay test | Now "a process with at least 180° phase lag at high frequencies may then oscillate at approximately the critical period". |
| 8 | Actuator limits, thermal model | "A common thermal model" → "Kawaharazuka et al.'s basic two-node thermal model". |
| 9 | Product section, closed-loop basics | Replaced with REV's own wording: "a process that uses feedback to improve the accuracy of its outputs". |
| 10 | Product section, units table | "i.e. the table is written for position control" → table does not name a mode; per-rotation units and position-conversion-factor note "suggest the position form". |
| 11 | Recommended practice 18 | "raise P until just before oscillation" → "raise P until the output starts to oscillate, then back off (decrease P) or add a little D"; C1-S04 "Tuning" step 9 added to the citation. |
| 12 | Key numbers, gain units row | "(position form)" → "(per-rotation form; inferred to be position form)". |

### Sources
- **C1-S32, C1-S33 (format):** the Europe PMC markdown copies (equations lost) were replaced by the publisher PDFs from the PMC open-access dataset (`pmc-oa-opendata` PMC12788085.1, PMC9865440.1; MDPI's own PDF link returned "Access Denied"). The new files are `cardonasoto_2026_dc_motor_step_ramp_identification.pdf` and `siwek_2023_diff_drive_dynamic_identification.pdf`, and the `.md` files are deleted. All cited sections and tables (C1-S32 §§2.6, 3.1–3.3, 4.1.2–4.1.9, Tables 4–6, 9, 10; C1-S33 §§2, 3.1, Table 4, Appendix A) were re-checked in the PDFs and have the same numbering and values.
- **C1-S32 citation:** the extra author "Y. Xi" was removed. The PDF shows Yi Xi as Academic Editor, not as an author.
- **C1-S33 citation:** "et al." was replaced with the full author list (Siwek, Panasiuk, Baranowski, Kaczmarek, Prusaczyk, Borys).
- **C1-S11:** level C → B. The 2026 example files are pinned to REVLib-Examples commit 6b03aa4; the pinned files are byte-identical, ignoring whitespace, to the copies fetched from `main`. File header lines and the link were updated.
- **C1-S15 to C1-S19:** pinned to frc-docs commit 1897feb (all 10 rst files diffed identical to that commit). File header lines and citations were updated.
- **C1-S20:** level C → B (official project source).
- **C1-S23:** the citation now states it is legacy Phoenix 5 documentation, still current for Talon SRX.
- **C1-S26:** the citation now states the link is a third-party mirror.
- **C1-S28:** level A → C (unpublished lecture-note chapter). The reason is stated in the citation.
- **C1-S46:** RA-L DOI 10.1109/LRA.2020.2990889 added (confirmed via Crossref).
- **C1-S49:** issue and DOI added: 35(7):1050–1062, DOI 10.1002/rob.21794 (confirmed via Crossref).
- **C1-S51:** level A → C (InTech edited-volume chapter). The peer-reviewed companion (Yu et al., IEEE T-RO 26(2), 2010) is named as not downloaded.
- **Foundational references:** added not-downloaded rows for Åström & Hägglund 1995, Ljung 1999 and Preitl & Precup 1999.
- **Status:** set to Verified.
- **Mechanical check:** 68 files in `sources/`, each with a matching `file` type. Every file is in the Sources table. S01–S51 have no gaps or duplicates. Every cited ID exists and every row is cited.
