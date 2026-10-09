# C2 — Drive kinematics: claim verification

| | |
|---|---|
| **Topic** | C2 — Drive kinematics (`README.md`, sources C2-S01 … C2-S41) |
| **Date** | 2026-09-28 |
| **Reviewer** | independent — claims |
| **Method** | Every cited item in Summary, Foundational references, Findings, Recommended practice, Key numbers, How it is tested, Common mistakes, Disagreements and Open questions was checked against the cited file at the cited location. PDFs were read with `pdftotext -layout` and pages were counted by form feed (printed page numbers used where the README uses them: S21, S22, S35 translation pp. 733–769, S36 pp. 2632–2638, S25 pp. 2871–2876). The two image-only PDFs (S36 Caracciolo, S37 Martínez 2017) were read page by page as images. Code, YAML, RST and Markdown sources were read by line number. Rules: STANDARDS.md section 5. Sources themselves are not graded here (see `SOURCE_AUDIT.md`). This file replaces the 2026-09-27 version. "L" numbers refer to lines of `README.md` as of this review. |

## Counts

| Section | Checked | Verified | Partly supported | Not supported |
|---|---|---|---|---|
| Summary | 6 | 4 | 2 | 0 |
| Foundational references | 11 | 7 | 4 | 0 |
| Findings: classification / textbook | 13 | 13 | 0 | 0 |
| Findings: terramechanics | 15 | 15 | 0 | 0 |
| Findings: dynamic / power models | 9 | 9 | 0 | 0 |
| Findings: online / learned estimation | 15 | 15 | 0 | 0 |
| Findings: other platforms' calibration / validation | 11 | 11 | 0 | 0 |
| Findings: how the vehicle moves (Q1) | 21 | 21 | 0 | 0 |
| Findings: effective width (Q2) | 19 | 19 | 0 | 0 |
| Findings: load, inertia, slope | 12 | 11 | 1 | 0 |
| Findings: calibration experiments (Q3) | 21 | 21 | 0 | 0 |
| Findings: calibration on grass | 5 | 4 | 1 | 0 |
| Findings: rotation centre (Q4) | 13 | 13 | 0 | 0 |
| Findings: accuracy of models | 7 | 7 | 0 | 0 |
| Findings: controllers / configs | 13 | 13 | 0 | 0 |
| Recommended practice | 18 | 17 | 1 | 0 |
| Key numbers | 41 | 41 | 0 | 0 |
| How it is tested | 22 | 22 | 0 | 0 |
| Common mistakes | 22 | 21 | 1 | 0 |
| Disagreements | 12 | 12 | 0 | 0 |
| Open questions | 15 | 12 | 3 | 0 |
| **Total** | **321** | **308** | **13** | **0** |

## Items needing correction

| # | Item | Citation | Status | Problem | Correction needed |
|---|---|---|---|---|---|
| 1 | Summary 3 (L14): P3-AT "23.6 kg … plus a 15.8 kg sensor structure"; Warthog "590 kg" | S01 Tables I–II, §II; S03 Table I | Partly | Ratios and arithmetic are right (1/0.7612 = 1.31 … 1/0.6949 = 1.44; 3.08/1.2 = 2.57, 4.46/1.2 = 3.72; 1.159/0.95 = 1.22, 1.329/0.95 = 1.40). But the masses are not at the cited places: 23.6 kg is in S01 §IV.A and 15.8 kg in §IV.C; 590 kg is in S03 abstract/§IV text, not Table I. | Add [C2-S01, section IV.A, IV.C] and [C2-S03, section IV] for the masses. |
| 2 | Summary 5 (L16): external references include "motion capture" | S01 §III; S03 §IV–V; S26 §7; S28; S29; S30; S31; S25 §II.D | Partly | None of the cited locations uses motion capture. S25 uses stereo motion capture, but in §III, not the cited §II.D (S06 §V.A and S40 §4.1 also use it but are not cited here). | Cite [C2-S25, section III] (or [C2-S06, section V.A]) for motion capture. |
| 3 | Foundational: Campion 1996 is the "origin of the five-type classification … that the textbooks use" | S35 | Partly | S35 gives the five types (p. 741). No source states that Siegwart & Nourbakhsh or Lynch & Park take their classification from Campion; neither textbook excerpt cites Campion. | Drop "that the textbooks use" or cite a source that says so. |
| 4 | Foundational: Caracciolo 1999 is the "first skid-steer model with the added operational ICR constraint" | S36 | Partly | S36 introduces the operative constraint (§3.1) and Kozłowski builds on it (S02 cites "according to Caracciolo et al. (1999), an operational non-holonomic constraint"). No source says it was the *first*. | Write "introduces the operational ICR constraint that Kozłowski and later controllers build on". |
| 5 | Foundational: Bekker; Janosi & Hanamoto; Steeds 1950; Kitano & Kuma 1977 "known here through C2-S32 and C2-S34" | S32, S34 | Partly | Bekker and Janosi–Hanamoto appear in S32 and S34. Steeds appears in no downloaded source. Kitano & Kuma (J. Terramechanics 14(4), 1977) appears only in S06's reference list, not in S32 or S34. The descriptions "classical Coulomb skid-steer theory" and "first transient tracked-vehicle dynamics" are not stated in any source. | Say Kitano & Kuma is known through [C2-S06], and remove Steeds or mark it "not seen in any downloaded source"; drop the uncited characterisations or cite them. |
| 6 | Foundational: Pentzer et al. 2014 is the "first online EKF estimation of track ICRs" | (S03 §II, S30) | Partly | S03 says Pentzer et al. "tracked them individually using" an EKF on a 118 kg robot; S30 follows Pentzer. No source says it was the *first*. Bibliographic details (JFR 31(3):455–476, 2014; Yu et al. T-RO 26(2):340–353, 2010) match the reference lists of S03 and S39. | Write "online EKF estimation of track ICRs that later work (C2-S30) follows". |
| 7 | L163: J8 on-spot turn, "mean squared lateral-position error was 0.957 against 0.006" | S38 §IV.C, Table II | Partly | Table II gives (Σδy²)/N = 0.957 (ICRHK) vs 0.006 (ICRIK) for experiment #1. δx, δy are position errors along the global frame axes Xg, Yg (eq. (12)–(13)), not a lateral (body-frame) error. | Write "mean squared y-position error (global frame)" or "mean squared position error along one global axis". |
| 8 | L350 (Common mistakes): "J8 lateral mean squared error 0.957 vs 0.006" | S38 §IV.C, Table II | Partly | Same as #7. | Same as #7. |
| 9 | L197: Rover mini floor→grass parameter reuse, 0.158 m vs 0.028 m, wood tiles about 2× | S33 Table 2, p. 14 | Partly | Numbers and wording are right ("about six times as accurate", "twice as accurate"), but Table 2 and this text are on p. 10 of the kept copy (section 4.1), not p. 14. | Cite [C2-S33, Table 2, p. 10]. |
| 10 | Rec. practice 18 (L259): floor parameters gave about six times larger error on grass | S33 Table 2, p. 14; S40 §7.2 | Partly | Same page error as #9; S40 §7.2 support is fine (identification repeated for parquet and slippery terrain). | Cite [C2-S33, Table 2, p. 10]. |
| 11 | Open question L371: "the tracked-robot sources found are the Flipper [S06], the CV-04 [S25], and a 40 kg rubber-tracked robot [S29]" | S06, S25, S29 | Partly | The main claim (no identified effective width for a rubber-belt tracked robot on natural grass) holds. The list is out of date: C2-S37 (Auriga-α tracked robot, simulation), C2-S40 (MaxxII/LIMO tracked robots, parquet and soapy plastic), C2-S30 (16 t tracked vehicle) and C2-S31/S32 (tracked agricultural vehicles) are also tracked-vehicle sources. | Add S30, S31, S32, S37 and S40 to the list, or write "the tracked sources with identified width/slip values on specific surfaces include …". |
| 12 | Open question L382: "Pentzer et al. 2014 reportedly tested their ICR EKF on grass" | none | Partly | Uncited factual statement. No downloaded source mentions grass for Pentzer et al. (S03 and S30 only describe the EKF and the 118 kg robot). SCOPE.md records it came from a paywalled abstract. | Cite the abstract (with link) or drop the grass remark. |
| 13 | Open question L383: "Nikitin's empirical relation … and Kitano and Kuma's transient tracked dynamics are known only through citations" | none | Partly | Kitano & Kuma is cited in S06's reference list. Nikitin is not mentioned in any downloaded source, so "known through citations" has no support in this folder. | Remove Nikitin, or name the source that cites it; cite [C2-S06] for Kitano & Kuma. |

Minor location notes (counted as Verified): L190 cites S38 "section III.B, eq. (13), p. 3" — eq. (13) is on p. 4 (also covered by the cited "section IV, pp. 4–5"). L63/L284 cite S29 "Table 1, p. 8" — Table 1 is printed on p. 7; the matching text is on p. 8.

No uncited factual statements were found in Findings, Recommended practice, Key numbers, How it is tested or Common mistakes. L376, L384 and L385 (Open questions) are uncited process notes about what was or was not downloaded; they make no claim about the subject and are counted as Verified.

## Full verification table

| # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|
| | **Summary** | | | | |
| 1 | L12 No-slip wheel assumption; skid steer breaks it; texts call it degenerate or exclude it; track ICRs outside centrelines | S21 p.61; S22 p.513; S01 §II | Verified | S21 p.61 "four wheeled slip-skid steering system" degenerate; S22 p.513 "tanks and skid-steered vehicles are excluded"; S01 §II "always lie outside of the tread centerlines" | — |
| 2 | L13 Symmetric → DD with virtual width + slip factor; multiplier on command and odometry; Husky 1.875, Jackal 1.5 | S03 §III.B eq.(8)–(9); S10 L142–145, 257–260, 298–302; S15; S16 | Verified | S03 "b̂ = 2y0" eq.(9); S10 L143, L257–260, L298, L302; S15 L20 1.875; S16 L20 1.5 | — |
| 3 | L14 Ratios 1.3–1.5 (P3-AT, 23.6 + 15.8 kg), 2.6–3.7 (590 kg Warthog), 1.22–1.40 (40 kg tracked); vary with surface, tyres, CoM, inertia, slope, radius, (inferred) acceleration; ICR outside wheelbase → skid | S01 T I–II, §II; S02 p.484; S04 T3; S03 T I; S05 §V Fig.5; S29 §2.1 T1; S37 §III Fig.7(c); S38 §II.B | Partly | Ratios and arithmetic correct; masses are in S01 §IV.A (23.6 kg), §IV.C (15.8 kg) and S03 §IV text (590 kg), not the cited tables/sections | See #1 |
| 4 | L15 τ = τmax(1−e^(−j/K)); turning resistance/torque fall with radius; terrain-dependent parameters | S23 PDF pp.8,10,20; S32 pp.5–6 | Verified | S23 p.20 "magnitudes of both the left and right torques reduce as the commanded turning radius increases"; S32 eq.(10)–(11) | — |
| 5 | L16 External reference + fit over horizon; online EKF or gyro + empirical relation; 44–90 % improvement | S01 §III; S03 §IV–V; S26 §7; S28 Abstract, §6.1; S29 §3.2; S30 §6.2; S31 Abstract; S25 §II.D | Partly | Method and numbers OK (S30 44 %, S31 50.6–78.1 %, S28 72/83/90 %); total station now cited (S26 §7); "motion capture" not at any cited location | See #2 |
| 6 | L17 Dynamic/power models for elevation, terrain, accelerations, motor limits; outer motor consumes, inner can generate | S24 Abstract; S23 PDF pp.1,5,18 | Verified | S23 p.1 "substantial changes in elevation … frequent accelerations and decelerations"; S23 p.5 inner motor "first consumes power, then generates power" | — |
| | **Foundational references** | | | | |
| 7 | S21 Siegwart & Nourbakhsh: rolling/sliding constraints, ICR, degree of mobility | S21 | Verified | §3.2.3 pp.45–46; §3.3.1 eq.(3.40) p.61 | — |
| 8 | S22 Lynch & Park ch.13: nonholonomic and diff-drive kinematics | S22 | Verified | §13.1 p.514; §13.3.1 pp.523–525 | — |
| 9 | S01 Mandow: ICR skid-steer model with experimental calibration | S01 | Verified | §II eq.(2)–(8); §III eq.(11)–(13) | — |
| 10 | S02 Kozłowski & Pazderski: non-integrable ICR constraint, ICR stability condition | S02 | Verified | §2.1 eq.(6); p.484 "goes out of the robot wheelbase … loses its stability" | — |
| 11 | S34 Jia et al.: summarises Bekker and Janosi–Hanamoto | S34 | Verified | §3.2 p.494 pressure–sinkage; §3.4 eq.(22) "shear formula given by Janosi and Hanamoto" | — |
| 12 | S35 Campion: origin of five-type classification "that the textbooks use" | S35 | Partly | p.741 five types; no source links the textbooks to Campion | See #3 |
| 13 | S36 Caracciolo: "first" model with operational ICR constraint that Kozłowski builds on | S36 | Partly | S36 §3.1 eq.(7); S02 builds on it; "first" unsupported | See #4 |
| 14 | Martínez et al. 2005 IJRR 24(10):867–878; S37/S38 summarise constant-ICR result | S37, S38 | Verified | S37 ref. [12] "IJRR vol. 24, no. 10, pp. 867–878, 2005"; S38 §II.A cites [22] for constant tread ICRs | — |
| 15 | Wong, Theory of Ground Vehicles 4th ed. 2008; Wong & Chiang 2001 Proc IMechE D 215(3):343–355; used only as reported in S23 | S23 | Verified | S34 ref. 7 "Theory of Ground Vehicles, 4th ed. (Wiley, Aug. 2008)"; S06 ref. [4] "vol. 215, no. 3, pp. 343–355"; S23 pp.3–4, 7–8 | — |
| 16 | Bekker 1956; Janosi & Hanamoto 1961; Steeds 1950; Kitano & Kuma 1977; known via S32 and S34 | S32, S34 | Partly | Bekker, Janosi–Hanamoto in S32/S34; Steeds in no source; Kitano & Kuma only in S06 refs | See #5 |
| 17 | Pentzer 2014 "first" online EKF of track ICRs; Yu et al. T-RO 2010 | (S03, S30, S39) | Partly | Bibliography matches S03 ref [11], S39 ref 4; "first" unsupported | See #6 |
| | **Findings: classification / textbook** | | | | |
| 18 | L39 Rolling + sliding constraints; pure rolling at one contact point | S21 §3.2.3 pp.45–46 | Verified | p.46 "no lateral slippage … must not slide orthogonal to the wheel plane" | — |
| 19 | L40 Zero motion line; ICR on every line; mobility depends on constraints not wheels | S21 §3.3.1 pp.58–59 | Verified | p.58 "a function of the number of constraints … not the number of wheels" | — |
| 20 | L41 δm = 3 − rank C1(βs), 0–3 | S21 eq.(3.40) p.61 | Verified | eq.(3.40); "must range between 0 and 3" | — |
| 21 | L42 Diff drive δm=2, bicycle 1, omni 3 | S21 pp.61–62; Fig.3.14 p.63 | Verified | pp.61–62 text; Fig. 3.14 | — |
| 22 | L43 rank C1f>1 → circle/line; slip-skid degenerate; "dead-reckoning … less accurate and power efficiency is reduced dramatically" | S21 §3.3.1 p.61 | Verified | p.61 text (lines 1119–1123 of extraction) | — |
| 23 | L44 Campion: conventional wheel rolls without slipping, contact-point velocity zero in both directions | S35 §II.B p.736 | Verified | p.736 "скорость точки колеса … равна нулю … как в проекции на … плоскости колеса, так и … ортогональное" | — |
| 24 | L45 Five non-degenerate types (3,0),(2,0),(2,1),(1,1),(1,2); inequalities (16)–(18) | S35 §II.C p.741 | Verified | p.741 eq.(16) 1⩽δm⩽3, (17) 0⩽δs⩽2, (18) 2⩽δm+δs⩽3; "существует только пять типов" | — |
| 25 | L46 Type (2,0): fixed wheels on one axle else rank C1f>1 (HILARE); inference labelled | S35 §II.C p.742; S21 p.61 | Verified | p.742 "несколько … фиксированных колес, расположенных на одной оси (иначе rank[C1f] был бы больше 1)"; inference labelled | — |
| 26 | L47 Omnidirectional vs nonholonomic (one Pfaffian constraint) | S22 §13.1 p.514 | Verified | p.514 | — |
| 27 | L48 Chapter excludes tanks and skid-steered vehicles | S22 p.513 | Verified | "without skidding (i.e., tanks and skid-steered vehicles are excluded)" | — |
| 28 | L49 Diff-drive φ̇ = (r/2d)(uR−uL) etc.; canonical (v,ω) model | S22 eq.(13.14)–(13.15) pp.523–524; §13.3.1.4 p.525 | Verified | eqs. pp.523–524; canonical model p.525 | — |
| 29 | L50 No linear / continuous time-invariant law stabilises full pose; controllable | S22 §13.3.2 p.528 | Verified | p.528 "There is no linear controller that can stabilize the full chassis configuration"; Theorem 13.1 p.529 | — |
| 30 | L51 Odometry errors from slipping/skidding + integration; fuse with GPS in KF/PF | S22 §13.4 | Verified | "unexpected slipping and skidding of the wheels and to numerical integration error" | — |
| | **Findings: terramechanics** | | | | |
| 31 | L54 Slipping vs skidding definitions | S23 PDF p.3; S32 p.6 eq.(12) | Verified | S23 p.3 "if the wheel linear velocity computed … is larger than the actual … slipping occurs" | — |
| 32 | L55 Pressure–sinkage (Bekker) + shear–displacement | S34 §3.2 p.494; S32 p.5 | Verified | S34 p.494; S32 p.5 | — |
| 33 | L56 Janosi–Hanamoto τ = τmax(1−e^(−j/K)); τmax = c + p tanφ | S34 eq.(22)–(23) p.496; S32 eq.(10)–(11) | Verified | S34 eq.(22)–(23) | — |
| 34 | L57 Hump-shaped soils → Wong formula with K_r | S34 §3.4 eq.(24) p.496 | Verified | "For certain muskegs, dry sands, and certain snows … Wong's shear formula"; Kr residual shear stress | — |
| 35 | L58 j = i·x | S32 p.6 eq.(12) | Verified | eq.(12) | — |
| 36 | L59 Yu apply τss = pμ(1−e^(−j/K)) instead of immediate Coulomb max | S23 PDF pp.7–8 eq.(9) | Verified | p.7 "assumed that the shear stress takes on its maximum magnitude as soon as a small relative movement occurs"; eq.(9) p.8 | — |
| 37 | L60 Coulomb → same torque at all radii; measured torques fall with radius | S23 PDF p.10 eq.(24); p.20 §5.1 | Verified | p.10 "Coulomb's law leads to a resistance torque that has the same constant value"; p.20 | — |
| 38 | L61 Expansion factor 1.5 vinyl, >2 concrete; larger rolling resistance → larger α | S23 PDF p.6; S24 §II T II | Verified | S23 p.6; S24 Table II α 1.5 | — |
| 39 | L62 Slip-ratio model = expansion-factor model under eq.(5); no verifying experiment | S23 PDF pp.6–7 eq.(3)–(5) | Verified | pp.6–7 | — |
| 40 | L63 40 kg, 0.95 m; equivalent track 1.269/1.252/1.329/1.159 m; "wet surface…" | S29 §2.1 p.2; §4.2 T1 p.8 | Verified | p.2 "W = 40 kg … nominal track width is 0.95 m"; Table 1 (p.7) values; p.8 "wet surface generates the highest slipping effect" | — (Table 1 on p.7) |
| 41 | L64 Currents higher on sand, mud; flexible terrain, larger contact area | S29 §4.2 p.8 | Verified | Table 2 26.17/21.77/24.01/21.23 A; text p.8 | — |
| 42 | L65 n = 0.4811/0.6213/0.5094; slower track slips more; "does not change significantly" | S25 §II.D eq.(7); §III.B p.2873 | Verified | p.2873 values and quote | — |
| 43 | L66 Crawler slip 9.8/8.7/6.2 %; 12.45/11/15 % with 13 kN; 20 % climbing; 20° slope | S32 Abstract; p.11; p.15 | Verified | p.11 "9.8% dry sand, 8.7% sandy loam, and 6.2% clay"; "nearly 15%", "close to 11%", "12.45%"; p.15 "reaching 20 %"; abstract "20° slope" | — |
| 44 | L67 High internal motion resistance; belt tension affects resistance and wear | S32 p.1 | Verified | p.1 | — |
| 45 | L68 Grouser height strongly affects traction; slip 10–45 % | S34 §4.1 p.499 | Verified | "slip ratio in real application is from 10% to 45%" | — |
| | **Findings: dynamic / power models** | | | | |
| 46 | L71 Models needed for planning in elevation/terrain/accelerations; examples | S23 PDF p.1; S24 §I | Verified | S23 p.1 quote | — |
| 47 | L72 Three terrain parameters; motor saturation, power limits; hills | S24 Abstract | Verified | Abstract | — |
| 48 | L73 Closed-loop model "much better"; recommended | S24 Abstract | Verified | "closed-loop model is recommended for motion planning" | — |
| 49 | L74 K 0.00054 m, 0.0371, 0.4437/0.3093, α 1.5, asphalt 0.051 | S24 T II p.6 | Verified | Table II | — |
| 50 | L75 N = 2 → 31: 16.7 %, 9.9 %, 7.0 % | S24 §V p.6 | Verified | "variation from N = 2 to N = 31 is modest (16.7% for K, 9.9% for µsa, and 7.0% for µop)" | — |
| 51 | L76 Roll + slide makes modelling hard; other disadvantages listed | S23 PDF p.2 | Verified | p.2 "roll and slide at the same time"; "tires tend to wear out faster" | — |
| 52 | L77 Outer always consumes; inner consumes → generates → consumes | S23 PDF pp.5, 18, 20 | Verified | p.5 | — |
| 53 | L78 Coulomb power models wrong for larger radii; terrain-side, no per-motor power | S23 PDF p.4 | Verified | p.4 "can lead to incorrect predictions for larger turning radii … does not appear possible to quantify the power consumption of the left and right side motors" | — |
| 54 | L79 First-order powertrain model (τ, delay, EKF) matched encoder wheel speeds | S28 §4 Fig.5 p.21 | Verified | p.21 | — |
| | **Findings: online / learned estimation** | | | | |
| 55 | L82 Endo SCOG: gyro + encoders + empirical relation | S25 §II.B–D eq.(1)–(7) | Verified | eq.(1)–(7) | — |
| 56 | L83 Inverse kinematics with slip → wheeled path-following laws | S25 §IV.A eq.(11) | Verified | §IV.A | — |
| 57 | L84 Helmick IMU+VO KF vs kinematics → slip; heading + crab loops | S26 Abstract; §6.2 eq.(24)–(25) | Verified | Abstract, §6.2 | — |
| 58 | L85 χ² gate 5 dof, 11.07; discard and declare slip | S26 §5 eq.(22) p.13 | Verified | "when md ≤ t, with t = 11.07 … discarded" | — |
| 59 | L86 Galati scalar EKF on B_s; off in straight driving | S29 §1; §3.2 eq.(10)–(16) p.5 | Verified | p.5 "due to the lack of excitation" | — |
| 60 | L87 Çiloğlu EKF states; ICRs random walk; GPS+IMU; follows Pentzer; variable track width to pure pursuit | S30 §§1–2 | Verified | §2 "As specified by Pentzer [15], the ICR coordinates … remain…"; §4.2 | — |
| 61 | L88 16 t, 2.17 m; ICRs ≈ +1.7/−1.75 m; ≈3 m = 36 %; majority within ±3 m | S30 §6.2; T5 | Verified | "clustered around 1.7 m … −1.75 m"; "majority of the data is within the predicted bounds [−3 m, 3 m]"; "amplification of 36%"; Table 5 16,000 kg, 2.17 m | — |
| 62 | L89 Needs logical initial conditions; process noise by trial and error | S30 §7 | Verified | "The filter needs logical initial conditions to converge quickly" | — |
| 63 | L90 Liu EKF [x,y,θ,sL,sR]; RTK–IMU; slip updated only in measurement step | S31 §3.2.1 eq.(11) p.7 | Verified | eq.(11) | — |
| 64 | L91 No slip ground truth; judged by plausibility, inner track slips more | S31 §4.2 p.12 | Verified | "true slip ground-truth is not directly measurable in field conditions" | — |
| 65 | L92 IPEM: fit integrated prediction; low-frequency observations; EKF form | S28 Abstract; §1 pp.3–4; §2.4.2 | Verified | Abstract; §1 | — |
| 66 | L93 Crusher slip surfaces: forward, lateral, angular; gravity | S28 §6.1 p.30 | Verified | p.30 "predominantly a linear function of angular velocity" | — |
| 67 | L94 Angelova: framework; this paper learns slip vs slope per known terrain; ≈15 % | S27 Abstract; §III | Verified | Abstract "In this paper we focus only on the latter problem … about 15" | — |
| 68 | L95 Okawara: DD ignores slip; must be maintained online; linear model; offline net | S33 §1 pp.1–2 | Verified | p.1 "must be maintained online to adapt" | — |
| 69 | L96 Grass parameters separated; 0.03 s per step vs 0.1 s LiDAR | S33 §4.2.2 Fig.9 p.14 | Verified | p.14 "grass (uneven and soft) were isolated"; "entire process (0.03 s)" | — |
| | **Findings: other platforms' calibration / validation** | | | | |
| 70 | L99 CV-04 500 mm, 400 mm, 25 kg; mocap 9 mm, 30 fps; log-log least squares | S25 §III T I p.2873 | Verified | Table I "Horizontal accuracy 9[mm] … Frame rate 30[fps]" | — |
| 71 | L100 Path 15–30 cm arcs, 112.8 cm/s; end poses; gyro run inside path | S25 §V.C p.2875 | Verified | (22,188,2.02), (13,−6,3.15), (2,−3,3.15) | — |
| 72 | L101 Leica total station ±2 mm/±0.2°, four prisms; Mars yard, Mojave 25°, sandbox; VO < 2.5 % | S26 §7 pp.16–17 | Verified | "accuracy of ±2 mm in position and ±0.2° in attitude"; "less than 2.5%" | — |
| 73 | L102 10° slope: 0.54 → 0.05 m² | S26 §7.3 p.18 | Verified | "0.05 and 0.54 meters²" | — |
| 74 | L103 IMU + DGPS, 2 s intervals, holdout; 1.8 m → few cm; −72/−83/−90 % | S28 §6 p.26; §6.1 pp.27–28; Fig.12 p.30 | Verified | "mean error is reduced from 1.8 meters to a few…"; "72% and 83%"; "90%" | — |
| 75 | L104 Effective turn rate one third; four of six wheels; hill sideways | S28 §6.1 p.31 | Verified | "effective turn rate is only a third" | — |
| 76 | L105 28 trajectories, 2 mm repeatability, ≈75 % | S28 §3 p.19 | Verified | "repeatability of 2 mm … 28 different trajectories … reduced by about 75%" | — |
| 77 | L106 MBS validated; 7 % grade; 17.1 → 9.6 cm; sim 94 → 36 cm | S30 §§6.1–6.2, 7 | Verified | §6.2 "17.1 cm and 9.6 cm"; §7 "94 cm to 36 cm"; "7% longitudinal grade" | — |
| 78 | L107 Liu farmland, 2.5–5 m radii, 0.35/0.75 m/s, 3 repeats, RTK–IMU 2–3 cm; percentages | S31 Abstract; p.8; §4.2 p.12 | Verified | Abstract 78.1/50.6, 63.1/57.6 % | — |
| 79 | L108 Circles at 0.2 m/s, radii 10^−0.7–10^4 m; lemniscate | S23 PDF pp.19–22 §5; S24 §V | Verified | S23 §5 | — |
| 80 | L109 Galati four terrains, 120 Hz ROS, fingerprint, mud ≈72 % | S29 §4 pp.7–10 | Verified | p.7 "Fs = 120 Hz"; p.10 "success of about 72%" | — |
| | **Findings: how the vehicle moves (Q1)** | | | | |
| 81 | L112 Turning requires slippage; no-slip assumptions fail | S01 §I | Verified | §I | — |
| 82 | L113 Three ICRs on one line; same ω | S01 §II Fig.2 | Verified | §II | — |
| 83 | L114 x_ICRl,r = (αV − vy)/ωz, y_ICR = vx/ωz; α for inflation/belt tension | S01 §II eq.(2)–(5) | Verified | eq.(2)–(5) | — |
| 84 | L115 Tread ICRs bounded; kinematic motion, centrifugal neglected | S01 §II | Verified | "kinematic motion, in which centrifugal dynamics are" negligible | — |
| 85 | L116 3×2 matrix; symmetric = DD at tread ICRs | S01 §II eq.(6)–(8) Fig.3 | Verified | eq.(6)–(8) | — |
| 86 | L117 Tread ICRs dynamics-dependent, outside centrelines | S01 §II | Verified | "dynamics-dependent and always lie outside of the tread centerlines" | — |
| 87 | L118 No inverse → non-holonomic | S01 §II; §IV.B.2 | Verified | §IV.B.2 | — |
| 88 | L119 vy + x_ICR θ̇ = 0 non-integrable; tangent only when ω = 0 | S02 §2.1 eq.(6), (12) | Verified | §2.1 | — |
| 89 | L120 Wheels skid laterally, ICR may leave wheelbase, dynamic model needed | S36 §1 p.2632 | Verified | p.2632 "the ICR … may move out of the robot wheelbase, causing loss of motion stability … prescribes the use of a dynamic model" | — |
| 90 | L121 Assumptions: rigid, horizontal, < 10 km/h, no longitudinal slip, lateral force ∝ load | S36 §2 p.2633 | Verified | p.2633 assumptions 1–4 | — |
| 91 | L122 Front load (b/(a+b))mg/2, rear (a/(a+b))mg/2 | S36 §2.2 p.2633 | Verified | p.2633 Fz1 = Fz2 = b/(a+b)·mg/2; Fz3 = Fz4 = a/(a+b)·mg/2 | — |
| 92 | L123 Lateral forces μFzi sgn(ẏi); Mr eq.(5); centrifugal transfer neglected | S36 §2.2 eq.(3)–(5) p.2634 | Verified | p.2634 eq.(5); "At low speed, the lateral load transfer due to centrifugal forces … can be neglected" | — |
| 93 | L124 ẏ + d0θ̇ = 0, 0 < d0 < a | S36 §3.1 eq.(7) p.2634 | Verified | p.2634 | — |
| 94 | L125 Same ICR model for P3-AT, Jackal, tracked | S01 §I; S07 §3 eq.(1)–(3) | Verified | S07 §3 (Jackal) | — |
| 95 | L126 Power dissipation: ẏ = 0; ẋ independent of L; θ̇ = C(Sr−Sl); L=0 → 1/(2W) | S06 §III.A Remark Fig.2 | Verified | §III.A | — |
| 96 | L127 Flipper 0.42/0.27 m; 1.5, 1.27, 1.37, 2.38 | S06 T I; §IV.A–B; §V.A | Verified | Table I; "θ̇ = 2.38(Sr − Sl)"; "1.37" | — |
| 97 | L128 Forward velocity a little lower than predicted | S06 §V.A | Verified | "a little lower than predicted in all three models" | — |
| 98 | L129 R = (B/2)(vo+vi)/(vo−vi); slip version; "can only be assumed to be estimates" | S09 pp.9–10 eq.(7)–(9) | Verified | p.10 | — |
| 99 | L130 s = 1 − Vwx/(rω), α = arctan(Vwy/Vwx) | S05 §II eq.(4) | Verified | eq.(4) | — |
| 100 | L131 Per-side slip + x_ICR solved each step | S05 §III eq.(8)–(12) | Verified | "θt = {slt, srt, xICRt}" | — |
| 101 | L132 Point-turn power ≈ double | S09 Abstract | Verified | "approximately double" | — |
| | **Findings: effective width (Q2)** | | | | |
| 102 | L135 χ = L/(x_ICRr − x_ICRl), 0–1 | S01 §II eq.(9) | Verified | eq.(9) | — |
| 103 | L136 ICR coefficient χ = 2y0/B ≥ 1 | S04 §2.1 eq.(15); S05 §II eq.(1) | Verified | eq.(15); eq.(1) | — |
| 104 | L137 b̂ = 2y0 + α; vy = 0 | S03 §III.B eq.(8)–(9) | Verified | eq.(8)–(9) | — |
| 105 | L138 P3-AT χ asphalt 0.6949/0.7112/0.7612; concrete 0.7098/0.7053/0.7460 | S01 §IV.A T I–II | Verified | Tables I–II (50 psi, 20 psi, solid) | — |
| 106 | L139 Asphalt lower χ "due to greater friction"; solid more efficient | S01 §IV.C | Verified | "compact wheels provide better efficiency than pneumatic tires" | — |
| 107 | L140 Factory x_ICR 0.3 m vs 0.2 m | S01 §IV.B.1 | Verified | "α = 1 and xICR = 0.3 m … would have xICR = 0.2 m" | — |
| 108 | L141 χ 1.4662 (λ=0) → 1.4115 (λ=7); fit a=0.4728, b=0.0538 | S04 eq.(13); T3; eq.(44)–(45) | Verified | Table 3; eq.(45) | — |
| 109 | L142 χ almost constant 0.1–0.5 m/s | S04 §3.4 Fig.12b | Verified | "remains almost constant with increasing velocity v" | — |
| 110 | L143 Default χ = 1.5 | S04 §3.5 | Verified | "χ is a constant value of 1.5" | — |
| 111 | L144 Warthog b = 1.2 m; b̂ 4.46/3.08 m; α 0.94/0.86 | S03 T I | Verified | Table I (590 kg in §IV text) | — |
| 112 | L145 Ideal DD better on snow; wheel vs terrain deformation | S03 §IV; §V Fig.5 | Verified | "ideal differential drive performs better on snow"; §IV deformation | — |
| 113 | L146 Rotated more on snow; >30° grows faster | S03 §V Fig.7 | Verified | "over 30°, the curve … grows faster … on snow" | — |
| 114 | L147 Angular error peaks where one side ≈ 2× other | S03 §V Fig.8 | Verified | "either side's commanded wheel velocity is about twice that of the other side" | — |
| 115 | L148 Higher angular acceleration → lower ω (inference labelled) | S05 §V Fig.5 | Verified | §V; inference labelled | — |
| 116 | L149 Nomad 1.97 m; 4 m circle theory 0.11/0.19, used 0.08/0.21 → 4.2 m | S09 p.15 T1; p.32 T3 | Verified | Table 1; Table 3 "4.2 … 0.08 … 0.21" | — |
| 117 | L150 DRIVE: angular slip depends on angular command; > half IDD | S08 p.12 §V | Verified | p.12 | — |
| 118 | L151 Husky median angular slip 0.735/0.708/0.690 | S08 p.11 | Verified | p.11 | — |
| 119 | L152 Coefficient depends on geometry | S06 §III.A Remark | Verified | Remark | — |
| 120 | L153 Grousers 1.5 → 1.27 | S06 §IV.A–B | Verified | "θ̇ 1.27" | — |
| | **Findings: load, inertia, slope** | | | | |
| 121 | L156 Constant-ICR valid on hard horizontal terrain, low inertia; symmetry simplifications | S38 §II.A p.2 | Verified | "On hard horizontal terrain and under low inertia, tread ICRs … remain almost with constant local coordinates outside of the tread contact lines"; x̄l = −x̄r, ȳ = 0 | — |
| 122 | L157 Auriga-α 258 kg, 0.42 m, CoM −0.015/0.04 m; 441 pairs; ICRs −0.458/0.630/0.0482 m | S37 §II T I Fig.3 p.2 | Verified | Table I; "441 simulations"; "x̂ICRl = −0.458 m, x̂ICRr = 0.630 m and ŷICR = 0.0482 m" | — |
| 123 | L158 F_i = −M ω × v_CM; near zero straight and on-spot; V_l = −0.67134 V_r | S37 §III.A eq.(4)–(13) pp.2–3 | Verified | eq.(4); eq.(13) "H = 0.67134" | — |
| 124 | L159 η = (x_ICRr − x_ICRl)/L; ≈2.5 → ≈4 (Fig.7(c)); ωz RMSE 0.0637 vs 0.1433; simulation; experiments future work | S37 §III.B eq.(19); Fig.7(c); T II p.5; §V pp.5–6 | Verified | eq.(19); Fig. 7(c) axis 2.5–4.5; Table II 0.0637394 vs 0.143285 rad/s; "halved errors"; "Future work … experimental procedure" | — |
| 125 | L160 Slopes: ICRs change continuously; roll, pitch, turn direction; |Vr−Vl| little influence; slide down even on small inclines | S38 §II.B p.2 | Verified | "relevant and continuous changes"; "|Vr −Vl| seemed to have little influence"; "slide down, even on small inclines" | — |
| 126 | L161 Lazaro half-width 0.2 m, 0°–7.6°, level −0.44/0.42 m; distance constant except aggravated sliding | S38 §III.A Fig.2 p.3 | Verified | "xm = ym = 0.2 m"; "0°, 2.5°, 4.4° and 7.6°"; "−0.44 m … 0.42 m"; "remains relatively constant with the exception of the aggravated sliding down cases" | — |
| 127 | L162 λ1, λ2 from roll/pitch; J8 1090 kg, 5 psi, 567 kg, xm 0.6 m; −0.980/1.073/0.206 m | S38 §III; §IV T I pp.4–5 | Verified | p.4 "weights 1090 kg … maximum payload of 567 kg"; "(5-psi)"; Table I | — |
| 128 | L163 J8 ≈3.6° on-spot turn: "lateral-position" MSE 0.957 vs 0.006 | S38 §IV.C T II p.5 | Partly | Table II (Σδy²)/N 0.957 vs 0.006; δy is a global-frame y error, not lateral | See #7 |
| 129 | L164 Non-uniform loading: 25.6 % (4 m over 15.6 m); 11 ms vs 0.49 ms | S40 §8.1 p.18 | Verified | "vertical difference of 4 m over a 15.6 m distance, corresponding to a cumulative error of 25.6%"; "0.49 ms … 11 ms" | — |
| 130 | L165 At 0.8 m/s gravity dominated; centrifugal transfer not visible | S40 §8.1 p.18 | Verified | "gravity still dominates the loading and the effects due to centrifugal forces … is not visible" | — |
| 131 | L166 Warthog tyre pressure varies with payload and temperature; manufacturer radius overestimated | S08 §V pp.9–10 | Verified | p.10 "very low tire pressure … varies … with extra payloads and operating temperature … overestimated the real radius" | — |
| 132 | L167 330 kg, 1.45 m; B 1.9270–2.2259 m "significantly much larger"; slip differs below 20 m | S39 T1 p.6; T2 p.8; p.14 | Verified | Table 1 330 kg, 1.45 m; Table 2 min 1.9270 max 2.2259; p.14 "when R0 was below 20 m … larger differences"; 1.33–1.54 arithmetic correct | — |
| | **Findings: calibration experiments (Q3)** | | | | |
| 133 | L170 Spin test x_ICR ≈ (∫Vr − ∫Vl)/(2φ) | S01 §III eq.(11) | Verified | eq.(11) | — |
| 134 | L171 α ≈ 2d/(∫Vr + ∫Vl) | S01 §III eq.(12) | Verified | eq.(12) | — |
| 135 | L172 Misses CoM asymmetry, misalignment | S01 §III | Verified | "nor mechanical misalignments" | — |
| 136 | L173 N segments; 5 parameters; GA | S01 §III eq.(13) | Verified | eq.(13); GA | — |
| 137 | L174 RTK/DGPS < 1 cm, 5 Hz; ticks every 10 ms; joystick paths | S01 §IV.C | Verified | "1 cm … RTCM/RTK … 5 Hz"; "recorded every 10 ms"; "joystick controlled paths" | — |
| 138 | L175 8 curvatures × 5 speeds (40); LMS400; y0 eq.(25) | S04 §3.1 eq.(25); §3.2 | Verified | "40 experiments"; "Sick LMS400" | — |
| 139 | L176 Δy0 = 0.007 m; χ error < 1 % | S04 §§3.3–3.4 | Verified | "∆y0 = 0.007 m"; "maximum relative error is less than 1%" | — |
| 140 | L177 Lidar ICP; maximise range/coverage; separate trajectories | S03 §IV | Verified | §IV | — |
| 141 | L178 Mahalanobis loss; spatial horizon; zero-command outliers | S03 §IV eq.(13) | Verified | "spatial horizon … easy removal of outlier data … zero velocity commands" | — |
| 142 | L179 Short horizons; 2 m / 5 m; he = 2 m | S03 §V Fig.4 | Verified | "training horizons of 2 m or 5 m"; "he = 2 m" | — |
| 143 | L180 DRIVE 6 s steps; 1 transient + 2 steady windows; 20 Hz | S08 §I; p.8 Fig.5 | Verified | p.8 | — |
| 144 | L181 Unloaded calibration | S08 p.15 §VI.B | Verified | p.15 | — |
| 145 | L182 Min area 1.5 × 6 s at max speed; 20 × 45 m, 9 × 6 m | S08 p.15 §VI.B | Verified | p.15 | — |
| 146 | L183 2 m circle, 0.5 m/s, 0.5 rad/s, OptiTrack 120 Hz | S06 §V.A | Verified | "Optitrack motion capture system (120 Hz)"; "(0.5 m/sec) and angular (0.5 rad/sec)" | — |
| 147 | L184 Zuo online ICR parameters; fast convergence | S07 Abstract; §3; §6.1 | Verified | §3 "ξ cannot remain constant"; §6.1 | — |
| 148 | L185 Identifiability degenerate cases | S07 §5.2 Lemma 2 | Verified | Lemma 2 (i)–(iv) | — |
| 149 | L186 Pentzer EKF, 118 kg (via Baril) | S03 §II | Verified | "Pentzer et al. [11] tracked them individually using … on a 118 kg skid-steer robot" | — |
| 150 | L188 Focchi: parquet, mocap 200 Hz, [−10, 10] rad/s, 0.65 steps, 2 s, averaged, decision trees | S40 §§4.1–4.2 p.8 | Verified | p.8 "200 Hz"; "[−10, 10] rad/s with increments 0.65 rad/s"; "2 s to achieve a steady state"; "we chose to employ decision trees" | — |
| 151 | L189 Two-speed parametrisation avoids Moosavian singularity | S40 §4.2 p.8 | Verified | "singularity … when the turning radius R is equal to half the distance from the two wheels and the inner wheel speed becomes zero" | — |
| 152 | L190 Two-stage identification; lidarslam_ros2 ≈100 ms; eq.(13) | S38 §III.B eq.(13) p.3; §IV pp.4–5 | Verified | "two consecutive stages"; "every 100 ms, approximately"; eq.(13) J | — (eq.(13) is on p.4) |
| 153 | L191 Zhou: width overestimated until INS reference; slip ratio and true radius | S39 Abstract; p.8 | Verified | Abstract "width of UGV was much larger than that of actual value … INS … true turning radius" | — |
| | **Findings: calibration on grass** | | | | |
| 154 | L194 Jackal 16 kg dataset: grass Train3 977 s/764.1 m, CV3 2127 s/771.1 m, LD3 685 s/561.5 m; slip 0.08–0.13 m/s | S05 §IV T I p.4 | Verified | Table I rows Train3, CV3, LD3; "16kg" | — |
| 155 | L195 EDD5 grass 18.9 % angular, 6.2 % linear (asphalt 17.6/14.2, tile 21.1/11.6); GP 5.6/5.7; 1 s | S41 §IV.D T I | Verified | Table I; "we elect a 1-second prediction horizon" | — |
| 156 | L196 DRIVE on grass: Husky 75 kg 200 steps 0.63 km 23.4 min; Warthog 470 kg 200 / 2.14 km / 36.2 min; ICP + Xsens | S08 §IV T1 Fig.6 pp.9–10 | Verified | Table 1; p.9 "Xsens inertial measurement unit"; "(ICP)-based localization" | — |
| 157 | L197 Floor parameters on grass: 0.158 vs 0.028 m (≈6×); wood tiles ≈2× | S33 T2 p.14 | Partly | Values/wording correct; Table 2 is on p.10 | See #9 |
| 158 | L198 Crusher on dirt roads and tall dry grass on slopes | S28 §6.1 pp.27–28 | Verified | §6.1 | — |
| | **Findings: rotation centre (Q4)** | | | | |
| 159 | L201 y_ICRv ≈ 1 cm behind origin; only CoM | S01 §IV.C T I–II | Verified | "always about 1 cm behind of the frame origin, which confirms that it only depends on the center of mass" | — |
| 160 | L202 CoM side/front/rear effects; tyre pressure | S01 §II | Verified | §II | — |
| 161 | L203 e; −0.0213 to 0.0921 | S01 §II eq.(10); T I–II | Verified | Tables I–II | — |
| 162 | L204 x_ICR out of wheelbase → skid, unstable; x0 ∈ (−a, b) | S02 p.484 §3.1 eq.(60)–(61) | Verified | p.484 | — |
| 163 | L205 x0 = 0 ≈ two-wheel robot; tuning coefficient | S02 p.484 §3.1 | Verified | "selecting x0 = 0 makes the SSMR kinematics model similar to … two-wheel robot" | — |
| 164 | L206 Lateral velocity control needs ICR projection | S02 §2.1 | Verified | §2.1 | — |
| 165 | L207 Rabiee x_ICR offset solved each step | S05 §III Fig.1 | Verified | §III | — |
| 166 | L208 Warthog x_v −2.57/−2.71 m; near-symmetric | S03 T I; §V | Verified | Table I; "our vehicle is almost symmetric" | — |
| 167 | L209 Zuo X_v estimated; ideal X_v = 0 | S07 §3 eq.(2)–(3) | Verified | ξ = [0, b/2, −b/2, 1, 1] gives eq.(3) | — |
| 168 | L210 Symmetric tracked: zero lateral velocity on flat ground | S06 §III.A Remark | Verified | Remark | — |
| 169 | L211 base_link "typically at its main chassis and its rotational center" | S19 | Verified | L86 of source | — |
| 170 | L212 Footprint origin base_link; robot_radius centred at base_link | S20 | Verified | L9 of source | — |
| 171 | L213 Odometry/control from ICRs; eq.(16)–(17) | S01 §IV.B.2 | Verified | eq.(16)–(17) | — |
| | **Findings: accuracy of models** | | | | |
| 172 | L216 Trained models "vastly better" | S03 §V Fig.5 | Verified | "the trained models perform vastly better" | — |
| 173 | L217 Asymmetric/full linear best translation; symmetric best rotation, less overfitting | S03 §V Fig.6 | Verified | §V | — |
| 174 | L218 Order of magnitude in ≈half of 20; 1.93 vs 28.91 m on 632.64 m | S07 §6 T1 | Verified | Table 1 row 632.64 (1.9297 / 28.9106); "almost half of the tests" | — |
| 175 | L219 Δx, Δy MSE 0.00010/0.00011 vs 0.00183/0.00468 | S01 §IV.D T III | Verified | Table III | — |
| 176 | L220 Friction model best on long-distance, comparable on constant velocity | S05 §V | Verified | §V | — |
| 177 | L222 LIMO 4.36 kg chicane; parquet/soapy; −48 % (7.2→3.8 cm), −24 % (4.2°→3.2°); 55 %, 22 % | S40 T6 p.13; §7.2 p.14 | Verified | Table 6 m = 4.36; p.14 quotes | — |
| 178 | L223 Contrary to Moosavian, slip depends on absolute velocity | S40 §4.2 p.8 | Verified | "there is also a dependency on the vehicle's absolute velocity" | — |
| | **Findings: controllers / configs** | | | | |
| 179 | L226 wheel_separation = multiplier × separation in update() and on_configure() | S10 L142–145, 298–300 | Verified | L143 (update from L101); L298 (on_configure from L272) | — |
| 180 | L227 Wheel commands with corrected separation | S10 L257–260 | Verified | L257–260 | — |
| 181 | L228 Odometry uses corrected separation; ω = (R−L)/sep | S10 L302; S12 L81–84 | Verified | S10 L302; S12 L84 | — |
| 182 | L229 Multiplier doc; wheel_separation doc | S11 L18–24, 39–43 | Verified | L21; L42 | — |
| 183 | L230 wheels_per_side doc; Husky 1 | S11 L26–30 | Verified | L29 | — |
| 184 | L231 ROS 1 Noetic same scheme | S14 L296–299, 492–493 | Verified | L296; L492–493 | — |
| 185 | L232 Kinematics page no-slip DD only | S13 | Verified | "We assume that the wheels are rolling without slipping" | — |
| 186 | L233 Only three scalar multipliers | S10; S11 L39–53 | Verified | L39–53 | — |
| 187 | L234 A200 0.555, 1.875, wheels_per_side comment | S15 L16–20 | Verified | L16, L17, L20 | — |
| 188 | L235 J100 0.37559, 1.5 | S16 L16–20 | Verified | L16, L20 | — |
| 189 | L236 Noetic Husky 1.875 | S18 L26 | Verified | L26 | — |
| 190 | L237 husky humble-devel 0.512, 1.0 | S17 L16–20 | Verified | L16, L20 | — |
| 191 | L238 P3-AT Ticksmm / Revcount | S01 §IV.B.1 eq.(14)–(15) | Verified | eq.(14)–(15) | — |
| | **Recommended practice** | | | | |
| 192 | R1 Do not use ideal DD; effective width + slip factor | S03 §V Fig.5; S07 §3 | Verified | S03 §V; S07 §3 "significantly degraded" | — |
| 193 | R2 Start with symmetric two-parameter model | S03 §V | Verified | "only has two parameters … interesting choice" | — |
| 194 | R3 Spin + straight run | S01 §III eq.(11)–(12) | Verified | eq.(11)–(12) | — |
| 195 | R4 External reference | S01 §IV.C; S03 §IV; S04 §3.2; S06 §V | Verified | DGPS; ICP; LMS400; OptiTrack | — |
| 196 | R5 Varied paths, segments, pose-increment fit, separate train/eval | S01 §III eq.(13); S03 §IV | Verified | — | — |
| 197 | R6 Horizon matches use | S03 §V | Verified | "training horizon ht should be chosen in accordance with the application" | — |
| 198 | R7 Non-proportional, changing speeds | S07 §5.2 Lemma 2 | Verified | Lemma 2 | — |
| 199 | R8 Per terrain; "for a specific robotic task …" | S01 §III; S03 T I | Verified | S01 §III quote | — |
| 200 | R9 Radius-dependent width or online | S04 §3.4 eq.(44); S07 §6.1 | Verified | — | — |
| 201 | R10 Unloaded check; limit acceleration | S08 p.15 §§VI.A–B | Verified | p.15 | — |
| 202 | R11 Model closed-loop controllers + limits | S24 Abstract | Verified | Abstract | — |
| 203 | R12 Shear-displacement over Coulomb | S23 PDF pp.10, 20 | Verified | — | — |
| 204 | R13 Minimise integrated prediction error over the use horizon | S28 Abstract; pp.3–4 | Verified | — | — |
| 205 | R14 Online estimation: EKF (S29–S31) or gyro + empirical (S25); update only when turning | S25 §II.D; S29 §3.2; S30 §2; S31 §3.2.1 | Verified | S29 "lack of excitation" | — |
| 206 | R15 Sensible initial values | S30 §7 | Verified | "needs logical initial conditions" | — |
| 207 | R16 χ² gate on wheel odometry; rejection = slip | S26 §5 eq.(22) | Verified | — | — |
| 208 | R17 Constant ICRs only for hard level ground, low inertia; slopes: roll/pitch terms with on-incline turn | S38 §§II.A, III.B | Verified | §II.A; §III.B "single experiment on an inclined plane" | — |
| 209 | R18 Re-identify per surface; ≈6× worse on grass with floor parameters | S33 T2 p.14; S40 §7.2 | Partly | S33 Table 2 on p.10; S40 §7.2 re-identified for both surfaces | See #10 |
| | **Key numbers** | | | | |
| 210 | χ 0.6949–0.7612 asphalt | S01 T I | Verified | Table I | — |
| 211 | χ 0.7053–0.7460 concrete | S01 T II | Verified | Table II | — |
| 212 | α 0.9047–0.9464 | S01 T I–II | Verified | min 0.9047, max 0.9464 | — |
| 213 | y_ICRv −0.008 to −0.016 m | S01 T I–II, §IV.C | Verified | −0.0080 … −0.0159 | — |
| 214 | x_ICR 0.3 vs 0.2 m | S01 §IV.B.1 | Verified | — | — |
| 215 | χ 1.4662 → 1.4115, tile, 0.1–0.5 m/s | S04 T3 | Verified | Table 3; "faced with a tile" | — |
| 216 | χ(λ) fit, λ ∈ [0, 10] | S04 eq.(45) | Verified | eq.(45) | — |
| 217 | b̂ 4.46/3.08 m (b = 1.2 m), 590 kg | S03 T I | Verified | Table I; 590 kg in §IV text | — |
| 218 | α 0.94/0.86 | S03 T I | Verified | Table I | — |
| 219 | Flipper C 1.5/1.27/1.37/2.38; 2W 0.27, 2L 0.42 m | S06 T I, §§IV–V | Verified | Table I "0.42 m 0.27 m" | — |
| 220 | Multiplier 1.875 (0.555 m) / 1.5 (0.37559 m) | S15, S16 L16–20 | Verified | — | — |
| 221 | Multiplier 1.0 (0.512 m) | S17 L16–20 | Verified | — | — |
| 222 | Husky angular slip 0.735/0.708/0.690 | S08 p.11 | Verified | — | — |
| 223 | Point-turn power ≈ 2× | S09 Abstract | Verified | — | — |
| 224 | 1.93 vs 28.91 m; 632.64 m; cement brick + tile | S07 T1 | Verified | "(b,f)" = cement brick, ceramic tiles | — |
| 225 | δm 2/1/3 | S21 pp.61–63 | Verified | — | — |
| 226 | α 1.5 vinyl; > 2 concrete | S23 PDF p.6; S24 T II | Verified | — | — |
| 227 | K 0.00054 m | S24 T II | Verified | — | — |
| 228 | Rolling resistance 0.0371 / 0.051 | S24 T II | Verified | — | — |
| 229 | n 0.4811/0.6213/0.5094; a_l/a_r = −sgn(vl·vr)|vr/vl|^n; CV-04 specs | S25 §III.B T I | Verified | eq.(7) "−sgn(vl · vr)"; Table I | — |
| 230 | Equivalent track 1.269/1.252/1.329/1.159 m | S29 §2.1, T1 | Verified | Table 1 | — |
| 231 | ≈3 m vs 2.17–2.2 m (36 %) | S30 §6.2, T5 | Verified | — | — |
| 232 | Effective turn rate ≈ 1/3 | S28 p.31 | Verified | — | — |
| 233 | −72/−83/−90 %; 1.8 m → few cm | S28 §6.1 p.28 | Verified | — | — |
| 234 | χ² gate 11.07 | S26 p.13 | Verified | — | — |
| 235 | 0.05 vs 0.54 m², 4.5 m run, 10° sand | S26 §7.3 | Verified | "4.5 meter run … 10° slope" | — |
| 236 | VO < 2.5 % | S26 §7.1 | Verified | — | — |
| 237 | Slip prediction ≈ 15 % | S27 Abstract | Verified | — | — |
| 238 | 17.1 → 9.6 cm (−44 %), 7 % grade | S30 §6.2 | Verified | — | — |
| 239 | Liu −78.1/−50.6, −63.1/−57.6 % | S31 Abstract | Verified | — | — |
| 240 | Crawler slip values; up to 20 % climbing | S32 pp.11–12, 15 | Verified | p.11; p.12 "increases from 10% to 20%"; p.15 | — |
| 241 | Slip range 10–45 % | S34 §4.1 | Verified | — | — |
| 242 | Auriga-α ICRs −0.458/0.630/0.0482 m; η = 1.088/0.42 ≈ 2.59 (arithmetic) | S37 §II p.2 | Verified | arithmetic correct | — |
| 243 | η ≈ 2.5–4 vs inertial force | S37 Fig.7(c) p.5 | Verified | Fig. 7(c) | — |
| 244 | J8 −0.980/1.073/0.206 m; half-width 0.6 m; 1090 kg; smooth concrete | S38 T I p.5 | Verified | Table I; "xm = 0.6 m" | — |
| 245 | Lazaro −0.44/0.42 m; half-width 0.2 m; melamine | S38 §III.A p.3 | Verified | "melamine board" | — |
| 246 | Back-calculated width 1.9270–2.2259 m vs 1.45 m | S39 T2 p.8 | Verified | Table 2 | — |
| 247 | 25.6 % (4 m over 15.6 m) | S40 §8.1 p.18 | Verified | — | — |
| 248 | EDD5 grass 18.9 % / 6.2 % | S41 T I | Verified | — | — |
| 249 | Wheel slip velocity on grass 0.08–0.13 m/s; 16 kg | S05 T I p.4 | Verified | Train3 0.13, CV3 0.12, LD3 0.08 | — |
| 250 | Front/rear wheel normal load formula | S36 §2.2 p.2633 | Verified | p.2633 | — |
| | **How it is tested** | | | | |
| 251 | Spin in place; no criterion | S01 §III eq.(11) | Verified | — | — |
| 252 | Straight run; no criterion | S01 §III eq.(12) | Verified | — | — |
| 253 | Joystick paths + RTK/DGPS, GA; lower Δx, Δy, J; Δφ equal | S01 §§III–IV T III | Verified | Table III | — |
| 254 | 8 curvatures × 5 speeds, laser; χ error < 1 %; dead reckoning | S04 §§3.1–3.5 | Verified | — | — |
| 255 | Command coverage, ICP; 2 m horizon errors | S03 §§IV–V | Verified | — | — |
| 256 | DRIVE 6 s steps; slip transfer functions | S08 §§III–V | Verified | — | — |
| 257 | Open-loop 2 m circle, mocap 120 Hz | S06 §V.A | Verified | — | — |
| 258 | Online ICR; RTK-GPS; drift and RMSE | S07 §6 T1–2 | Verified | "RTK-GPS … centimeter level" | — |
| 259 | 4/8/12 m circles, DGPS; geometric slip R_des/R_act | S09 pp.31–32 T4 | Verified | Table 4; "ratio of the desired radius and the actual radius" | — |
| 260 | Mocap 9 mm/30 fps; three odometry methods; SCOG (2, −3) cm | S25 §§III, V | Verified | — | — |
| 261 | Leica total station; VO < 2.5 %; path offset | S26 §7 | Verified | — | — |
| 262 | IMU + DGPS; 2 s intervals; holdout | S28 §6 | Verified | — | — |
| 263 | Circles 0.2 m/s, 10^−0.7–10^4 m; lemniscate | S23 §5; S24 §V | Verified | — | — |
| 264 | Four terrains; EKF track; mud ≈72 % | S29 §4 | Verified | — | — |
| 265 | GPS/IMU, 7 % grade, 16 t; pure pursuit vs classical | S30 §6.2 | Verified | — | — |
| 266 | RTK–IMU farmland; radii, speeds, repeats; no slip ground truth | S31 §4.2 | Verified | — | — |
| 267 | LiDAR–IMU–wheel across terrains; RTE/ATE | S33 §4 | Verified | — | — |
| 268 | 441 track-speed pairs; ωz RMSE 0.0637 vs 0.1433 | S37 §§II, IV, T II | Verified | Table II | — |
| 269 | Level manual driving, on-spot slope turn, LiDAR SLAM; three validation paths | S38 §§III.B, IV | Verified | "Three experiments" (rotation, 8-shaped, longer trajectory) | — |
| 270 | Sprocket grid 0.65 rad/s, 2 s, mocap 200 Hz; regressor test fit; chicane tracking | S40 §§4, 7.2 | Verified | p.8 "test set … r2 score"; §7.2 chicane | — |
| 271 | INS steady turns; recalculated width 1.4601 m vs 1.45 m (0.7 %) | S39 T3 p.10; pp.8–14 | Verified | p.10 "1.4601 m … error of 0.7%" | — |
| 272 | 1 s moving-horizon; kinematic vs GP; % position error, velocity MAE | S41 §IV.D T I–II | Verified | Tables I–II | — |
| | **Common mistakes** | | | | |
| 273 | Ideal DD on skid steer: "much higher" errors | S03 §V Fig.5 | Verified | "much higher than for any of the trained models" | — |
| 274 | DD-style odometry: ≈10× drift in ≈half of 20 | S07 §6 T1 | Verified | — | — |
| 275 | Spin-only calibration misses asymmetry | S01 §III | Verified | — | — |
| 276 | Constant width across surfaces / radii | S01 T I–II; S03 T I; S04 T3 | Verified | — | — |
| 277 | Unobservable motions | S07 §5.2 Lemma 2 | Verified | — | — |
| 278 | Horizon mismatch | S03 §V Fig.4 | Verified | — | — |
| 279 | ICR outside wheelbase | S02 p.484 | Verified | — | — |
| 280 | DRIVE wear: 13 km at 4 m/s², gear skipping, 30°, 12 → 2 mm | S08 p.15 §VI.A | Verified | p.15 | — |
| 281 | Wrong wheel_separation | S11 L18–24 | Verified | L21 | — |
| 282 | Pure Coulomb friction | S23 PDF pp.4, 10 | Verified | — | — |
| 283 | Open-loop dynamic model | S24 Abstract | Verified | — | — |
| 284 | Wheeled odometry on tracked vehicle: (22, 188, 2.02) vs (2, −3) | S25 §V.C | Verified | — | — |
| 285 | Gyro only: ran inside path | S25 §V.C | Verified | — | — |
| 286 | Updating effective track in straight driving | S29 §3.2 | Verified | — | — |
| 287 | No χ² consistency check | S26 §5 | Verified | — | — |
| 288 | Offline-trained learned model on changing terrain | S33 §1 | Verified | — | — |
| 289 | Slip estimate without ground truth | S31 §4.2 | Verified | — | — |
| 290 | Level ICRs on slopes; J8 "lateral" MSE 0.957 vs 0.006 | S38 §IV.C T II | Partly | ICRHK "unable to predict the slide down"; δy is a global-frame error | See #8 |
| 291 | ICR spread assumed independent of inertia; constant ICRs ≈ double errors | S37 §IV T II | Verified | "halved errors for the proposed kinematics" | — |
| 292 | Back-calculating width without external velocity | S39 p.8 | Verified | p.8 "B values were remarkably larger than 1.45 m" | — |
| 293 | Slip by turn radius alone → singular when inner track stops | S40 §4.2 p.8 | Verified | — | — |
| 294 | Manufacturer wheel radius under load | S08 pp.9–10 | Verified | p.10 | — |
| | **Disagreements** | | | | |
| 295 | Constant width enough? Mandow/Baril constant; Wang χ(λ); Rabiee acceleration; Zuo "cannot remain constant" | S01 §II; S03 §III.B; S04 §3.4; S05 §V; S07 §3 | Verified | S07 §3 quote | — |
| 296 | ROC model: Wang improves dead reckoning; Baril ROC unlike others | S04 §3.5; S03 §V Fig.6 | Verified | S03 "except for the ROC-based one, all models tend to perform similarly" | — |
| 297 | Identification vs first principles; Dixit "without requiring experimental identification", similar to tuned | S01 §III; S03 §IV; S06 §I, §V.A | Verified | S06 "(similar to the tuned model) despite it not using any experimental identification" | — |
| 298 | Speed dependence: Wang, Baril Fig.8, Mandow, Focchi vs Moosavian | S04 §3.4; S03 Fig.8; S01 §II; S40 §4.2 p.8 | Verified | — | — |
| 299 | Husky 1.875 vs 1.0 (0.512 vs 0.555 m) | S15 L20; S18 L26; S17 L16–20 | Verified | — | — |
| 300 | Symmetric vs asymmetric | S03 §V; S01 T III | Verified | — | — |
| 301 | Coulomb vs shear displacement (Yu on Caracciolo, Kozłowski) | S23 PDF pp.4, 7, 8, 20 | Verified | p.3–4 and p.7 "modeled using Coulomb friction in Caracciolo et al. (1999); Kozlowski & Pazderski (2004)" | — |
| 302 | Surface and slip: Endo n; Galati track; Okawara grass | S25 §III.B; S29 T1; S33 Fig.9 | Verified | — | — |
| 303 | Slip as random-walk states vs functions vs neural network | S31 §3.2.1; S30 §2; S28 §6.1; S33 §1 | Verified | — | — |
| 304 | Kinematic vs dynamic for control: Caracciolo vs Martínez "implicitly take into account some dynamical effects" | S36 §1 p.2632; S38 §I p.1; S37 §V | Verified | S38 p.1 quote | — |
| 305 | Is ICR spread constant? Mandow, Martínez level; S37 varies; S38 slopes | S01 §II; S38 §II.A; S37 Fig.7(c); S38 §III.A | Verified | — | — |
| 306 | Textbooks: Siegwart includes as degenerate; Lynch excludes | S21 p.61; S22 p.513 | Verified | — | — |
| | **Open questions** | | | | |
| 307 | L371 No width for rubber-belt tracked robot on grass; list of tracked sources; Martínez 2005, Moosavian 2008 not downloaded | S06; S25; S29 | Partly | Main claim holds; list omits S30, S31, S32, S37, S40 | See #11 |
| 308 | L372 Wong / Wong & Chiang cited by S04, S05, S23, S25; not summarised beyond S23 | S04; S05; S23 PDF pp.7–8; S25 | Verified | S04 "following Wong's model [16]"; S05 ref. [11]; S25 ref. [2] | — |
| 309 | L373 Controllers apply multiplier to both paths by design; no source evaluates one-path use | S10; S14 | Verified | S10 L143/L298; S14 L296/L492 | — |
| 310 | L374 Nav2 only says "typically" at rotational centre | S19 | Verified | L86 | — |
| 311 | L375 Speed above 0.5 m/s not tested by S04; Baril higher concrete speeds not reached | S04; S03 §V | Verified | S03 "Higher speeds were not reached for the concrete trajectory because of safety purposes" | — |
| 312 | L376 No source explains 1.875 / 1.5 | (none) | Verified | Configs S15–S18 give values without rationale (process note) | — |
| 313 | L377 Only Endo on artificial turf; Galati no grass | S25; S29 | Verified | S25 turf; S29 sand/gravel/mud/asphalt | — |
| 314 | L378 Terramechanics sources straight-line / firm-ground; no K or friction for turf | S32; S34; S23 | Verified | — | — |
| 315 | L379 Online estimators validated only indirectly | S31 §4.2; S30 §6.2 | Verified | S30 "sensor readings … clustered around 1.5 m, whereas the … estimates … 1.7 m" | — |
| 316 | L380 Pentzer and Martínez 2005 paywalled; known via S03 and S30 | S03; S30 | Verified | S03 §II; S30 §2 | — |
| 317 | L381 Payload sweep: indirect evidence only | S36 eq.(5); S37; S08 pp.9–10; S38; S40 §8.1 | Verified | S36 eq.(5) ∝ mg; S38 p.1 "uneven loading" on slopes | — |
| 318 | L382 Grass data sources; "Pentzer reportedly tested on grass" | S05; S41; S08; S33; S28 | Partly | Data sources correct; Pentzer-grass remark uncited | See #12 |
| 319 | L383 Nikitin and Kitano & Kuma known only through citations | (none) | Partly | Kitano & Kuma in S06 refs; Nikitin in no source | See #13 |
| 320 | L384 Huskić et al. 2019 not downloaded (bot check) | (none) | Verified | Process note; Huskić is cited in S03 ref. [1] with the stated venue | — |
| 321 | L385 2023 off-road slippage survey not downloaded, not cited | (none) | Verified | Process note | — |

## Corrections applied (2026-09-28)

Editor pass applying this review and `SOURCE_AUDIT.md`. Every flagged location was re-checked in the source file before it was changed. After this pass, no Partly supported or Not supported claims remain and no source fails the checklist. README Status is set to **Verified**.

### Claims
| # | Change in README.md |
|---|---|
| 1 | Summary 3: masses now cited where they appear: [C2-S01, sections II, IV.A, IV.C] and [C2-S03, Table I, section IV]. |
| 2 | Summary 5: "motion capture" now supported by [C2-S25, sections II.D, III] (S25 §III uses a motion-capture camera). |
| 3 | Foundational, Campion (C2-S35): "that the textbooks use" dropped; now "Source of the five-type classification … by degree of mobility and degree of steerability". |
| 4 | Foundational, Caracciolo (C2-S36): "First" dropped; now "Introduces the operational ICR constraint … that Kozłowski and Pazderski (C2-S02) and later controllers build on". |
| 5 | Foundational, Bekker / Janosi–Hanamoto / Steeds / Kitano–Kuma: split into three rows. Bekker and Janosi–Hanamoto known through C2-S32 and C2-S34; Kitano & Kuma known only as a reference in C2-S06; Steeds kept (SCOPE lists it) but marked as not cited by any downloaded source and not used. The uncited descriptions ("classical Coulomb skid-steer theory", "first transient tracked-vehicle dynamics") were removed. |
| 6 | Foundational, Pentzer 2014: "First" dropped; now "Online EKF estimation of track ICRs that later work (C2-S30) follows". |
| 7, 8 | J8 slope results (Findings and Common mistakes): "lateral" error reworded to "mean squared position error along the global y axis" [C2-S38, section IV.C, Table II]. |
| 9, 10 | Okawara floor→grass result (Findings and Recommended practice 18): citation corrected to [C2-S33, section 4.1, Table 2, p. 10]. |
| 11 | Open question on tracked robots on grass: list of tracked-vehicle sources extended with C2-S30, C2-S31, C2-S32, C2-S37 and C2-S40. |
| 12 | Open question on grass: uncited "Pentzer et al. 2014 reportedly tested their ICR EKF on grass" removed. |
| 13 | Open question: Nikitin removed (not in any downloaded source); Kitano & Kuma now cited as known only through [C2-S06]. |
| minor | [C2-S38, eq. (13)] page corrected p. 3 → p. 4. [C2-S29, Table 1] now "p. 7, text p. 8". |
| S30 re-check | After replacing C2-S30 with the publisher PDF, the 16 t vehicle's 2.17 m track width was found in section 5, Table 4 (vehicle parameters), not Table 5 (lateral errors). The two citations "[C2-S30, section 6.2; Table 5]" and "[C2-S30, section 6.2, Table 5]" are now "[C2-S30, section 5, Table 4; section 6.2]". Other C2-S30 claims (44 %, 17.1 → 9.6 cm, ~7 % grade, ICRs ≈ 1.7 / −1.75 m, bounds [−3 m, 3 m], 36 % amplification, "logical initial conditions", trial-and-error tuning) were re-checked in the PDF at the cited sections. |
| S04 re-check | After replacing C2-S04 with the publisher PDF, the section, equation, table and figure numbers cited (sections 2.1, 3.1–3.5; eqs. (13), (15), (25), (44), (45); Table 3; Fig. 12b) were confirmed to be unchanged, along with the quoted numbers (χ 1.4662 / 1.4115, 0.4728, 0.0538, Δy₀ = 0.007 m, < 1 %, "remains almost constant"). |
| Open questions | Two gaps named in SOURCE_AUDIT coverage were added: track tension and contact-length / gauge ratio (subtopic 4), and no agreed benchmark protocol (subtopic 9). |
| Foundational | C2-S28 Seegmiller et al. 2013 added to the Foundational references table (SCOPE lists it as foundational). |

### Sources
| ID | Change |
|---|---|
| C2-S01 | Added doi:10.1109/IROS.2007.4399139. |
| C2-S02 | Removed the stray leading space byte so the file starts with `%PDF` (`file` now reports PDF; 20 pages per pdfinfo). Added the journal site and noted that no DOI is assigned. |
| C2-S04 | Markdown copy replaced with the publisher PDF (`wang_2015_skid_steer_laser_kinematics.pdf`, 22 pp.; from mdpi-res.com because www.mdpi.com returned Access Denied). Link and access date updated. The `.md` file was deleted. |
| C2-S06 | Noted that the kept copy is v2 and that the latest arXiv version is retitled "The Kinematics of Tracked Vehicles via the Power Dissipation Method". |
| C2-S07 | Added Springer Proceedings in Advanced Robotics vol. 20 (*Robotics Research*), 2022, pp. 741–756, doi:10.1007/978-3-030-95459-8_45 (confirmed via Crossref). |
| C2-S08 | Volume and pages 2:380–399 confirmed via Crossref. |
| C2-S09 | Level stated as "A (examined thesis, published as CMU RI technical report)", the same grading L2 uses for its examined thesis. |
| C2-S10, S11, S12, S14 | Level C → B (official ros-controls project source, STANDARDS §2). Pinned to commits bde7fe7e (S10, S11), b4d79ab1 (S12) and e2aaddfa (S14). The links now point to those commits. |
| C2-S13 | Pinned to commit f912c6c2 (2026-03-29). The row states that this is master-branch development documentation, not the Humble release. |
| C2-S15, S16 | Pinned to clearpath_common commit 842d57e5 (2024-01-18). |
| C2-S17 | Pinned to husky commit 95c5df9d (2023-04-17). |
| C2-S18 | Pinned to husky commit 07229c29 (2024-03-18). |
| C2-S25 | Added doi:10.1109/IROS.2007.4399228 (Crossref). |
| C2-S27 | Added Orlando, pp. 3324–3331, doi:10.1109/ROBOT.2006.1642209 (Crossref). |
| C2-S30 | Text copy replaced with the publisher PDF (`ciloglu_2025_tracked_pure_pursuit_slip_ekf.pdf`, 24 pp.). Link and access date updated. The `.md` file was deleted. |
| C2-S33 | Added the RAS doi:10.1016/j.robot.2025.104929. |
| C2-S41 | Corrected "a 4-page version" to "a 7-page version". |

For every pinned file in S10–S18, the saved copy was compared byte for byte with the file at the pinned commit, and all of them matched. The years in the file names were left unchanged; the commit dates are given in the Sources table.

### Final mechanical check
`file` reports a type that matches the extension for all 41 files in `sources/`: 30 PDFs, plus Markdown, YAML, C++ and `.rst` files. Every file has exactly one Sources-table row. Every source ID cited in the README exists in the table. The IDs run from C2-S01 to C2-S41 with no gaps or duplicates. Evidence levels: 29 A, 7 B, 5 C.
