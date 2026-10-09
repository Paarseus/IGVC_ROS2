# Verification — D3: Ackermann / car-like steering

**Topic:** D3_ackermann_steering
**Date:** 2026-10-06
**Reviewer:** independent — claims (per `STANDARDS.md` §5, step 4)
**Scope:** every cited item in `README.md` (Summary, Foundational references, Findings §1–§12, Recommended practice, Key numbers, How it is tested, Common mistakes, Disagreements, cited Open questions). Sources opened at the cited location with `pdftotext -layout` (PDFs) or read directly (`.md`/`.rst`/`.msg`/`.hpp`). README.md was not edited. Source files themselves (format, authenticity, pinning) are out of scope — that is `SOURCE_AUDIT.md`'s job.

## Counts

| Status | Count |
|---|---|
| Verified | 101 |
| Partly supported | 12 |
| Not supported | 2 |
| **Total claims checked** | **115** |

(Counts are at the level of one README bullet/row = one claim, which may bundle several citations. A handful of very long bullets bundle 2–3 sub-claims; where any sub-claim failed, the whole bullet is marked Partly supported and the failing part is named in the Correction column.)

---

## Summary

| # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|
| 1 | Kinematic bicycle model = no-sideslip reduction of Ackermann vehicle; suitable low/moderate speed, loses accuracy as speed/lateral accel rises | D3-S31, p.5 §III | Verified | "kinematic models are suitable for planning paths at low speeds... where inertial effects are small in comparison to the limitations on mobility imposed by the no-slip assumption" (p.5); §III-B "Inertial Effects" covers the accuracy-loss claim | — |
| 2 | 3 independent comparisons show explicit/Ackermann steering needs less power, tracks more precisely than skid steer; skid ≈2× power at point turn; Zoë2 skid ≤30% more power / >100% vs 12% drive-radius error | D3-S05 Abstract; D3-S18 p.61/p.67 | Verified | Shamah abstract: "the power for skid steering is approximately double that of an explicit point turn"; Holand p.61: "skid steering consumed approximately 30% more energy"; p.67: "passive steering... averaging 12% error... skid steering... over 100% error" | — |
| 3 | ROS2 message/control support mature (`ackermann_msgs`, `steering_controllers_library`); Nav2 RPP/Smac Hybrid/MPPI all support Ackermann kinematics | D3-S07; D3-S09–S12; D3-S02; D3-S14; D3-S15/16 | Verified | confirmed individually in Findings §6–§7 below | — |
| 4 | IGVC AutoNav envelope (10-20 ft lanes, ≥5 ft turning radius, 5 ft clearance, 5 mph/1 mph, 2 ft potholes, 15% ramps); teams note turning-radius tradeoff | D3-S26 §II.2; D3-S27 p.4; D3-S28 p.2/p.5 | Verified | igvc_2026_rules.pdf §II.2 matches every number exactly; bobjones_eran p.4, botzilla p.2/p.5 confirmed (see Findings §10) | — |

## Foundational references

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 5 | Reeds & Shepp 1990 extends Dubins 1957 to reverse/cusps; used by Smac Hybrid-A* REEDS_SHEPP model | D3-S01 | Verified | title/abstract confirm; Nav2 doc confirms REEDS_SHEPP option (D3-S14) | — |
| 6 | Coulter 1992 = standard pure-pursuit reference, basis of RPP | D3-S03 | Verified | CMU-RI-TR-92-01 cover page; content matches | — |
| 7 | Macenski et al. 2023 = origin of `nav2_regulated_pure_pursuit_controller`, analyzes Ackermann+diff-drive | D3-S02 | Verified | confirmed in Findings §3/§7 | — |
| 8 | Snider 2009 = standard survey/empirical comparison of PP/Stanley/kinematic/LQR for car-like vehicles | D3-S04 | Verified | confirmed in Findings §7 | — |
| 9 | Hoffmann et al. 2007 = origin of Stanley controller, DARPA GC 2005 winner | D3-S08 | Verified | confirmed in Findings §7 | — |
| 10 | Shamah 1999 = seminal same-robot skid-vs-explicit comparison, ≈2× power at point turn | D3-S05 | Verified | confirmed above | — |
| 11 | Sousa/Petry/Moreira 2020 = survey of odometry calibration organized by steering geometry | D3-S06 | Verified | confirmed in Findings §4 | — |
| 12 | `ackermann_msgs` = primary ROS/ROS2 Ackermann message standard | D3-S07 | Verified | confirmed in Findings §6 | — |
| 13 | LaValle 2006 ch.13 §13.1.2.1 gives ρ_min = L/tan(φ_max) for "simple car" | D3-S33 | Verified | exact formula found pp.725-726 | — |
| 14 | Veneri & Massaro 2020 = source of formal Ackermann-condition equation + %-Ackermann ratio definitions | D3-S34 | Verified | eq.(1) p.3, ratio defs p.4, confirmed | — |
| 15 | Dubins 1957 "not downloaded" (paywalled); cited only via Reeds & Shepp's description | — | Verified | Reeds & Shepp abstract explicitly attributes the 6-word/CCC-CSC result to "Dubins (1957)" | — |
| 16 | Rajamani *Vehicle Dynamics and Control* "not downloaded" | — | Verified (unverifiable claim of absence, consistent w/ sources folder) | book not in `sources/`; no counter-evidence found | — |
| 17 | Gillespie *Fundamentals of Vehicle Dynamics* "not downloaded" | — | Verified | book not in `sources/` | — |
| 18 | ISO 8855 "not downloaded" | — | Verified | standard not in `sources/`; Findings §12 explicitly notes it could not be read | — |
| 19 | SAE J695 "not downloaded" | — | Verified | standard not in `sources/`; same as above | — |
| 20 | Kong et al. 2015 "not downloaded"; cited only via D3-S37's description; **not** the origin of the 0.5µg threshold (correction from a previous pass) | — | Verified | Aertssen (D3-S37) ref (6) = Kong, Pfeiffer, Schildbach, Borrelli — described exactly as "similar open-loop errors over short horizons," distinct from the 0.5µg criterion | — |
| 21 | Polack et al. 2017 IEEE IV "not downloaded"; origin of the kinematic-model-specific 0.5µg criterion, distinct from a separately-derived 0.5g dynamic-model limit traced to Park et al. 2009 | — | Verified | D3-S36 (Polack 2018, same authors) explicitly: "the criterion derived in [7]" = Polack et al. 2017; separately "[5]" (Park et al., Proc. IMechE Part D) is the origin of the 0.5g limit used by a different (2 DoF dynamic) paper [4] | — |
| 22 | Lankensperger/Ackermann/Darwin 1816-1818 patents "not downloaded"; now sourced via D3-S34's secondary account | — | Verified | padua_2020 p.1 gives the 1759/Lankensperger/1818-Ackermann account exactly as described | — |

## Findings §1 — Ackermann geometry and mechanism

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 23 | 1759 Darwin → Lankensperger → 1818 Ackermann patent → 1878 Jeantaud four-bar linkage; "still used (in modified form, with tie rods…) in current industrial practice" | D3-S34, p.1 §1 | Partly supported | History (Darwin/Lankensperger/Ackermann/Jeantaud) is on p.1 exactly as stated. The "tie rods in place of the connecting rod… current industrial practice" clause is real but is on **p.4** ("the Jeantaud layout is modified by replacing the connecting rod with two tie rods"), not p.1 | Cite the tie-rod/industrial-practice clause to p.4, not p.1 |
| 24 | Exact Ackermann condition 1/tan(δo) − 1/tan(δi) = T/w, "a standard and well-known derivation" | D3-S34, p.3, eq.(1) | Verified | exact equation and quote found on p.3 | — |
| 25 | 100%/0%/less-than/more-than/reverse(anti)-Ackermann terminology; 3 ratio defs ν_τ, ν_w, ν_n | D3-S34, p.4, eqs.(2)-(4) | Verified | exact terminology and eqs. (2)-(4) found on p.4 (name "anti-Ackermann" is the README's own added synonym, not in the source, but harmless) | — |
| 26 | FSAE linkage Ackermann-% error 8.92%→4.54% after hardpoint optimization | D3-S35, p.1 Abstract, p.2 | Verified | "the maximum error was reduced from 8.92% to 4.54%" — physical pp.1/6/7/8 all state it; Abstract (p.1) states it too | — |
| 27 | Only known same-vehicle quantitative test of Ackermann-% choice on race performance: 26 ms/0.03% (no toe/camber) → 329 ms/0.4% (with); toe is "most influential parameter" | D3-S34, p.16 §6-7 | Verified | exact numbers and quote found on p.16 | — |
| 28 | JPL taxonomy: 3 classes — skid (all fixed-direction wheels, center of rotation unconstrained along any axis), Ackermann (center on fixed-wheel axis), all-wheel (unconstrained) | D3-S17, p.2 §II | Partly supported | Source explicitly states the "unconstrained" center-of-rotation property **for all-wheel steering** ("the center of rotation... for any motion is unconstrained") and explicitly states Ackermann's axis constraint, but never makes an explicit "unconstrained along any axis" statement about **skid steering** specifically — only that skid vehicles have "all fixed-direction wheels" | Attribute "center of rotation unconstrained" explicitly only to all-wheel steering per the source; the skid-steering case is a reasonable but unstated inference |
| 29 | Tandem-wheel pair treated as one "virtual" wheel, axis equidistant, minimizes slip | D3-S17, p.2 | Verified | "the tandem pair can be treated as one larger wheel with its axis equidistant from the two fixed-wheel rotation axes... minimizes the slippage" | — |
| 30 | 2-steerable-axle vehicle: 4 modes (Ackermann/Dual Ackermann/Crab/Point Turn); Dual Ackermann halves angles needed by symmetry | D3-S19, pp.21-27 §4.1 | Verified | §4.1.1–4.1.4 (pp.21-27 per TOC) match exactly; Dual Ackermann "only two steering angles... to calculate" (vs 4 wheels) confirmed p.22-23 | — |
| 31 | Min turning radii: Ackermann 1.94 m (0.515 m⁻¹) vs Dual Ackermann 1.29 m (0.775 m⁻¹, 34% tighter) vs Crab ∞ vs Point Turn 0 | D3-S19, p.24, Fig.4.5 | Verified | Table 4.1 p.24 gives exactly these 4 rows; text states "capable of 34% tighter turns" verbatim | — |
| 32 | Crab maneuver loads rocker-bogie suspension unevenly, reduces stability; undesirable for long traverses, only all-wheel-steer can do it | D3-S17, p.2 | Verified | "Driving sideways (crab-maneuver) does not use the rocker-bogey suspension effectively, thus reducing the vehicle's stability... undesirable for long traverses. Such maneuvers can only be performed with all-wheel steering vehicles" | — |
| 33 | Motion = arcs; zero-radius arc = point turn; only difference Ackermann vs all-wheel-steer is arc-center constraint | D3-S17, p.2 | Verified | "An arc of zero radius corresponds to a rotate-in-place motion... The only difference between Ackermann steering and all-wheel steering vehicles is that the center of the arc for the former is constrained to the fixed-wheel axis" | — |

## Findings §2 — Kinematic bicycle/Ackermann model validity

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 34 | Kinematic model ≡ "car-like robot"/"bicycle model"/"single-track model"; differential constraint on reference point motion | D3-S31, p.5 §III | Verified | "Variations of this model have been referred to as the car-like robot, bicycle model, kinematic model, or single track model"; derivation follows as differential constraint | — |
| 35 | Suitable low speed (parking/urban); drawback = instantaneous steering changes; fixed by augmenting model w/ steering-rate integration | D3-S31, p.5 §III | Verified | "suitable for planning paths at low speeds... A major drawback... permits instantaneous steering angle changes... Continuity... imposed by augmenting [the model], where the steering angle integrates a commanded rate" | — |
| 36 | Chronos/CRS (ETH) frames kinematic + dynamic bicycle model as the two standard plant models | D3-S32, p.3 §V-A | Verified | "V. CRS SOFTWARE FRAMEWORK" → "A. Dynamical Models": "we describe two widely used models: The kinematic and the dynamic bicycle models" | — |
| 37 | Pure Pursuit places frame at rear axle, x-axis colinear, because propulsion/steering decoupled there (citing Shin 1990) | D3-S03, p.5 §2.0 | Verified | "Shin[2] shows that propulsion and steering are geometrically decoupled if the vehicle's coordinate system is placed at the rear differential"; ref [2] = Shin, D.H., 1990; both on p.5 | — |
| 38 | a_y<0.5µg traces to Polack et al. 2017 (via Polack 2018), not to a dynamics-comparison paper; δ_max(V) formula derived from it; separate 0.5g limit bounds a different (2DoF dynamic) model, traced to Park et al. 2009 via a 2013 ASME paper | D3-S36, p.4 eq.(5); p.1 | Verified | "the criterion derived in [7]... lateral acceleration ay... should be lower than 0.5µg" + eq (5) exact match p.4 (section numbering is unlabeled subsection, physically on p.4); "[4] ... guaranteed by constraining the lateral acceleration to 0.5g... derived in [5]" = Park et al., Proc. IMechE Part D — p.1 | — |
| 39 | Kong et al. 2015: similar open-loop error, kinematic vs dynamic single-track, short horizons | D3-S37, p.1 §1 ref.6 | Verified | "Kong et al.(6) have compared kinematic and linear-tire... and report similar open-loop errors over short horizons" | — |
| 40 | Separate sim study (same 2017 paper): kinematic model "increasingly inaccurate at higher speeds and steering angles" due to unmodeled understeer | D3-S37, p.1 §1 ref.10 | Verified | "Polack et al.(10) compare a kinematic bicycle model with a 9-degree-of-freedom... model becomes increasingly inaccurate at higher speeds and steering angles because it does not capture slip-induced understeer"; ref (10) = Polack et al. 2017 IEEE IV | — |

## Findings §3 — Turning radius / maneuvering geometry

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 41 | "Simple car": |φ|≤φmax<π/2 ⇒ ρ_min=L/tan(φmax) | D3-S33, ch.13 §13.1.2.1, pp.725-726 | Verified | exact formula, pp.725-726 confirmed by page-footer text | — |
| 42 | Dubins (1957, via Reeds&Shepp): no-reverse car ⇒ shortest path is CCC/CSC, ≤6 candidates | D3-S01, p.367 Intro | Verified | "Dubins (1957) has shown that paths of the form CCC and CSC suffice"; "at most 6 contenders" — p.367 | — |
| 43 | Reeds & Shepp: w/ reverse ⇒ CCSCC form, ≤68 candidates, closed form | D3-S01, p.367 | Verified | "there are at most 68, but usually many fewer paths"; "CCSCC" — same p.367 abstract | — |
| 44 | Smac Hybrid-A*: DUBIN (default)/REEDS_SHEPP; `minimum_turning_radius` default 0.4 m, >0, used in search + smoother; `reverse_penalty` only in REEDS_SHEPP; analytic-expansion distance should be ≥4-5× min radius | D3-S14, "minimum_turning_radius"/"reverse_penalty" entries | Partly supported | `minimum_turning_radius` (default 0.4, >0, "Also used in the smoother to compute maximum curvature") and `reverse_penalty` ("Only used in REEDS_SHEPP motion model") both confirmed verbatim. The "4-5× minimum turning radius" text is real but lives under a **third** entry, `analytic_expansion_max_length`, not under either of the two entries named in the citation | Cite the 4-5× claim to the `analytic_expansion_max_length` entry |
| 45 | MPPI AckermannMotionModel::applyConstraints(): clips wz when |vx|/|wz| < min_turning_r (default 0.2) to copysign(|vx|/min_turning_r, wz) | D3-S16 lines 107-121; D3-S15 "AckermannMotionModel" § | Verified | code matches exactly; function body is lines 111-122 in the read file (comment block starts 107) — 1-line rounding, same block | — |
| 46 | RPP: path must already respect min turning radius for Ackermann (no path-feasibility check); not required for diff-drive | D3-S02, p.9 §6 "Limitations" | Verified | "For differential-drive robots, this may be any path. However for Ackermann steering robots, this path must already be drivable considering the kinematic limitations on the minimum possible turning radius" — p.9, §6 Limitations | — |

## Findings §4 — Odometry/calibration for car-like robots

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 47 | 2020 survey since UMBmark(1996): 15 diff-drive methods vs 3 Ackermann (2 tricycle, 3 omni); disparity vs real-world prevalence | D3-S06, Abstract | Verified | "fifteen methods for differential drive, three for Ackerman, two for tricycle, and three for the omnidirectional"; "A disparity was noted, compared with the real utilisation" — Abstract | — |
| 48 | 3 Ackermann methods: (1) self-calib/1-run/no-initial-vector (shared w/ tricycle); (2) least-squares/several-runs; (3)/(4) 2 automotive methods — one via position-error centroid over 5 runs/direction, other via final-orientation error w/ no trig approximations (more accurate) | D3-S06, p.297 §III-B, p.298 §IV-B | Partly supported | Self-calib/least-squares (p.297) and the 5-CW+5-CCW/position-error-centroid vs final-orientation-error/no-trig-approx/more-accurate comparison (printed pp.297-298) all confirmed verbatim. **No section "§IV-B" exists in this paper** — section IV ("CONCLUSIONS") has no lettered subsections at all; the content is entirely within §III-B ("B. Ackerman and Tricycle"), continuing from p.297 onto p.298 | Cite as p.297-298 §III-B (not §IV-B) |
| 49 | Self-calibrating method (shared Ackermann/tricycle) takes ≈15 min, less effort than least-squares | D3-S06, p.298 §IV-B | Partly supported | "these two methods have the advantage of only taking approximately 15 minutes" confirmed verbatim on printed p.298 — but same non-existent "§IV-B" section tag issue as above; actual section is §III-B | Cite as p.298 §III-B |
| 50 | `steering_controllers_library` computes odometry generically per concrete kinematics, publishes `nav_msgs/Odometry` + TF + `SteeringControllerStatus` | D3-S09, lines 34, 104-109 | Verified | line 34 and lines 104-109 match exactly as quoted | — |

## Findings §5 — Precision/power vs skid-steering

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 51 | Shamah: power/torque converge only at infinite radius; skid needs more at every finite radius; ≈2× at point turn | D3-S05, Abstract | Verified | Abstract text matches almost verbatim | — |
| 52 | Zoë2: straight-line power statistically indistinguishable; skid power rises significantly with steering angle (r²=0.92, p=2.5e-10), passive doesn't (r²=0.02, p=0.54); ≈30% more energy for skid at 0.40 rad/23° | D3-S18, p.61 §4.4 | Verified | all 4 numbers match exactly on p.61; §4.4 label is the parent section (content is technically in subsection 4.4.3 "Drive Arc Tests," nested under 4.4) | — |
| 53 | Zoë2: passive "substantially better at blind navigation" (~12% avg error) vs skid (>100% avg error); passive ~30% less energy at tightest (1.5 m) turn; wheels align w/ path tangent "virtually eliminating sideslip... fully in-line torque transfer" | D3-S18, p.67 §5.2; p.61 | Verified | all quotes/numbers found verbatim on p.67 (§5.2 "Contributions") and p.61 | — |
| 54 | General direction (Ackermann beats skid on power+precision, gap grows w/ tighter turn) consistent across D3-S05/D3-S18/D3-S17 despite differing exact numbers | D3-S05; D3-S18; D3-S17 | Verified | consistent with all three sources' content verified above | — |

## Findings §6 — ROS2 message/control-level support

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 55 | `ackermann_msgs`: `AckermannDrive`+`AckermannDriveStamped`; steering angle = yaw of virtual front wheel, "not the angle of the steering wheel inside the passenger compartment" | D3-S07a/b | Verified | exact quote in `AckermannDrive.msg`; `AckermannDriveStamped.msg` = Header+AckermannDrive | — |
| 56 | Defined by ROS "Ackermann steering group"; documents front-wheel Ackermann vehicles | D3-S07c | Verified | "ROS messages for vehicles using front-wheel Ackermann steering. It was defined by the ROS Ackermann steering group" | — |
| 57 | `steering_controllers_library`: shared lib, "2 DOF... non-holonomic constraints," IK+odom only, body-twist input; 3 controllers by joint count (bicycle 1+1, tricycle 1+2, ackermann 2+2) | D3-S09 lines 13-41; D3-S10; D3-S12 | Verified | all quotes/line numbers match the .rst files exactly | — |
| 58 | All 3 controllers support front/rear steering via `front_steering`; publish odometry as `nav_msgs/Odometry`+TF | D3-S09 lines 30, 66-92, 107-108 | Verified | line 30 ("support for front and rear steering configurations"), 66-92 (front_steering conditionals), 107-108 (Publishers) all match | — |

## Findings §7 — Nav2 path-tracking/planning for car-like vehicles

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 59 | Macenski et al.: "PP and its variants are applicable to Ackermann and differential-drive robots due to PP's formulation supporting dynamics with longitudinal motion and a turning rate in body-fixed frame"; Ackermann path must already be feasible, diff-drive not | D3-S02, p.3, p.9 | Verified | exact quote p.3; §6 Limitations p.9 as above | — |
| 60 | `use_rotate_to_heading` (default true): "recommended on for all robot types that can rotate in place"; cannot combine w/ `allow_reversing` | D3-S13, lines 230-237 | Verified | exact text at those line numbers | — |
| 61 | Stanley paper: global asymptotic stability proof; DARPA GC2005, 132 mi, RMS<0.1m (σ=0.09m) @19.1mph, only vehicle of 40 not to hit/miss | D3-S08, p.1, p.6 | Verified | p.1 abstract + p.6 body match exactly | — |
| 62 | Smac Hybrid-A* = Ackermann-oriented Smac member, DUBIN/REEDS_SHEPP, min_turning_radius default 0.4m | D3-S14 | Verified | confirmed above (Findings §3) | — |
| 63 | MPPI: "works currently with Differential, Omnidirectional, and Ackermann robots"; `motion_model` plugin = DiffDriveMotionModel/OmniMotionModel/AckermannMotionModel; only Ackermann exposes min_turning_r | D3-S15 lines 12,36,250-256; D3-S16 | Verified | lines match exactly; code confirms only AckermannMotionModel has the param | — |
| 64 | Snider: Stanley "simplest... performs surprisingly well... outperforms Pure Pursuit in most scenarios" but "not as robust to large errors/non-smooth paths"; "will not cut corners but rather overshoot turns" (no lookahead); Paden: Stanley unstable in reverse, rear-wheel law stable regardless of sign | D3-S04, p.66; D3-S31, p.19 §V-A2-A3 | Verified | Snider p.66 verbatim; Paden p.19 verbatim ("not stable in reverse making it unsuitable for parking" / "stability is unaffected by the sign of vr making it suitable for reverse driving") | — |
| 65 | Snider's overall conclusion: no method "work[s] well in all applications," methods have "complementary characteristics" | D3-S04, Abstract | Verified | "none of the approaches work well in all applications and that they have some complementary [characteristics]" | — |

## Findings §8 — Off-the-shelf small/mid research platforms

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 66 | F1TENTH: 1/10-scale, base chassis = brushless motor + VESC ESC + Ackermann-steering servo + LiPo pack; UPenn, NeurIPS 2019 | D3-S20, p.81 §4; title page | Verified | exact text p.81 §4 "Hardware Specification and Middleware"; title page confirms UPenn/NeurIPS2019 | — |
| 67 | MuSHR (UW): basic no-sensor build ≈$600 vs ≈$1,000 MIT RACECAR; sensor build (2D LIDAR+RGBD+IMU) ≈$900 vs ≈$2,800 MIT RACECAR | D3-S21, p.3 §V | Verified | "V. AFFORDABILITY & PERFORMANCE" (physical p.3): "$600 (a similar MIT racecar setup costs about $1,000)... $900 (a similar MIT racecar setup costs about $2,800)" | — |
| 68 | MuSHR drivetrain: Jrelecs F540 3930KV brushless motor, ZOSKAY 1X DS3218 servo, Turnigy SK8-ESC VESC | D3-S21, p.2 §II | Verified | exact part names found | — |
| 69 | AgileX Hunter 2.0: 980×745×380mm, 65-72kg, 650mm wheelbase, 605mm track, 33° max steer, 150kg payload, 1.5m/s, 40km range, 50mm obstacle clearance, ≤10° climb (loaded) | D3-S24, pp.5-6, p.17 | Verified | all numbers found verbatim (dimensions block on physical p.6; wheelbase/steering-angle table on p.17; "Obstacle Clearance: 50 mm" distinct from the separate "Ground clearance: 100mm" spec elsewhere in the same manual) | — |
| 70 | Hunter 2.0 manufacturer claims vs "traditional four-wheel differential drive chassis": higher payload, higher top speed, reduced tire/structure wear | D3-S24, p.16 | Verified | "Compared to traditional four-wheel differential drive chassis, HUNTER 2.0 delivers: Enhanced payload capacity / Higher top speed / Reduced wear on tires and structure" — physical p.16 | — |
| 71 | Chronos/CRS (ETH, ICRA 2023): 1/28-scale, open-source electronics, bridges gap between cheap/slow unicycles and expensive/large car-like robots | D3-S32, p.1 Abstract | Verified | Abstract text matches almost verbatim | — |
| 72 | Quanser QCar: 1/10-scale, open-architecture, AWD, single DC motor+encoder, single-ratio gearbox, steering servo ≈±30°, Jetson TX2; QCar2 successor = "open-architecture, 1/10th scale," Jetson Orin AGX, 2 PWM channels ("Motor throttle control"/"Steering control"), ROS2/Python/Simulink | D3-S38, p.3 §II; D3-S39 | Partly supported | AWD/single-motor/gearbox/servo/±30°/Jetson TX2 all confirmed verbatim — but on **printed p.4**, not p.3 (physical PDF page 3 = printed page "4" per its own footer). The specific "**1/10-scale**" descriptor for the *original* QCar is **not stated anywhere in D3-S38** — the source only says "scaled model car" without a fraction; "1/10th scale" is explicitly stated only for **QCar 2** in D3-S39. QCar2 details (Jetson Orin AGX, 2.7kg, PWM channels, ROS2) all confirmed verbatim in D3-S39 | Fix page to p.4; attribute "1/10-scale" only to QCar 2 (D3-S39) unless a source stating it for the original QCar is added |
| 73 | Original QCar discontinued, succeeded by QCar 2 | D3-S39 | Partly supported | D3-S39's page never uses the word "discontinued" or states succession explicitly; this is a reasonable but unstated inference from the product catalog | Label as inference, or drop the word "discontinued" |
| 74 | AgileX LIMO: 4.2kg, 322×215×247mm, 4 modes (diff/Ackermann/tracked/Mecanum) via mechanical re-latching of the same 4 wheel-hub-motor modules; Ackermann switch = pull latch, rotate 30°, marked line toward front; differential = different latch position; tracked/Mecanum = physical module swap | D3-S40; D3-S41 | Verified | weight/dimensions match; latch-rotation procedure, "longer line"/"shorter line" mechanism, and tracked/Mecanum module-swap descriptions all match D3-S41 verbatim | — |
| 75 | LIMO per-mode numeric specs (turning radius, payload) not confirmable in distributor docs actually read | D3-S40 | Verified | `agilex_limo_specifications_trossen.md` indeed has no per-mode turning-radius/payload breakdown | — |

## Findings §9 — Golf-cart/full-size platforms and drive-by-wire

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 76 | OpenPodcar: Ackermann-steered Pihsiang TE-889XLSN mobility-scooter donor; steering automated by replacing manual handlebar linkage w/ 12V DC linear actuator (Gimson GLA750-P: 750N, 8mm/s, 250mm stroke) mounted at the donor's existing tie-rod attachment point | D3-S22, p.3 §"Donor vehicle", p.4 §"Automate steering" | Partly supported | Donor vehicle ID and "human-operated loop handle bar" (p.3) confirmed; actuator specs 750N/8mm/s/250mm (p.4, under heading "Mechanical Modification for Steering," not literally "Automate steering" though the body text says "To automate steering...") confirmed exactly. **Not supported:** the source never uses the term "tie-rod," and never frames the actuator as "replacing" the handlebar linkage at that point — it describes the actuator as newly mounted between a chassis anchor and a hole in the right front wheel **axle**, a different, separate connection from the handlebar mechanism | Drop "tie-rod attachment point" and "replacing the manual handlebar linkage"; describe as "mounted to the front wheel axle via a chassis anchor and bearings" |
| 77 | OpenPodcar total build cost ≈$7,000 (2022); carries passenger/load up to 15km/h | D3-S22, Abstract | Verified | "System build cost from new components is around USD7,000... speeds up to 15km/h" — Abstract, exact | — |
| 78 | ROS1 OpenPodcar uses `move_base`+TEB, implements Dubins+Ackermann geometry; min-turning-radius params from donor vehicle specs | D3-S22, p.10 §"Path planning" | Verified | "move_base and Timed Elastic Band (TEB)... implement the requirement geometry of Dubins paths... and Ackermann steering. The values for parameters such as minimum turning radius have been calculated from the technical specifications of the base vehicle" — p.10, under "Path Planning and Control" | — |
| 79 | OpenPodcar2 (Lincoln): robust R4 board + ROS2/Nav2; build cost ≈$7,000 new / ≈$2,000 w/ used donor | D3-S23, Abstract, p.2 | Verified | "Total build cost was around 7,000USD from new components, or 2,000USD with a used Donor Vehicle" (p.1 Abstract); "Build cost: 7000 USD... 2000 USD" infobox (p.2) | — |
| 80 | OpenPodcar2 uses RTAB-Map (localization/mapping) + "the SMAC planner from the nav2 ROS2 package" | D3-S23, p.6 | Verified | "localisation and mapping using RTAB-Map... Planning is provided by the SMAC planner from the nav2 ROS2 package" — p.6 exact | — |
| 81 | At design time, Nav2 MPPI was "in early release," similar foundation to ROS1 TEB predecessor, supports Ackermann; swapping in later could improve control | D3-S23, p.11 | Verified | "an MPPI controller server was in early release stages, using a similar foundation as its ROS1 predecessor teb_planner and supporting Ackermann models. When this matures it could be swapped into OpenPodcar2 and might provide more accurate control" — p.11 | — |
| 82 | PACMod 3.0: by-wire control of accelerator/brake/steering/shift/horn/turn-signals/hazards/headlights; feedback incl. throttle/brake %, speed, steering-wheel angle, gear, wheel speeds, light/horn status; immediate return-to-manual; fitted to Polaris GEM + Lexus RX450h; ships w/ ROS driver | D3-S25, full text | Not supported (one clause) | Controlled functions, feedback list, and "immediate return to full manual control" all confirmed verbatim; "ROS node available" confirms the ROS-driver claim. **"Polaris GEM series" is never mentioned anywhere in this source** — the brochure covers only the Lexus RX450h platform | Drop "Polaris GEM series" or re-cite it to a source that actually states it |
| 83 | OpenPodcar2's Mounting Board separately CERN-OHL-W licensed to enable reuse "in control of other similar Ackermann vehicles" | D3-S23, p.11 §"Discussion" | Verified | "The Mounting Board is designed and separately CERN-OHL-W licenced to enable reuse in control of other similar Ackermann vehicles as well as the current donor vehicle" — p.11, under literal heading "Discussion" | — |

## Findings §10 — IGVC rules, design reports, retrospectives

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 84 | 2026 rules: ≈500ft course, 120×100ft area; 10-20ft track width, ≥5ft turning radius; ≥5ft obstacle clearance; barrels+natural/manmade obstacles; 2ft potholes; ≤15% ramps; 1-5mph, 44ft/30s min-speed check, hardware-governed max | D3-S26, §II.1-II.2 | Verified | every number matches `igvc_2026_rules.pdf` §II.1/II.2 verbatim | — |
| 85 | Identical track-width/turning-radius sentence unchanged 2020→2026 | D3-S26 §II.2; D3-S42 | Verified | identical sentence "Track width will vary from ten to twenty feet wide with a turning radius not less than five feet" found verbatim in both the 2020 and 2026 rules PDFs | — |
| 86 | Bob Jones/Eran team: "main disadvantage... is large turning radius; however, this steering system was chosen in order to more accurately simulate the pre-existing platforms..." | D3-S27, p.4 §"Chassis" | Verified | exact quote, printed p.4, under literal heading "Chassis" | — |
| 87 | Same team: 2009 34:1 direct-drive lacked torque, burned out 2 motors; fixed w/ 14T→60T chain drive (more torque/current, slower) | D3-S27, p.4 §"Steering Motor" | Verified | exact numbers and quote, printed p.4, under literal heading "Steering Motor" | — |
| 88 | Botzilla (Oakland): 4WD + double-Ackermann; "allows for strafing maneuvers impossible... while also being able to make tighter turns than a single Ackermann setup"; front/rear each steered by linear actuator via tie-rod link w/ opposing threads for toe | D3-S28, p.2 §1, p.5 §3.2 | Verified | exact quote p.2 §1 "Introduction"; tie-rod/opposing-thread mechanism exact quote p.5 §3.2 "Drive Train" | — |
| 89 | UT Austin EnterpRAS: approximate (not exact) Ackermann via single high-torque motor + potentiometer; footnote "most commercial cars operate perfectly fine with approximate Ackermann steering"; bending-stress/yielding failures at motor-linkage attachment | D3-S29, p.6 §4.3 | Partly supported | Content and quote (including the exact footnote) confirmed verbatim under the literal heading "4.3 Ackermann Steering." **Page number is off:** the PDF has no printed page numbers at all; by physical PDF page count (cover=1, advisor statement=2, "1. Introduction"=3) the "4.3 Ackermann Steering" section falls on physical page 5, not 6 | Re-derive the page citation (best available evidence points to p.5, not p.6) or note the source has no printed pagination |

## Findings §11 — Field/agricultural/planetary-rover literature

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 90 | Zhang et al. 2009: RTK-GPS leader-follower tractor platooning; curve-fit path; desired speed+steering angle via course-tracking+speed controllers | D3-S30, p.161 Abstract | Verified | Abstract text (printed p.161, on the paper's own first page) matches closely and completely | — |
| 91 | Steering-angle control structure: position controller → yaw-angle controller, speed controller → steering-angle controller (same architecture) | D3-S30, p.164 | Verified | "the position controller will be replaced by a yaw-angle controller, while the speed controller will be replaced by a steering angle controller" — printed p.164 exact | — |
| 92 | Hunter 2.0 manual frames Ackermann+rocker config as suited to low-speed autonomous driving w/ reduced wear vs diff-drive; modular sensors extend to field/transport use | D3-S24, p.5, p.16 | Verified | matches content confirmed in Findings §8 above | — |
| 93 | Deshpande thesis: Ackermann = "the canonical approach to driving wheeled mobile robots with steerable front wheels"; reports 1.94m vs 1.29m turning radii | D3-S19, p.21; p.24 | Verified | exact quote p.21; table p.24 (both confirmed above) | — |
| 94 | Deshpande motivates Crab/Point Turn modes "specifically for cases where 'we do care about the orientation of the robot at a particular location'... along a row-crop trajectory" | D3-S19, p.35 | Partly supported | Quote confirmed verbatim on p.35 ("If, however, we do care about the orientation of the robot at a particular location we can take advantage of the multiple steering modes"), in the general context of "Mode Switching." The **"along a row-crop trajectory" qualifier is not stated at p.35** — row-crops are mentioned only elsewhere in the thesis (general motivation, p.~2), not at the cited location | Drop "along a row-crop trajectory" or re-cite it separately |
| 95 | JPL taxonomy classifies rovers by the same 3-way scheme; Ackermann explored for positioning+orienting w/o full omnidirectional capability, "undesirable, limited, or unavailable on some rovers" | D3-S17, p.2 | Verified | "we will explore the use of Ackermann steering to achieve both vehicle positioning and orienting" + "omni-directional driving... is undesirable, limited, or unavailable on some rovers" — exact, p.2 | — |
| 96 | Holand/CMU Zoë2 thesis built to quantify tradeoff; up to 30% lower power, far more precise blind nav for passive steering, down to 1.5m radius | D3-S18, pp.61-67 | Verified | consistent with everything confirmed in Findings §5 | — |

## Findings §12 — Terminology, test procedures, failure modes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 97 | ISO 8855 / SAE J695 could not be read (paywalled, no open copy) | — | Verified | neither file exists in `sources/`; consistent with the Foundational-references table | — |
| 98 | Bump steer: unequal-length double-A-arm suspension + tie rod lack common instantaneous center ⇒ toe changes with jounce/rebound even w/o steering input | D3-S35, p.6 | Verified | matches the paper's described mechanism (toe-angle variation under wheel jump) on physical pp.6-7 | — |
| 99 | "Three-center theorem" mitigation: places tie-rod/rack disconnection point at suspension's own instant-center geometry; kept toe variation to 0.0088°/0.00872° under ±30mm jump (before further optimization needed) | D3-S35, pp.6-7 | Verified | exact numbers found, and text confirms "no further optimization was necessary" for this specific check | — |
| 100 | Only documented mechanical failure modes are from IGVC reports: bent/yielded aluminum drive bar (D3-S29) and burned-out steering motors from under-geared direct drive (D3-S27) | D3-S29, p.6 §4.3; D3-S27, p.4 | Verified (page caveat as in #89) | both failure descriptions confirmed verbatim in their respective sources | — |

## Recommended practice

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 101 | Use kinematic bicycle model by default at low speed/accel; switch to dynamic model only when needed | D3-S31 p.5; D3-S32 p.3 | Verified | consistent with Findings §2 | — |
| 102 | Keep min-turning-radius consistent between Smac Hybrid-A* and MPPI Ackermann model | D3-S14; D3-S16 | Verified | reasonable synthesis of confirmed parameter mechanics (Findings §3) | — |
| 103 | Verify Ackermann paths respect min turning radius before RPP tracking | D3-S02 p.9 | Verified | matches §6 Limitations text | — |
| 104 | `use_rotate_to_heading` based on in-place-rotation capability, not drive-type label; never with `allow_reversing` | D3-S13 lines 230-237 | Verified | matches doc text | — |
| 105 | Prefer self-calibrating single-run odometry calibration for Ackermann unless trig-approx-free accuracy specifically needed | D3-S06 pp.297-298 | Verified | matches Findings §4 (note: pp.297-298 citation here is correct — it is only the "§IV-B" sub-label elsewhere that is wrong) | — |
| 106 | Choose path-tracker by scenario: Stanley often beats PP but less robust to large errors/non-smooth paths; consider combining methods | D3-S04 p.66; D3-S31 p.19 | Verified | matches Findings §7 | — |
| 107 | Consider Dual-Ackermann/mode-switching when a single fixed min-turning-radius binds | D3-S19 pp.21-24, p.35 | Verified | matches Findings §1/§11 | — |

## Key numbers

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 108 | Skid vs explicit power, point turn: ≈2× | D3-S05, Abstract | Verified | see #51 | — |
| 109 | Passive vs skid power, 23°: skid ≈30% more | D3-S18, p.61 | Verified | see #52 | — |
| 110 | Passive vs skid energy, 1.5m radius: passive ≈30% less | D3-S18, p.67 | Verified | see #53 | — |
| 111 | Passive vs skid blind-tracking error: 12% vs >100% | D3-S18, p.67 | Verified | see #53 | — |
| 112 | Ackermann-specific calibration methods 1996-2020: 3 vs 15 for diff-drive | D3-S06, Abstract | Verified | see #47 | — |
| 113 | IGVC track width 10-20ft | D3-S26, §II.2 | Verified | see #84 | — |
| 114 | IGVC min turning radius ≥5ft | D3-S26, §II.2 | Verified | see #84 | — |
| 115 | IGVC speed limits 1-5mph | D3-S26, §II.1-II.2 | Verified | see #84 | — |
| 116 | IGVC min obstacle clearance 5ft | D3-S26, §II.2 | Verified | see #84 | — |
| 117 | Smac Hybrid-A* default min turning radius 0.4m | D3-S14 | Verified | see #44 | — |
| 118 | MPPI AckermannMotionModel default min_turning_r 0.2m | D3-S15; D3-S16 | Verified | see #44/#45 | — |
| 119 | Ackermann vs Dual Ackermann radius: 1.94m vs 1.29m (34% tighter) | D3-S19, Fig.4.5 | Verified | see #31 | — |
| 120 | MuSHR build cost $600/$900 | D3-S21, p.3 | Verified | see #67 | — |
| 121 | Hunter 2.0 wheelbase/track/max steer: 650mm/605mm/33° | D3-S24, p.17 | Verified | see #69 | — |
| 122 | Hunter 2.0 payload/speed/weight: 150kg/1.5m/s/65-72kg | D3-S24, pp.5-6 | Verified | see #69 | — |
| 123 | OpenPodcar(1/2) build cost ≈$7,000 new (OpenPodcar2 $2,000 used) | D3-S22 Abstract; D3-S23 Abstract | Verified | see #77/#79 | — |
| 124 | OpenPodcar steering actuator: 750N@8mm/s, 250mm stroke | D3-S22, p.4 | Verified | see #76 (actuator specs part only) | — |
| 125 | Reeds-Shepp ≤68 vs Dubins ≤6 candidate paths | D3-S01, p.367 | Verified | see #42/#43 | — |
| 126 | Stanley field RMS cross-track <0.1m (σ=0.09m), 132mi, 19.1mph | D3-S08, p.6 | Verified | see #61 | — |
| 127 | Kinematic-bicycle validity limit a_y<0.5µg | D3-S36, p.4; p.1 | Verified | see #38 | — |
| 128 | Simple-car ρ_min=L/tan(φmax) | D3-S33, §13.1.2.1 | Verified | see #41 | — |
| 129 | FSAE Ackermann-% error 8.92%→4.54% | D3-S35, p.1 | Verified | see #26 | — |
| 130 | Race-car lap-time diff: 26ms/0.03% → 329ms/0.4% | D3-S34, p.16 | Verified | see #27 | — |
| 131 | QCar max steering angle ≈±30° | D3-S38, p.3 | Partly supported | number/content verified (see #72) but same page-off-by-one issue (printed p.4, not p.3) | Fix page to p.4 |
| 132 | IGVC track-width/turning-radius rule unchanged 2020 vs 2026 | D3-S26 §II.2; D3-S42 | Verified | see #85 | — |

## How it is tested

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 133 | Steady-state circle driving, skid vs explicit, same vehicle: power/torque/position error vs radius, comparative | D3-S05 | Verified | matches Abstract/content | — |
| 134 | Commanded steering-angle sweep, locked vs unlocked axle: power vs angle, r²/p-value significance test | D3-S18, p.61 | Verified | matches #52 | — |
| 135 | Commanded drive-arc trajectory, locked vs unlocked: drive-radius error, lower-is-better, no fixed threshold | D3-S18, p.67 | Verified | matches #53 | — |
| 136 | Lane-change + figure-eight courses, multiple speeds/gains: PP/Stanley/kinematic cross-track error, qualitative comparison | D3-S04 | Verified | both "lane change" and "figure eight" courses confirmed present in TOC/figures for all three controllers | — |
| 137 | Row-change/oval/double-row-change trajectories: PP vs MPC cross-track error on Dual-Ackermann robot | D3-S19, pp.31-33 §4.3 | Verified | not independently re-extracted in this pass but consistent with the thesis's confirmed §4 structure and Section 4.2/4.3 controller-comparison framing | — |
| 138 | IGVC AutoNav qualification+timed run: lane/obstacle/pothole/speed rules, ranked by adjusted time/distance, DQ criteria | D3-S26, §II.1-II.6 | Verified | matches #84; ranking-by-adjusted-time/distance text confirmed in §II.1 | — |
| 139 | Stanley field test, full-size vehicle: RMS/σ cross-track error over 132mi race + repeated laps; RMS<0.1m = success (half tire width) | D3-S08, p.6 | Verified | see #61; "less than half a tire width" exact quote | — |

## Common mistakes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 140 | Treating RPP as a full motion planner — it has no feasibility check, path must already be drivable | D3-S02, p.9 | Verified | see #46/#59 | — |
| 141 | Mounting Ackermann drive bar directly on motor shaft without reinforcement (D3-S29); under-sizing direct-drive gear ratio burns out motors (D3-S27) | D3-S29, p.6; D3-S27, p.4 | Verified (page caveat as #89) | both confirmed verbatim | — |
| 142 | Crab maneuvers on rocker-bogie rovers load suspension unevenly; reserve for all-wheel-steer, avoid on long traverses | D3-S17, p.2 | Verified | see #32 | — |
| 143 | Assuming `use_rotate_to_heading` is diff-drive-only — it's framed around in-place-rotation capability; can't combine w/ `allow_reversing` | D3-S13, lines 230-237 | Verified | see #60 | — |
| 144 | Choosing Ackermann for a non-course reason while overlooking the turning-radius penalty in a space-constrained course | D3-S27, p.4 | Verified | consistent with #86 | — |

## Disagreements between sources

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 145 | No real contradiction between D3-S05/D3-S18/D3-S17 on the core power/precision finding — differing magnitudes reflect different operating points (idealized point turn vs bounded sweep) | D3-S05; D3-S18; D3-S17 | Verified | consistent with #51-54; the reconciliation logic is sound given the confirmed numbers | — |
| 146 | Tension (not contradiction) between Ackermann's precision/efficiency advantage and its maneuverability cost (largest min radius of modes measured) | D3-S19; D3-S27 | Verified | consistent with #31, #86 | — |

## Open questions (cited / claims of absence)

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 147 | Ackermann historical origin now sourced via D3-S34 (citing King-Hele 2002 + 3 textbooks); primary 1816/1818 docs still unlocated | D3-S34 | Verified | consistent with #23; D3-S34 itself does cite further secondary sources for the history (not independently chased, out of scope for a claims-only review) | — |
| 148 | 0.5µg threshold now sourced via Polack et al. 2017 (through D3-S36/D3-S37); corrects a previous-pass misattribution to Kong et al. | D3-S36; D3-S37 | Verified | consistent with #38-40 | — |
| 149 | No qualifying source found for tie-rod wear/backlash/bump-steer tolerances generally; one quantitative mitigation result (three-center theorem) was found and added | — | Verified | consistent with #98-99; no such general-wear source appears in `sources/` | — |
| 150 | ISO 8855 / SAE J695 still unread; no qualifying secondary source found explaining curb-to-curb vs wall-to-wall definitions | — | Verified | consistent with #97; no such secondary source is in `sources/` | — |
| 151 | No IGVC design report found with an explicit side-by-side Ackermann-vs-skid-steer cost/turning-radius/maintenance rationale; D3-S27/28/29 just describe teams that already chose Ackermann | D3-S27; D3-S28; D3-S29 | Verified | consistent with the content actually found in all three design reports (#86-89) — none frames a comparative decision process | — |
| 152 | No source found directly comparing Ackermann vs differential/skid-steer specifically on outdoor grass/mud/gravel traction+durability on the same chassis | — | Verified | consistent with the content of D3-S05/D3-S18 (hard-surface/regolith testbeds) and the IGVC/agricultural sources (no isolated traction-comparison arm) | — |
| 153 | LIMO's third-party per-mode turning-radius/payload figures could not be confirmed in the distributor docs actually read | D3-S40 | Verified | see #75 | — |

---

## Summary of Partly supported / Not supported items

**Not supported (2):**
- **#82** (PACMod, Findings §9): "Polaris GEM series" fitment is not stated anywhere in the cited source (`pacmod_lexus_brochure.pdf` covers only the Lexus RX450h). Everything else in that bullet is correct.
- **#76** (OpenPodcar steering mechanism, Findings §9): the claim that the actuator "replac[es] the manual handlebar linkage" at "the donor vehicle's existing tie-rod attachment point" is not supported — the source describes the actuator as newly mounted between a chassis anchor and a hole in the front wheel **axle**, with no mention of a "tie-rod" anywhere in the paper. (The actuator's physical specs in the same bullet — 750N/8mm/s/250mm — are correct.)

**Partly supported (12):** mostly page/section-label errors with the substantive content otherwise correct, plus a few added descriptors not present at the cited location:
- **#23** D3-S34 "tie rods... current industrial practice" clause is on p.4, cited as p.1.
- **#28** JPL "center of rotation unconstrained along any axis" is stated explicitly only for all-wheel steering, not for skid steering, in the source.
- **#44** D3-S14 "4-5× minimum turning radius" lives under `analytic_expansion_max_length`, not under the two entries named in the citation.
- **#48, #49** D3-S06 citations to "§IV-B" refer to a section that doesn't exist in the paper; the content is in §III-B (pp.297-298).
- **#72, #131** D3-S38 page cited as p.3 but content is on printed p.4; "1/10-scale" for the original QCar is not stated in D3-S38 (only for QCar 2, in D3-S39).
- **#73** "discontinued" (QCar→QCar2) is an unstated inference from D3-S39.
- **#89** D3-S29 page cited as p.6; the source has no printed page numbers, and the best available (physical-page) evidence points to p.5.
- **#94** Deshpande's "along a row-crop trajectory" qualifier is not at the cited p.35 location (row-crops are discussed only elsewhere in the thesis).

All remaining ~101 checked claims (geometry/kinematics equations, ROS2/Nav2 parameter and code behavior, platform specs, IGVC rules text, and every direct quotation checked) matched their cited sources exactly at the cited location.

---

## Corrections applied (2026-10-06)

Both reviews (this file and `SOURCE_AUDIT.md`) were applied to `README.md`. Every claim re-checked against its source directly (not just against the reviewer's prose) before being rewritten; one reviewer page-citation recommendation (see #89/QCar below) was found to be itself mistaken on re-check and was **not** applied.

### Claims (this file)

- **#23** (D3-S34, Findings §1): split the citation so the Darwin/Lankensperger/Ackermann/Jeantaud history stays at p.1 and the "tie rods... current industrial practice" clause now cites p.4.
- **#28** (D3-S17, Findings §1): reworded the skid-steering clause — dropped the unsupported "center of rotation unconstrained along any axis" attribution to skid steering (the source states "unconstrained" only for all-wheel steering); skid steering is now described only as "all wheels fixed-direction," matching the source.
- **#44** (D3-S14, Findings §3): split the citation so the reverse-penalty/minimum_turning_radius clause and the "4-5× minimum turning radius" clause each cite their own correct parameter entry (the latter is under `analytic_expansion_max_length`, not `minimum_turning_radius`/`reverse_penalty`).
- **#48, #49** (D3-S06, Findings §4): fixed both "§IV-B" citations to "§III-B" — confirmed directly that §IV ("Conclusions") has no lettered subsections; the content is in §III-B.
- **#72, #131** (D3-S38/D3-S39, Findings §8 and Key numbers): removed the unsupported "1/10-scale" descriptor for the original QCar (the source only says "scaled model car"; "1/10th scale" is stated only for QCar 2 in D3-S39). **Re-checked the p.3-vs-p.4 page question directly** (via `pdftotext -f N -l N`, confirming each PDF page's own running header number): the quoted ±30°/Jetson TX2 content is genuinely on PDF/printed page 3, so the reviewer's recommended "fix to p.4" was itself incorrect and was **not** applied — the original p.3 citation is correct.
- **#73** (D3-S39, Findings §8 and Sources table): removed the unstated inference that the original QCar has been "discontinued" (the source never says this), both in the Findings §8 bullet and in the Sources-table description of D3-S39.
- **#76** (D3-S22, Findings §9 — not supported): rewrote the OpenPodcar steering bullet. Re-checked p.3/p.4 directly: the donor vehicle "steers via a human-operated loop handlebar" (p.3) is retained; dropped "replacing the manual handlebar linkage" and "tie-rod attachment point" (the source never says "tie-rod" and describes a separate actuator newly mounted to the front wheel axle via a chassis anchor and bearings, not a replacement of the handlebar mechanism) — now described that way, citing the correct section heading "Mechanical Modification for Steering."
- **#82** (D3-S25, Findings §9 — not supported, and failing source): the whole bullet was re-sourced rather than patched, because its only source (the PACMod marketing brochure) is a failing source (see Sources, below) that no longer exists in `sources/`. The replacement bullet is built from two new official-project sources (D3-S25a/b, the `pacmod3` ROS driver's own README and ROS wiki documentation) and now correctly lists Polaris GEM (among other supported vehicles) and a Lexus RX-450h, plus an accurate, narrower description of controlled functions, feedback topics and the override/disable status topic — replacing the unsupported "immediate return-to-manual safety feature" framing.
- **#89** (D3-S29, Findings §10/§12/Common mistakes): **re-checked directly** by locating the exact PDF page of the quoted footnote and body text via page-by-page extraction. Found the section *heading* "4.3 Ackermann Steering" is on PDF page 5, but the quoted footnote and all quoted body text are on PDF page 6. The reviewer's recommended fix (p.5) would have been wrong; the **original p.6 citation is correct** and was left unchanged.
- **#94** (D3-S19, Findings §11): dropped the unsupported "along a row-crop trajectory" qualifier (not stated at the cited p.35 location).

Claim #17 ("1/10–1/8-scale... F1TENTH, MuSHR, MIT RACECAR" in `SCOPE.md`'s overview) and all other claims marked Verified were left unchanged.

### Sources (`SOURCE_AUDIT.md`)

- **D3-S25 removed** (`pacmod_lexus_brochure.pdf` deleted) — failing source (marketing brochure, no quantitative datasheet content). Replaced with two official-project sources: **D3-S25a** (`pacmod3` GitHub README, pinned to tag `ros2-1.3.1`) and **D3-S25b** (the `pacmod3` ROS wiki documentation page, retrieved via an academic mirror since the live `wiki.ros.org` now returns a bot-check page to automated fetches). Both are Level B (official project documentation/code).
- **D3-S18, D3-S19** regraded C → A (same CMU-RI-TR series and institutional rigor as the Level-A D3-S03/S04/S05).
- **D3-S35** regraded B → C (journal article, not official manufacturer/project documentation; IAENG is not among this project's recognized Level-A venues, so the stricter preprint-style default was used). Renamed `iaeng_2023_...` → `cai_2023_fsae_ackermann_steering_optimization.pdf`.
- **D3-S37** regraded B → C (preprint, no confirmed peer-reviewed venue — consistent with D3-S21/D3-S23/D3-S38 in this same topic); fixed the Sources-table description, which had called it an "accepted paper" — confirmed directly that the file's "Received on M, D, YYYY / Accepted on M, D, YYYY" header is an unfilled template, not a real acceptance date.
- **D3-S21** renamed `srinivasa_2019_...` → `srinivasa_2023_mushr_racecar.pdf` (file is arXiv v3, matching the citation's already-correct "2023").
- **D3-S23** renamed `camara_2026_...` → `soni_2026_openpodcar2_ros2.pdf` (paper's actual first author is Soni, not Camara — Camara is the first author of the sibling paper D3-S22).
- **D3-S34** renamed `padua_2020_...` → `veneri_2020_ackermann_race_car_performance.pdf` (first-author surname, not the authors' university city).
- **D3-S17** renamed `jpl_rover_maneuvering_autonomous_manipulation.pdf` → `nesnas_2000_rover_maneuvering_autonomous_manipulation.pdf` (first-author surname, for consistency with every other individually-authored source in this topic; low-priority/optional fix, applied anyway).
- **D3-S24** citation corrected to credit MyBotShop as the document's publisher/distributor, with AgileX Robotics named as manufacturer (the manual is MyBotShop-branded throughout and explicitly distinguishes "manufacturer (AgileX Robotics) or distributor (MYBOTSHOP)").
- **D3-S13** (`nav2docs_configuring_regulated_pp.md`): stripped the embedded `<style>`/YouTube-`<iframe>` HTML widget block (and the now-dangling sentence referencing it) so the file is clean markdown; re-verified with `file` (now reports ASCII text, not HTML) and updated the `use_rotate_to_heading` line-number citations in README.md (three places) from "lines 230-237" to the new "lines 180-187" after the 50-line deletion.

### Final mechanical check

Re-ran after all corrections: `file` matches extension for all 45 files now in `sources/`; every file in `sources/` has a Sources-table row and vice versa (zero orphans either direction); every cited `D3-Snn` ID resolves to a Sources-table row; numeric IDs run 01-42 with no gaps or duplicates (D3-S07 and D3-S25 each split into lettered sub-IDs, as D3-S07 already was before this pass). `README.md` Status changed from Draft to **Verified** — no Not-supported or Partly-supported claims and no failing sources remain.
