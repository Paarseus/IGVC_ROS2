# VERIFICATION.md — D2: Wheeled skid-steer / differential drive

**Topic:** D2_wheeled_skid_steer
**Date:** 2026-10-06
**Reviewer:** independent — claims (Step 4 of STANDARDS.md §5; source audit is a separate reviewer's job and is not duplicated here)
**Method:** every cited location was opened directly — PDFs via `pdftotext -layout` (cross-checked page-by-page with `\x0c` splits, and in one case by rendering the PDF page as an image to read a bar chart) and `.md`/`.yaml`/`.cpp`/`.hpp` sources via `grep`/`Read`. Two "not downloaded" abstract-only sources (D2-S19, D2-S39) were checked by fetching their cited institutional-repository pages.

## Counts

| Status | Count |
|---|---|
| Checked | 137 |
| Verified | 116 |
| Partly supported | 15 |
| Not supported | 6 |

(5 of the 6 "Not supported" items are genuine contradictions/mischaracterizations — a source cited for the opposite of what it says, or one paper's methodology wrongly attributed to another — not just imprecise page numbers; see detail rows #45, #80, #96, #126, #129. The 6th, #46/#58 (the same uncited sentence appears twice), is a factual claim with **no citation at all**, counted under "Not supported" since nothing could be checked against it.)

## Legend
- **Verified** — source says this, at (or acceptably near) the cited location.
- **Partly supported** — source supports the substance, but a number/word/page is off, a quote is paraphrased-as-verbatim, or the claim mildly overstates.
- **Not supported** — the source says something different or the opposite, or no source was cited.

---

## Summary

| # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|
| 1 | Ideal diff-drive excludes skid-steer; both textbooks exclude it; quote "tanks and skid-steered vehicles are excluded" | [D2-S02 §3.3.1 p.61][D2-S03 ch.13 intro p.513] | Partly supported | Quote is verbatim **only** in Lynch (p.513, confirmed by page-footer "513"): "...without skidding (i.e., tanks and skid-steered vehicles are excluded)." Siegwart p.61 does exclude the degenerate 4-wheel slip-skid case but never uses this phrase. | Attribute the quoted phrase to D2-S03 alone; Siegwart supported by paraphrase only. |
| 2 | Mandow ICR model: xICRr≈0.31, xICRl≈−0.26 m, half-sep 0.20 m (L=0.40 m), ratio ≈1.4× (1.31–1.44× across six conditions); same shape as `wheel_separation_multiplier`, applied identically to command+odometry path | [D2-S05 §IV.C Tables I–II][D2-S20 lines 143,298][D2-S09 "wheel_separation_multiplier"] | Verified | Table I (asphalt, 20psi): xICRr=0.3071, xICRl=−0.2553, L=0.4m. Computed ratios across all 6 rows = 1.314–1.439. `diff_drive_controller.cpp` line 143 (update(), command path) and line 298 (on_configure(), odometry path) both: `wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;` (byte-identical, confirmed at those exact line numbers). | — |
| 3 | Effective-width ratios: 1.31–1.44 (P3-AT 23.6kg), 1.33–1.54 (330kg UGV), 2.6–3.7 (590kg Warthog); dominant pattern = surface/vehicle-dependent | [D2-S05 Tables I–II][D2-S14 Tables 1–2][D2-S12 Table I] | Verified | Computed from Mandow Table I/II (above); Zhou Table1 B=1.45m, Table2 minB=1.927,maxB=2.2259 → 1.33–1.54×; Baril Table I b̂=4.46m(concrete)/3.08m(snow), b=1.2m → 3.72×/2.57×. | — |
| 4 | `diff_drive_controller` unified controller; Husky/Jackal/Warthog multiplier values (1.875/1.5/1.125); one Husky repo ships uncorrected 1.0 | [D2-S09][D2-S24][D2-S25][D2-S26][D2-S27] | Verified | `clearpath_2024_a200_husky_control.yaml`:20 `wheel_separation_multiplier: 1.875`; j100:20 `1.5`; w200:20 `1.125`; `husky_2023_humble_devel_control.yaml`:20 `1.0`. | — |
| 5 | Grass data: Crusher 1.8m→cm; Jackal GP 18.9%→5.6% ang., 6.2%→5.7% lin., baseline lin. lower on grass (6.2%) than asphalt (14.2%); 4-wheel robot found terrain insignificant, radius dominant | [D2-S17 §6.1 p.28][D2-S13 Table I][D2-S16 "Results and Discussion"/"Conclusions"] | Verified | Seegmiller p.28: "mean error is reduced from 1.8 meters to a few centimeters." Trivedi Table I: EDD5 ang=18.9(grass)/lin=6.2(grass) vs 14.2(asphalt); GP=5.6/5.7. Wanichratanagul Conclusions: "the terrain type is insignificant to the skid behavior." | Minor: says "ceramic tile"; source says "ceramic plate floor" / "ceramic floor" (Fig.7 caption). |
| 6 | Off-the-shelf range: Husky A300 80kg/2.0m/s/100kg; Jackal 17kg/2.0m/s; Warthog 280–590kg/18km/h/amphibious/track option; AndyMark AM14U6 6-wheel/ToughBox Mini S/$940; IGVC DIY builds | [D2-S28][D2-S29][D2-S30][D2-S31][D2-S32][D2-S33] | Verified | All figures independently confirmed against each datasheet/manual (see Findings §7 rows below). | — |

---

## Foundational references

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 7 | Siegwart&Nourbakhsh: textbook derivation of no-sideslip constraints, ICR, degree-of-mobility | D2-S02 | Verified | Confirmed ch.3 contains exactly this (§3.2.3, §3.3.1). | — |
| 8 | Lynch&Park: nonholonomic constraints/diff-drive; explicitly scopes out skid-steering | D2-S03 | Verified | Ch.13 intro, p.513, confirmed quote. | — |
| 9 | Campion et al.: origin of 5-type classification "every later kinematics paper... cites" | D2-S01 | Verified (framing claim, not independently falsifiable) | Confirmed the 5-type (δm,δs) classification and type-(2,0) diff-drive content on pp.736/741–742 (Russian translation, read in full). "every... cites" is rhetorical emphasis, consistent with the paper being cited by both Mandow and Kozłowski in this topic's own source set. | — |
| 10 | Borenstein&Feng: origin of UMBmark, Type A/B decomposition | D2-S04 | Verified | Abstract + §4.1 confirmed. | — |
| 11 | Mandow et al.: seminal wheeled ICR model, origin of `wheel_separation_multiplier` convention | D2-S05 | Verified | Confirmed content; the "origin of the convention" is the topic's own synthesis (reasonable — Mandow's model is structurally identical to the multiplier, as shown in Findings §2). | — |
| 12 | Kozłowski&Pazderski: widely-cited 4-wheel skid-steer model w/ ICR-boundedness condition | D2-S06 | Verified | §1 p.477–478 confirmed. | — |
| 13 | Wang et al.: open-access wheeled ICR follow-on (laser-scanner), direct comparator to Mandow | D2-S07 | Verified | Confirmed, same P3-AT platform family, compares to "default P3-AT model." | — |
| 14 | Rabiee&Biswas: SOTA wheeled slip model, benchmarked on public 6km Jackal dataset incl. grass | D2-S08 | Verified | Abstract: "more than 6km"; dataset spans "tile, asphalt, and grass." | — |
| 15 | `diff_drive_controller` docs: primary ROS2 spec for driving/correcting a wheeled base | D2-S09 | Verified | Confirmed exact content match throughout Findings §6. | — |
| 16 | Bruzzone&Quaglia: open-access survey, wheeled/tracked/legged/hybrid comparison framework | D2-S11 | Verified | Confirmed Table 1/Table 2/§3.1–3.2 content. | — |
| 17 | Wong 2008 *Theory of Ground Vehicles* — not downloaded, listed as foundational | — | Verified (accurately labeled as not-downloaded/not-cited-for-content) | No file present in sources/; README does not cite it for content. | — |
| 18 | Wong&Huang 2006 — not downloaded, identified via Mandow's own ref list | — | Verified | Mandow reference [6]: "J. Y. Wong and W. Huang, 'Wheels vs. tracks...', Journal of Terramechanics, vol. 43, pp. 27–42, 2006" confirmed present in Mandow's reference list. | — |

---

## Findings

### §1 Ideal wheeled differential-drive kinematics

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 19 | Siegwart: rolling + sliding constraint per wheel, single contact point, pure rolling | [D2-S02 §3.2.3 pp.45–46] | Verified | p.46: "we assume... one single point of contact... no sliding at this single point... pure rolling." | — |
| 20 | ICR on "zero motion line"; diff-drive δm=2 | [D2-S02 §3.3.1 pp.58–59,61] | Verified | p.59: "zero motion line"/ICR text; p.61 (just before p.62 marker): "rank C1(βs)=1 and δm=2." | — |
| 21 | Siegwart: 4-wheel slip-skid is *degenerate*/outside no-slip framework; quote on dead-reckoning/power cost | [D2-S02 §3.3.1 p.61] | Verified | p.61 (just after marker "61"): "...four wheeled slip-skid steering system are useful... dead-reckoning based on odometry becomes less accurate and power efficiency is reduced dramatically." | — |
| 22 | Campion: 5 non-degenerate structures, diff-drive=type(2,0), fixed wheels on 1 axle, skid-steer (>1 axle) outside all 5 types | [D2-S01 §II.B–C pp.736,741–742] | Verified | p.736: no-slip contact-point constraint; p.741: "only five non-degenerate structures"; p.742: type (2,0) = "one or several... fixed wheels located on one axis." | — |
| 23 | Lynch: excludes skid-steer by assumption; diff-drive eqs φ̇=(r/2d)(uR−uL), v=(r/2)(uL+uR), inverse uL=(v−ωd)/r, uR=(v+ωd)/r | [D2-S03 ch.13 intro p.513; §13.3.1.2 eq.13.14–15 pp.523–524] | Verified | Page-footer-confirmed p.523 (eq.13.14) / p.524 (eq.13.15 + inverse transform), exact algebra match. | — |
| 24 | Lynch: odometry error from "unexpected slipping and skidding... and numerical integration error"; recommends Kalman/particle filter+GPS | [D2-S03 §13.4] | Verified | §13.4 p.546 (page-footer-confirmed), exact quote match. | — |
| 25 | Caster wheel adds no kinematic constraint; Fig.3.13a (Pygmalion) is still ideal δm=2 case, not skid-steer | [D2-S02 §3.2.3.3 p.50; Fig.3.13a p.59] | Verified | p.50: "such wheels do not impose any real constraints on the kinematics of a robot chassis." Fig.3.13a confirmed on p.59, captioned exactly as quoted. | — |

### §2 ICR model / effective-width correction

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 26 | Mandow: diff-drive pure-rolling single contact point quote; skid-steer "several contact patches... mechanically linked wheels" | [D2-S05 §II p.2] | Verified | Local PDF p.2 (page 1 ends before "following:"); exact quote match. | — |
| 27 | ICR geometric relations eq.3–8; matrix A algebraically = ideal diff-drive w/ tread-ICR contact points | [D2-S05 §II eq.3–8 pp.2–3] | Verified | Eqs (2)–(5) on p.2, eqs (6)–(8) on p.3 (page split confirmed). | — |
| 28 | Fitted on P3-AT, 20psi asphalt: xICRr≈0.307, xICRl≈−0.255 (eff.sep≈0.56m,≈1.4×), α≈0.90–0.93; solid-rubber α "slightly lower" than pneumatic | [D2-S05 §IV.B–C Table I] | Partly supported | xICRr/xICRl/ratio exact. α range 0.9049–0.9271 for 20psi matches "0.90–0.93." But "solid rubber α slightly lower than pneumatic" is not uniform: asphalt solid αr=0.9128 > asphalt 20psi αr=0.9049 (one of four r/l×terrain comparisons goes the other way). | Qualify as "on average, slightly lower" or note the one exception (asphalt right tread). |
| 29 | Steering efficiency χ=L/(xICRr−xICRl); χ≈0.69–0.76; quotes on asphalt-vs-concrete thrust loss and compact-wheel efficiency/traction tradeoff | [D2-S05 §IV.C eq.9, Tables I–II] | Verified | Eq.9 confirmed; computed χ range 0.6949–0.7612 matches exactly; both quotes verbatim. | — |
| 30 | MSE: asymmetric (0.00010,0.00011) vs "factory symmetric model" (0.00162,0.00455) | [D2-S05 §IV.D Table III] | **Partly supported** | Table III has 3 columns: Asymmetric / **P3-AT** (factory default, α=1, xICR=0.3m) / **Symmetric** (experimentally fitted, α=0.91, xICR=0.275m). (0.00162,0.00455) is the **Symmetric** column, not the **P3-AT** (factory) column, which is actually (0.00183,0.00468). | Relabel: the cited numbers are the experimentally-fitted symmetric model, not the as-shipped factory default. If "factory" comparison is intended, use (0.00183, 0.00468) instead. |
| 31 | Kozłowski/Caracciolo: excessive ICR projection → instability quote; operational fix = bound projection to wheelbase | [D2-S06 §1 p.477–478] | Verified | Exact quote match ("vehicle can lose motion stability as a result of skidding... not important for traditional vehicles if only the no-slip assumption is satisfied") and Caracciolo fix description. | — |
| 32 | Wang: λ/χ relationship; improves dead-reckoning to pos.error <0.03m, angle <0.1rad vs default, a<3m/s², v<0.5m/s | [D2-S07 §4 "Conclusions/Outlook" p.9701] | **Partly supported** | Quote and numbers exact, but located (confirmed via `\x0c` page split) on **local PDF page 9700**, not 9701. | Change page to p.9700. |
| 33 | Rabiee: friction model outperforms SOTA (translational+rotational); advantage stands out w/ accelerated motion | [D2-S08 Abstract; §V p.4] | Partly supported | Abstract quote exact. "Accelerated motion" quote exact but located on local PDF page 5 (§V itself starts on p.4). | Cite p.5 for the "accelerated motion" sentence, or note "§V (starts p.4)". |
| 34 | On Jackal, ↑commanded angular accel → ↓observed angular velocity for fixed wheel-vel pair; correction not accel-invariant | [D2-S08 Fig.5 and surrounding text] | Verified | Fig.5 caption: "Increase in angular acceleration leads to decrease in observed angular velocity for the same pair of wheel velocities." | — |
| 35 | Baril: virtual width b̂=4.46m(concrete)/3.08m(snow), b=1.2m (≈3.7×/2.6×); quotes on >30° nonlinearity + "hard to describe with linear models" | [D2-S12 Table I; §V] | Verified | Table I exact; both quotes verbatim, confirmed in §V "Results." | — |
| 36 | Baril: ideal diff-drive model "performs better on snow than on concrete" | [D2-S12 §V results discussion] | Partly supported | Source literally reads "performs better on snow **that** on concrete" (apparent typo for "than"); claim silently renders it as "than" without `[sic]`/brackets. Substance correct. | Quote verbatim including the typo, or mark the correction with brackets. |
| 37 | Zhou: effective width B≈1.93–2.23m (naive), physical 1.45m (≈1.33–1.54×); quote on measured diff overstating actual diff | [D2-S14 Table 1, Table 2, Results] | Verified | Table1: width=1.45m. Table2: minB/maxB across both rows = 1.9270–2.2259 → ratios 1.329–1.538. Quote closely matches ("measured differences in velocity and trajectory between the outer and inner wheels were larger than those of actual values"). | — |

### §3 Contact-patch physics

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 38 | Contact-patch shape distinction; "several contact patches... mechanically linked wheels" | [D2-S05 §II p.2, Fig.1] | Verified | Same text as #26, Fig.1 present with (a)-(d) sub-captions. | — |
| 39 | Jia/Smith/Peng: per-wheel cylindrical contact-area model, stress integration for reaction forces/torques | [D2-S18 §3 pp.2–3] | Partly supported | "cylindrical wheel-soil contact area" quote confirmed verbatim — but on local PDF **page 4**; §3 header itself is on **page 3**; page 2 is Table I (background/related work, not §3). | Cite pp.3–4, not pp.2–3. |
| 40 | Li/Yin/Zhang/Yuan (BIT): "with reference to steering theory of tracked vehicles"; abstract: wheeled turning-resistance coefficient smaller than tracked's | [D2-S19 abstract, not downloaded] | Verified (confirmed via live fetch of the cited institutional-repository page) | Fetched page confirms: "the turning resistance coefficient of skid steer wheeled vehicles is smaller than that of tracked vehicles" (exact) and "...established with reference to **the** steering theory of tracked vehicles" (claim drops "the"). | Minor: restore "the" or mark its omission. |
| 41 | Wong&Huang secondhand, not used for content | [D2-S05 references, ref.6] | Verified | Mandow ref [6] confirmed verbatim (see #18). | — |
| 42 | Li/Zhang/Hu/Yuan (BIT, DEM+micromechanical): "a fitting formula for calculating the skid-steering resistance coefficient"; "bulldozing resistance... much higher than friction force" | [D2-S39 abstract, not downloaded] | Partly supported | Fetched abstract: "The fitting formula of skid-steering resistance coefficient **is given**" (not "...for calculating..." — the claim's first phrase is a paraphrase dressed as a verbatim quote). Second quote ("bulldozing resistance between tire and ground is much higher than friction force") is exact. DEM/micromechanical framing confirmed accurate. | Un-quote the first phrase, or match it to the actual wording. |

### §4 Magnitude/predictability of correction

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 43 | Wheeled ratio spread 1.31–3.7× not smaller/tighter than tracked (synthesis, no new source) | (synthesizes D2-S05/S14/S12 already cited above) | Verified | Arithmetic re-check of the three already-verified ranges confirms the "almost three-fold spread" claim. | — |
| 44 | Clearpath configs 1.875/1.5/1.125 → effective ≈1.041/0.563/1.688m | [D2-S24][D2-S25][D2-S26] | Verified | A200: 0.555×1.875=1.0406; J100: 0.37559×1.5=0.5634; W200: 1.5×1.125=1.6875. | — |
| 45 | Husky `humble-devel` ships 1.0 "with the same physical wheel-separation value" as the A200 `clearpath_common` config | [D2-S27][D2-S24] | **Not supported** (for the "same physical value" clause) | `clearpath_2024_a200_husky_control.yaml`:16 `wheel_separation: 0.555`; `husky_2023_humble_devel_control.yaml`:16 `wheel_separation: 0.512`. The two active values differ (0.555 vs 0.512 m) — not the same. | Drop "with the same physical wheel-separation value," or note the two configs disagree on *both* the physical value and the multiplier. |
| 46 | "No separate 'skid-steer controller' in the current `ros2_controllers` package" | **(none — no citation at all)** | **Not supported** — uncited | The bullet has zero `[D2-S…]` bracket. Likely true (only `diff_drive_controller`, `tricycle_controller`, etc. exist) but unverifiable against any cited source as written. | Add a citation (e.g. the controllers index page) or drop the "there is no..." sentence. |
| 47 | Wanichratanagul: "terrain type is insignificant to the skid behavior"; radius dominant, "larger radius generates a smaller slip" | [D2-S16 "Conclusions"] | Verified | Exact quotes confirmed in CONCLUSIONS section. | — |
| 48 | Baril: validating models "above 120 kg" previously unstudied; 5 models transfer but magnitudes platform/surface-specific | [D2-S12 §I p.1] | Partly supported | "above 120 kg" quote exact, on p.1 (confirmed). The second half of the sentence (models "transfer... but magnitudes... not portable constants") is a synthesis of Table I / §V / §VI results, not of §I p.1 specifically. | Either cite Table I/§V/§VI for the second clause, or split into two cited sentences. |

### §5 Odometry / state estimation

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 49 | UMBmark: Type A (wheelbase, same-direction) / Type B (diameter, opposite-direction); "at least one order of magnitude" improvement | [D2-S04 Abstract; §4.1 pp.13–14] | Verified | Abstract quote exact; Type A/B definitions confirmed on p.13 (page-marker-confirmed). | — |
| 50 | UMBmark targets only non-skid error; ICR method is separate/additional | [D2-S05 §III][D2-S04 Abstract] | Verified | Reasonable, accurate synthesis — UMBmark abstract scope is explicitly "differential-drive mobile robots," non-skid. | — |
| 51 | Okawara: wheel model "must be maintained online"; linear skid-steer model can't capture large-slip nonlinearity; online learning ≈0.03s/step vs 0.1s LiDAR period | [D2-S15 §1 pp.1–2; §4.2.2 p.14] | Verified | p.1 quote exact ("wheel odometry model must be maintained online to adapt..."); p.14 (confirmed via page split) quote exact ("...the entire process (0.03 s) are sufficiently faster than... (0.1 s)."). | — |
| 52 | Okawara: 8 terrain types incl. grass; grass params isolated from flat-hard-terrain params; indoor-floor→grass transfer poor vs hard-to-hard | [D2-S15 lines ~1266–1378 of extracted text, Case 2 results] | Verified | Table 1 caption: "eight terrains." Case 2 (train indoor-floor/test grass) error 0.158–0.165m vs Case1 (indoor/wood-tiles) 0.034m — about 5–6× worse, matching "significantly worse." (Line numbers are extraction-tool-dependent and only approximately reproducible, but all content matches.) | — |
| 53 | Quote: "indoor flat floor was more similar to... wood tiles... than to grass (i.e. rough [and soft])" | [D2-S15 §4.2.2 around line 1377] | Verified | Exact text: "...more similar to another flat and hard terrain (wood tiles) than to grass (i.e., rough and soft terrain)." (Claim's "[and soft]" bracket implies an inserted clarification, but "and soft" is already verbatim in the source — a harmless redundant bracket, not a misquote.) | — |
| 54 | Seegmiller IPEM on Crusher over "rough, grassy terrain" incl. dirt road + tall dry grass; 1.8m→cm, σ↓72%/83%/90% vs no-slip | [D2-S17 §6 pp.27–28] | Verified | "rough, grassy terrain" on p.27; "1.8 meters to a few centimeters... 72%... 83%... 90%" on p.28 (both page-split-confirmed). | — |
| 55 | Crusher "effective turn rate is only a third"; "4 of 6 wheels... slipping sideways"; hillside load-transfer quotes | [D2-S17 §6.1 p.31] | Verified | All three quotes exact, confirmed on p.31. | — |

### §6 ROS 2 / Nav2 support

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 56 | `diff_drive_controller`: "mobile robots with differential drive"; links "Wheeled Mobile Robot Kinematics"; `wheels_per_side` Husky example quote | [D2-S09; D2-S21] | Verified | All exact matches in userdoc.md and parameter yaml. | — |
| 57 | `wheel_separation_multiplier` default 1.0, "correction factor..." quote; radius multipliers; applied identically cmd+odom | [D2-S09; D2-S21; D2-S20 lines 143,298] | Verified | Same as #2; quote exact. | — |
| 58 | Single general-purpose controller for both ideal and skid-steer cases; "no separate skid-steer controller" | (none) | **Not supported — uncited** (duplicate of #46) | — | Same correction as #46. |
| 59 | Gazebo ROS2: skid-steer plugin not reimplemented, folded into diff_drive; `num_wheel_pairs` removes 4-wheel limit | [D2-S22 "Overview"/"Summary"] | Verified | All quotes exact; ROS1 plugin file `libgazebo_ros_skid_steer_drive.so` vs ROS2 `libgazebo_ros_diff_drive.so` confirmed in the same wiki page's SDF examples. | — |
| 60 | Nav2 footprint: polygon/robot_radius example on `sam_bot`, itself diff-drive; drivetrain-agnostic | [D2-S23 "Setup"; "Build, Run and Verification"] | Verified | Footprint string matches exactly; "Gazebo's differential drive plugin publishes the odom→base_link transform" confirms sam_bot is diff-drive. | — |
| 61 | MPPI `motion_model` param, default "DiffDrive", quote on DiffDrive/Omni/Ackermann distinctions | [D2-S36 "MPPI Parameters"/"motion_model"] | Verified | Exact text match. | — |
| 62 | `DiffDriveMotionModel::isHolonomic()` returns false, lines 138–153 | [D2-S37 lines 138–153] | Verified | Class starts line 138, `return false;` at lines 150–153, exact match. | — |

### §7 Off-the-shelf platforms

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 63 | Husky A300: 990×698×372mm, 80kg, 130mm clearance, 100kg(50kg AT) payload, 2.0m/s, 30°, Skid Steer/4 in-wheel brushless, LiFePO4 40/80/120Ah@25.6V, IP54 | [D2-S28 Specifications] | Verified | Every figure matches the manual's Specifications table exactly, including the verbatim "Drive Type: Skid Steer" / "Motor Configuration: Four In-Wheel Brushless Motors." | — |
| 64 | Jackal: 508×430×250mm, 17kg, 65mm, 20kg(10kg AT), 2.0m/s, 500W, 270Wh Li-ion, 3 control modes, "packaged with ROS Kinetic" | [D2-S29] | Verified | Every figure and the exact quote match the datasheet. | — |
| 65 | Warthog: 1.52×1.38×0.83m, 280/590kg, 254mm, 272kg payload, 35–45°, 18km/h, tire quote, IP67 amphibious (4km/h, not w/ track), AGM 105Ah@48V | [D2-S30] | Verified | Every figure and both quotes match the datasheet exactly (incl. "*Warthog is not amphibious with Quad Track System configuration"). | — |
| 66 | AM14U6: current gen of AM14U5 family, unassembled 6-wheel drop-center, HiGrip wheel quote, ToughBox Mini S, HTD belts, 5.95–12.76:1, Long/Square/Wide 24.3×27–32.3×31in, ≥2 CIM ("to be competitive" w/4), 31lb, $940, no motors/encoders/control incl. | [D2-S31] | Verified | Every figure/quote matches the AndyMark product page exactly. | — |
| 67 | Iorek (2010): 6-wheel drop-center, middle wheel −¼in, quote on turning scrub; 4×270W minibike motors (2/side), 15T/60T (4:1) + 14T/60T chain (17.14:1 total), 10.5in pneumatic wheels, 2.1m/s, 4 Grayhill encoders | [D2-S32 "Mechanical Systems and Design"] | Verified | Every figure and the long quote match the design report exactly. | — |
| 68 | Misti (2013): 4-wheel skid-steer, 2× 4.5hp Ampflow A28-400, 2-stage 30:1, fore/aft wheels mechanically linked (quotes), 145/70-6 pneumatic, 4-bar linkage + Fox DHX RC4 (~5in travel), 1 encoder/gearbox | [D2-S33 §2.2 pp.5–6] | Verified | Every figure and both quotes match exactly; content spans p.5→p.6 as cited (footer "6 of 18" confirms the split). | — |
| 69 | Clearpath quote-on-request pricing; 2014 Jackal launch-price quotes; "about half the price" | [D2-S38] | Verified | Both quotes exact, confirmed with byline "Evan Ackerman" and date "15 Sep 2014." | — |

### §8 Terrain traction / precision

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 70 | Crusher Camp Roberts: "both a dirt road and tall dry grass"; roll −28°/29°, pitch −22°/17°; lat. accel 0.5g(slopes)/0.3g(maneuvers) | [D2-S17 §6.1 Fig.8] | Verified | All four figures and the quote match Fig.8/caption exactly. | — |
| 71 | Trivedi: baseline % errors ang 17.6/18.9/21.1, lin 14.2/6.2/11.6 (asphalt/grass/tile); GP: 5.7/5.6/10.9, 5.8/5.7/5.0; baseline lin. lower on grass than asphalt | [D2-S13 Table I] | Verified | Every single number matches Table I exactly, including column order. | — |
| 72 | Rabiee dataset "more than 2.3km" on tile/asphalt/grass, human-joystick, benchmark framing | [D2-S08 §IV "Long Distance Dataset"] | Verified | Quote exact: "more than 2.3km traversed by the robot in total... driven by a human operator using a joystick." (Subsection label itself is truncated by column-layout extraction, but table + prose confirm it unambiguously.) | — |
| 73 | Wanichratanagul: ZED2i V-SLAM GT, 180° U-turns, 1m/2m radii, gravel y0=0.93m, "suspected to be the most friction[al] terrain" quote | [D2-S16 "Results and Discussion"] | Verified | "we obtain y0 = 0.93 m" and "is suspected to be the most friction terrain" both confirmed exactly (claim's "[al]" is a correctly-labeled editorial completion of the source's non-native-English phrasing). | — |
| 74 | Okawara: indoor-floor→grass transfer worst because floor closer to wood-tiles than to grass | [D2-S15 §4.2.2 around line 1377] | Verified | Same text as #53. | — |
| 75 | No full-text source gives a direct wheeled-vs-tracked grass/dirt/gravel comparison; Bruzzone qualitative only | (meta-statement, no new citation) | Verified | Accurate description of what this search did/didn't find, consistent with all sources reviewed. | — |

### §9 Testing/validation methods

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 76 | UMBmark: 4m bidirectional square, decomposed via closed-form geometric model | [D2-S04 §4.1 Figs.7–8] | Verified | "4×4 m square path" confirmed multiple times; Figs 7/8 described exactly. | — |
| 77 | Mandow: joystick paths, DGPS <1cm @5Hz RTK, GA minimizing sum-of-squared pose-increment error | [D2-S05 §III; §IV.C] | Verified | "precision under 1 cm... RTCM/RTK... 5 Hz" confirmed; GA/fitness eq.(13) confirmed. | — |
| 78 | Wang: laser-scanner localization fits χ-λ relation, validated on held-out path | [D2-S07 Abstract, Conclusions] | Verified | Abstract matches claim closely. | — |
| 79 | Wanichratanagul: ZED2i GT + SimScape cross-check (friction=0.9, no significant diff across range); sim replicates qualitative but not precise slip, attributed to no wheel-deformation modeling | [D2-S16 "Dynamic simulation"; "Results and Discussion"] | Verified | "we choose the friction coefficient 0.9... do not show any significant difference"; "the model does not consider the wheel deformation" — both exact. | — |
| 80 | Seegmiller + Baril: both use high-end IMU + differential GPS GT, multi-km field datasets, online/offline resp. | [D2-S17 §6 Intro][D2-S12 §IV "Experimental Setup"] | **Not supported** (for the Baril/D2-S12 half) | Seegmiller p.26 (confirmed): "a high-end IMU and differential GPS unit were used for ground truth position measurement" — true for Seegmiller. Baril's paper contains **zero** occurrences of "GPS" anywhere; its ground truth is a Robosense RS-32 **LiDAR + ICP** registration pipeline (§IV, explicit). | Rewrite: Seegmiller uses IMU+DGPS; Baril uses LiDAR+ICP. Do not attribute GPS-based ground truth to Baril. |
| 81 | Rabiee released ≥6km dataset as shared benchmark, not per-paper incomparable data | [D2-S08 Abstract; §IV] | Verified | Matches abstract's stated purpose. | — |

### §10 Lessons from other domains

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 82 | Bruzzone: wheeled "high speed/low power... obstacle ability limited" quote; tracked "well suited... overcome obstacles... move more slowly... vibrations... polygon with moving vertices... limits max speed" quotes | [D2-S11 §3.1 p.51; §3.2] | Verified | All quotes exact, p.51/52 boundary confirmed. | — |
| 83 | Table 2: wheeled "high" on speed/efficiency, "low" on obstacle/slope/soft-terrain; tracked "medium/high" speed, "medium/high"–"high" obstacle/slope, "high" soft-terrain, "medium" efficiency; both "low" mech./control complexity, lower than every hybrid/legged category | [D2-S11 Table 2] | Partly supported | Table 2 re-read directly: wheeled "slope climbing capability" = **"low/medium"**, not pure "low" as stated. For mechanical complexity, WT-hybrid = **"low/medium"**, which shares the "low" floor with wheeled/tracked rather than being strictly higher — so "lower than every... hybrid category" slightly overstates for that one cell (control-complexity comparison is fine: WT-hybrid="medium" there). | Qualify wheeled slope-climbing as "low/medium"; note the WT-hybrid mechanical-complexity exception. |
| 84 | Bruzzone: most unstructured-env wheeled robots use 4/6/8 wheels not 3 (hospital/Roomba 3-wheel exception); "not suitable... poor stability" quote; 4+ wheel = "hyperstatic" | [D2-S11 §3.1 pp.51–52] | Verified | All quotes exact. | — |
| 85 | Crusher (CMU/DARPA): turn-rate/traction findings framed as a general lesson for "**any**" six-wheel skid-steer vehicle | [D2-S17 §6.1 p.31] | Partly supported | The turn-rate/traction content is verbatim (see #55), but the word **"any"** is not itself a quoted word from the source at this location — it's the README's own generalization dressed in quote marks. | Remove the quote marks around "any," or state plainly that this is an inference. |
| 86 | LandTamer/RecBot included "to show applicability to other platforms"; IPEM works on all 3 w/o architecture-specific changes | [D2-S17 §6 Intro] | Verified | Quote exact (p.26, confirmed); "without architecture-specific changes" is a reasonable inference from the paper's uniform treatment of all three platforms in Table/§6.3. | — |
| 87 | Shamah/Nomad: 223km Atacama field trial; skid vs explicit steering compared on same vehicle/radii; "power for skid steering is approximately double that for an explicit point turn"; convergence only at infinite radius; "slow but capable locomotors..." quote | [D2-S34 Abstract; ch.3 "Nomad" p.25; ch.4 §4.1 p.27] | Verified | "223km... Atacama Desert" on p.25 (footer-confirmed); "double that **for**..." exact on p.27 (§4.1, footer-confirmed; abstract's own wording is "double that **of**," a harmless variant cited separately); "slow but capable locomotors characteristic of planetary robotic vehicles" exact in Abstract. | — |
| 88 | Nomad torque split "in the same diagonal fashion"; 4m-radius highest wheel 49–52% of total vs 5–8% lowest; "rear outer wheel has a consistently higher torque" quote | [D2-S34 ch.4 §4.3.3 "Wheel Torque" p.35] | Verified | Text quotes exact and on p.35 (footer-confirmed). **Figure 21 percentages independently re-read from the rendered PDF page**: Fwd-CW 38/7/49/6%, Fwd-CCW 5/37/5/52%, Rev-CW 8/49/8/34% → highest range 49–52%, lowest range 5–8% — exact match to the claimed numbers. | — |
| 89 | Agri.Q (Botta): peer-reviewed, 8-wheel articulated, "hilly or mountainous crops"; quote on understeer/lateral slip reproducing prior results | [D2-S35 Abstract p.103] | Verified | Both quotes exact in the Abstract. | — |
| 90 | Only front module skid-steered; "rear motors are wired..." / "rear module is not skid-steered" quotes; TACS activates rear only for extra traction (incl. "negotiating extremely tight curves"); 60%/40% front/rear static split of 112kg total weight | [D2-S35 §3 p.109; §5.3 "Contact Forces Analysis" p.121] | Verified | All quotes exact; "the four front wheels support about 60% of the total weight (112 kg) while the rear wheels support the remaining 40%" confirms 112kg = total robot weight (not the front's own weight). Postprint's internal pagination differs slightly from the final published pp.103–126 (expected for an author manuscript vs. publisher typeset), but the proportional location is consistent with pp.109/121. | — |

### Failure modes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 91 | Excessive ICR displacement → instability quote | [D2-S06 §1 pp.477–478] | Verified | Same as #31. | — |
| 92 | Hillside load transfer quotes | [D2-S17 §6.1 p.31] | Verified | Same as #55. | — |
| 93 | Direct wheel-speed differencing overstates width; Zhou quote | [D2-S14 Results p.5 of extracted text] | Verified | Same numbers/quote as #37. "p.5 of extracted text" is explicitly self-caveated as tool-dependent, not a claim about the journal's own pagination (journal page-footer there actually reads "7"). | — |
| 94 | Baril 2.6×/3.7× + >30° nonlinearity contradicts treating multiplier as a constant | [D2-S12 Table I, §V] | Verified | Same numbers as #35. | — |

---

## Recommended practice

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 95 | Fit ICR/effective-width model whenever >2 fixed wheels or linked multi-wheel side drive | [D2-S05 §II][D2-S02 §3.3.1 p.61] | Verified | Consistent with both sources' content. | — |
| 96 | Calibrate against external GT rather than vendor default, because every source testing >1 surface found the ratio changed | [D2-S05 Tables I–II][D2-S12 Table I][D2-S07 Abstract][D2-S16 "Conclusions"] | **Not supported** (for the D2-S16 citation) | D2-S05 and D2-S12 do show surface-dependent change (verified). D2-S16's own Conclusion is the **opposite**: "the terrain type is insignificant to the skid behavior" (verified at #47) — it is not a source that "found it changed with surface." D2-S07's Conclusions don't test >1 surface either (single platform/terrain, a<3m/s², v<0.5m/s) — it's about a different variable (curvature), not a surface comparison. | Drop D2-S16 (and reconsider D2-S07) from this citation list; D2-S16 is actually a counter-example, as the README itself states correctly elsewhere (Summary line 16, Findings §4 line 73). |
| 97 | Recalibrate per terrain or use online/adaptive estimator when crossing materially different-friction terrains | [D2-S12 §V][D2-S15 §1][D2-S17 §6] | Verified | D2-S12 did train separate parameters per terrain (supports "recalibrate per terrain"); D2-S15/D2-S17 are genuinely online. | — |
| 98 | Run UMBmark first, then layer ICR correction (additive, different error sources) | [D2-S04 Abstract, §4.1][D2-S05 §III] | Verified | Consistent with both sources' stated scopes. | — |
| 99 | Set `wheels_per_side` to control-signal count; set `wheel_separation_multiplier` from measurement, not default 1.0, since ≥1 vendor config ships uncorrected | [D2-S09 "wheels_per_side"][D2-S27] | Verified | D2-S27 confirmed ships 1.0 (#4/#45). | — |

---

## Key numbers

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 100 | δm=2, ideal no-slip wheeled model | [D2-S02 §3.3.1 p.61] | Verified | Same as #20. | — |
| 101 | UMBmark "at least one order of magnitude" | [D2-S04 Abstract] | Verified | Same as #49. | — |
| 102 | Effective-width ratio ≈1.4× (xICR 0.26–0.31m vs half-width 0.20m), P3-AT 23.6kg asphalt | [D2-S05 Table I] | Verified | Same as #28. | — |
| 103 | Steering efficiency χ 0.69–0.76, P3-AT asphalt/concrete | [D2-S05 §IV.C] | Verified | Same as #29. | — |
| 104 | Dead-reckoning MSE (0.00010,0.00011) vs (0.00162,0.00455), "ICR model vs. factory model" | [D2-S05 Table III] | **Partly supported** | Same issue as #30: (0.00162,0.00455) is the fitted **Symmetric** column, not the as-shipped factory (**P3-AT**) column (0.00183,0.00468). | Relabel column, or swap in the true factory-model numbers. |
| 105 | Laser-scanner ICR model: pos.err <0.03m, angle <0.1rad, P3-AT a<3m/s², v<0.5m/s | [D2-S07 Conclusions] | Verified | Same as #32 (no page cited here, so the earlier page-location issue doesn't apply to this row). | — |
| 106 | Virtual width b̂=4.46m(concrete)/3.08m(snow), b=1.2m, Warthog 590kg | [D2-S12 Table I] | Verified | Same as #35. | — |
| 107 | Naive effective width B≈1.93–2.23m, physical 1.45m, 330kg UGV | [D2-S14 Tables 1–2] | Verified | Same as #37. | — |
| 108 | Pose-prediction error 1.8m→cm; σ −72/−83/−90%, Crusher grass/dirt/slopes | [D2-S17 §6.1 p.28] | Verified | Same as #54. | — |
| 109 | Effective turn rate ≈1/3, Crusher tight turns | [D2-S17 §6.1 p.31] | Verified | Same as #55. | — |
| 110 | GP model 5.6%/5.7% vs baseline 18.9%/6.2%, Jackal grass | [D2-S13 Table I] | Verified | Same as #71. | — |
| 111 | `wheel_separation_multiplier` shipped values: Husky 1.875, Jackal 1.5, Warthog 1.125, Husky humble-devel 1.0 | [D2-S24][D2-S25][D2-S26][D2-S27] | Verified | Same as #4. | — |
| 112 | Husky A300 spec row | [D2-S28] | Verified | Same as #63. | — |
| 113 | Jackal spec row | [D2-S29] | Verified | Same as #64. | — |
| 114 | Warthog spec row | [D2-S30] | Verified | Same as #65. | — |
| 115 | AM14U6 spec row | [D2-S31] | Verified | Same as #66. | — |
| 116 | Iorek drivetrain row | [D2-S32] | Verified | Same as #67. | — |
| 117 | Misti drivetrain row | [D2-S33] | Verified | Same as #68. | — |
| 118 | Jackal launch-era price row | [D2-S38] | Verified | Same as #69. | — |
| 119 | Skid vs explicit power, point turn, Nomad | [D2-S34 p.27] | Verified | Same as #87. | — |
| 120 | Skid-steer torque share ≈49–52% vs ≈5–8%, Nomad | [D2-S34 p.35] | Verified | Same as #88 (figure-confirmed). | — |
| 121 | Agri.Q 60%/40% front/rear weight split, 112kg total, standstill | [D2-S35 p.121] | Partly supported | Numbers correct (same as #90), but the parenthetical "(112 kg front, of total robot weight)" is ambiguously worded — could be misread as "the front weighs 112 kg," whereas the source says 112kg is the **total** robot weight. | Reword to "(112 kg = total robot weight)". |

---

## How it is tested

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 122 | Bidirectional square path (UMBmark): non-skid systematic error, order-of-magnitude reduction | [D2-S04] | Verified | Same as #49/#76. | — |
| 123 | Joystick paths + DGPS/laser GT, GA-fitted ICR params; minimized sum-sq pose-increment error | [D2-S05][D2-S07] | Verified | Same as #77/#78. | — |
| 124 | Constant-velocity + long-distance dataset; lower RMS error over 6s horizon vs baseline | [D2-S08] | Verified | "6-second prediction horizon" confirmed verbatim. | — |
| 125 | 180° U-turn, visual-SLAM + sim cross-check; normalised error (distance/radius) | [D2-S16] | Verified | Eq.(13) confirmed: error = distance / radius. | — |
| 126 | Multi-km field run, high-end IMU+DGPS GT, online EKF/IPEM calibration; σ-reduction vs no-slip baseline | [D2-S17][D2-S12] | **Partly supported** | True for Seegmiller/D2-S17. For Baril/D2-S12: ground truth is LiDAR+ICP (no GPS — see #80), and calibration is **offline** per-terrain parameter optimization (train on one trajectory, test on another), not an "online EKF/IPEM" method; its reported metric is median/IQR of relative translational/rotational error, not "σ-reduction vs no-slip baseline" in the Seegmiller sense. | Split into two rows: Seegmiller (IMU+DGPS, online EKF, σ-reduction) and Baril (LiDAR+ICP, offline per-terrain fit, median/IQR relative error). |

---

## Common mistakes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 127 | Applying no-slip diff-drive eqs to multi-wheel-per-side robots; both textbooks exclude skid-steer | [D2-S02 §3.3.1 p.61][D2-S03 ch.13 intro p.513] | Verified | Same as #21/#23. | — |
| 128 | Computing effective width from simultaneous outer/inner speeds w/o GT overstates true width | [D2-S14 Results] | Verified | Same as #37/#93. | — |
| 129 | Treating fitted multiplier as universal constant; every source testing >1 surface found it changed | [D2-S05 Tables I–II][D2-S12 Table I, §V][D2-S16 "Conclusions"] | **Not supported** (for the D2-S16 citation) | Same contradiction as #96: D2-S16's own conclusion is that terrain is **insignificant**, the opposite of "found it changed with surface." D2-S05 and D2-S12 are valid supporting citations here. | Drop D2-S16 from this citation list. |
| 130 | Yu et al. (tracked, cross-referenced from `C2_drive_kinematics`) Coulomb-friction caution, flagged not asserted for wheels | (cross-reference to sibling topic, no D2-S bracket) | Verified (as a disclosed cross-reference, not an uncited D2 claim) | The bullet explicitly says "cited in `C2_drive_kinematics`" and is flagged only as a caution — consistent self-disclosure of its provenance. | — |
| 131 | Relying on vendor config as evidence of calibration; Husky configs disagree (1.0 vs 1.875) | [D2-S27][D2-S24] | Verified | Multiplier values confirmed (#4/#45); the general point ("presence of a config is not evidence of a calibrated value") holds regardless of the separate wheel_separation-value discrepancy (#45). | — |
| 132 | Conflating caster-stabilised 2-wheel diff-drive with true multi-wheel skid-steer | [D2-S02 §3.2.3.3 p.50; Fig.3.13a p.59] | Verified | Same as #25. | — |

---

## Disagreements between sources

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 133 | Wheels-smaller-resistance hypothesis (D2-S19, secondhand) vs actual wheeled effective-width spread (1.31–3.7×) as wide as tracked's; not the same quantity, no single comparing source | [D2-S19 abstract, not downloaded] | Verified | D2-S19 quote reconfirmed via live fetch (#40); the "not the same quantity" distinction (turning-resistance coefficient vs. effective-width ratio) is a sound, clearly-labeled analytical point. | — |
| 134 | Grass harder/easier disagreement: Baril (snow fits better), Trivedi (grass lin. error lower than asphalt) vs Okawara (grass generalizes worst) | [D2-S12 §V][D2-S13 Table I][D2-S15 §4.2.2] | Verified | All three underlying facts independently confirmed above (#36, #71, #74). | — |
| 135 | Clearpath's own defaults disagree: `husky_control` (humble-devel) 1.0 vs `clearpath_common` A200 1.875, "same physical A200 geometry" | [D2-S27][D2-S24] | **Partly supported** | Multiplier values (1.0 vs 1.875) correct. But "the same physical Husky A200 geometry" is incorrect — the two configs' active `wheel_separation` values differ (0.512 m vs 0.555 m), the identical issue as #45. | Same correction as #45: drop or qualify the "same physical geometry" claim. |

---

## Open questions (cited)

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 136 | No source directly compares wheeled vs tracked turning resistance/scrub/effective-width under matched conditions; closest (Li et al. abstract) unverified against derivation | [D2-S19, not downloaded] | Verified | Consistent with the confirmed abstract-only status of D2-S19 (#40) and the absence of any other such comparison in the full set of downloaded sources reviewed. | — |
| 137 | Only one dated/secondhand Jackal price estimate (2014) found; not current pricing | [D2-S38] | Verified | Same as #69/#118. | — |

---

## Summary of corrections needed (consolidated)

1. **D2-S05 Table III / "factory model" (items #30, #104):** the cited MSE pair (0.00162, 0.00455) is the experimentally-fitted **Symmetric** model, not the as-shipped **factory** ("P3-AT") model — which is actually (0.00183, 0.00468). Relabel or swap the figure.
2. **D2-S12 / Baril ground truth (items #80, #126):** Baril et al. use **LiDAR + ICP**, never GPS, and calibrate **offline** per terrain — not "a high-end IMU and differential GPS" / "online EKF/IPEM," which is only true of Seegmiller (D2-S17). This mischaracterization appears twice (Findings §9 and the "How it is tested" table).
3. **D2-S16 miscited as a "terrain changed the fit" source (items #96, #129):** D2-S16's own Conclusion is that terrain type is **insignificant** — it directly contradicts the claim it is cited to support, and the README itself correctly reports this elsewhere (Summary, Findings §4). Drop D2-S16 from these two citation lists.
4. **Husky config "same physical geometry" (items #45, #135):** the two official Clearpath Husky configs disagree not only on `wheel_separation_multiplier` (1.0 vs 1.875) but also on the underlying `wheel_separation` value itself (0.512 m vs 0.555 m) — they are not "the same physical geometry."
5. **Uncited claim (items #46/#58):** "there is no separate 'skid-steer controller' in the current `ros2_controllers` package" has no citation at all.
6. Several **page-citation off-by-one errors**, all minor and all content-accurate at the adjacent page: D2-S07 (#32, cited p.9701 → actually p.9700), D2-S08 (#33, cited p.4 → quote actually on p.5), D2-S18 (#39, cited pp.2–3 → actually pp.3–4).
7. A handful of **quotation-fidelity nitpicks** (word dropped, paraphrase presented in quote marks, typo silently corrected, scare-quoted word not actually verbatim): items #1, #36, #40, #42, #85.
8. Minor **wording looseness**: "ceramic tile" vs the source's "ceramic floor/plate" (#5); Bruzzone Table 2 cells rounded away a "/medium" qualifier and one hybrid-category exception (#83); Mandow's solid-rubber-vs-pneumatic α comparison isn't uniformly true (#28); Agri.Q key-numbers parenthetical is ambiguous about what "112 kg" refers to (#121).

No fabricated numbers, invented quotes, or wrong-paper citations were found anywhere in the 135 items checked — the errors above are either (a) a mislabeled column, (b) a methodology mix-up between two papers sharing a citation bracket, (c) a source cited for the opposite of what it concludes, or (d) small page/word-level imprecision. The great majority of numeric figures, including several that required reading values off a bar chart rather than body text (D2-S34 Fig.21), were reproduced exactly.

---

## Corrections applied (2026-10-06)

Editor pass applying this VERIFICATION.md and the independent SOURCE_AUDIT.md. Every "Partly supported" and "Not supported" claim below was re-checked against the source (via `pdftotext -layout`, paginated with `\x0c`/form-feed splitting to get the true local page for each quote) before rewriting.

**Claims — rewritten to state exactly what the source supports:**
1. **#1 (Summary):** the verbatim quote "tanks and skid-steered vehicles are excluded" is now attributed to D2-S03 (Lynch & Park) alone; D2-S02 (Siegwart & Nourbakhsh) is described as excluding the equivalent case via its own "degenerate slip-skid" framing, not the same phrase.
2. **#28 (Findings §2):** "solid-rubber α slightly lower than pneumatic" qualified as "on average," with the one exception (asphalt, right tread: solid α=0.9128 > 20psi α=0.9049) stated explicitly.
3. **#30 / #104 (Findings §2 and Key numbers):** the Table III comparison was mislabeled "ICR model vs. factory model." Re-read directly: (0.00162, 0.00455) m² is a separately-fitted **symmetric** model, not the as-shipped **factory** P3-AT model, which is actually (0.00183, 0.00468) m². Both rows now give all three Table III columns (asymmetric / factory / symmetric) correctly labeled.
4. **#32 (Findings §2):** Wang et al. page citation corrected from p.9701 to **p.9700** — re-verified directly: PDF physical page 20 carries the running header "Sensors 2015, 15 ... 9700" and contains the "4. Conclusions/Outlook" section with the exact quoted sentence; p.9701 (PDF page 21) is the Acknowledgments/Author Contributions page.
5. **#33 (Findings §2):** Rabiee & Biswas citation expanded to "section V (starts p.4), quote at p.5" — re-verified the "accelerated motion" sentence is on PDF page 5, one page after the section header.
6. **#36 (Findings §2):** Baril et al.'s "performs better on snow than on concrete" is quoted as the source actually prints it — "snow **that** on concrete" — with an explicit `[sic]` note that this is evidently a typo in the source for "than."
7. **#39 (Findings §3):** Jia, Smith & Peng citation corrected from "section 3, pp.2–3" to "section 3 (starts p.3), quote at p.4" — re-verified: the §3 header is on local PDF page 3 (journal p.493); the "cylindrical wheel-soil contact area" quote is on local PDF page 4 (journal p.494); local page 2 is Table I, unrelated to §3.
8. **#42 (Findings §3):** the paraphrase "a fitting formula for calculating the skid-steering resistance coefficient" (D2-S39) was presented in quote marks; replaced with the abstract's actual wording, "the fitting formula of skid-steering resistance coefficient is given."
9. **#40 (Findings §3):** D2-S19's quoted phrase "with reference to steering theory of tracked vehicles" restored the dropped article: "with reference to **the** steering theory of tracked vehicles."
10. **#45 / #135 (Findings §4 and Disagreements):** removed the unsupported claim that the `husky` `humble-devel` and `clearpath_common` A200 Husky configs share "the same physical wheel-separation value" / "the same physical A200 geometry." The configs' actual `wheel_separation` fields differ (0.512 m vs. 0.555 m); both bullets now state this explicitly instead of asserting they match.
11. **#46 / #58 (Findings §6):** removed the uncited sentence "there is no separate 'skid-steer controller' in the current `ros2_controllers` package" (zero `[D2-S…]` bracket, unverifiable as written). The surrounding bullet now states only what D2-S09 supports — that `diff_drive_controller` is a single controller covering both cases via `wheel_separation_multiplier` — with a citation.
12. **#48 (Findings §4):** split the Baril "above 120 kg" sentence into two separately-cited clauses: the "above 120 kg... previously unstudied" clause keeps its [D2-S12, §I, p.1] citation; the "models transfer but magnitudes are platform/surface-specific" clause is now cited to [D2-S12, Table I, §V–VI], which is what actually supports it.
13. **#80 / #126 (Findings §9 and "How it is tested"):** corrected the claim that Baril et al. use "a high-end IMU and differential GPS" ground truth like Seegmiller et al. Re-checked: Baril's paper contains zero occurrences of "GPS"; its ground truth is a Robosense RS-32 LiDAR + ICP registration pipeline, and its calibration is an **offline** per-terrain parameter fit (not an online EKF/IPEM method). The "How it is tested" table row was split into two separate rows (Seegmiller: IMU+DGPS, online, σ-reduction metric; Baril: LiDAR+ICP, offline, median/IQR relative-error metric).
14. **#83 (Findings §10):** Bruzzone & Quaglia Table 2 synthesis corrected — wheeled slope-climbing capability is rated "low/medium" in the table, not pure "low"; noted that the WT-hybrid category shares the same "low/medium" floor on mechanical complexity rather than being strictly higher than wheeled/tracked on every cell.
15. **#85 (Findings §10):** removed the scare-quoted "any" around the Crusher generalization (not an actual quoted word at that location); the sentence now states plainly that the "general lesson for other six-wheel skid-steer vehicles" framing is this topic's own reasonable inference from Crusher's reported mechanism, not the source's own wording.
16. **#96 (Recommended practice #2) and #129 (Common mistakes):** removed D2-S16 from both "found the ratio/value changed with surface" citation lists — D2-S16's own Conclusion is that terrain type is **insignificant**, the opposite of what it was cited to support. Also removed D2-S07 from Recommended practice #2 (its Conclusions test one platform/surface, not a cross-surface comparison). Recommended practice #2 now adds an explicit note flagging D2-S16 as a counter-example instead.
17. **#121 (Key numbers):** reworded the ambiguous Agri.Q parenthetical from "(112 kg front, of total robot weight)" to "(112 kg = total robot weight)" to remove the "front weighs 112 kg" misreading.
18. **D2-S15 citation locators (Findings §5, §8, Disagreements):** replaced the extraction-tool-dependent "lines ~1266–1378 of extracted text" / "around line 1377" / bare "section 4.2.2" citations with a verified section-and-page locator, "section 4.1, Table 1, p.10" — directly confirmed by re-running `pdftotext -layout` with page-break splitting on the saved PDF: §4.1 "Verification evaluation of the proposed neural network" starts on local PDF page 9, and the Table 1 / Case 1 / Case 2 discussion containing the quoted sentence is on local PDF page 10.

**Sources — corrected per SOURCE_AUDIT.md:**
19. **D2-S19, D2-S39:** evidence level relabeled **C (abstract only) → D**, matching STANDARDS §2's definition of D ("used only when A–C are missing") rather than C.
20. **D2-S38:** evidence level relabeled **C → D** (named-author journalism about an informal company quote is neither official documentation (B) nor open-source code/team report (C)).
21. **D2-S31:** evidence level relabeled **B → A** (AndyMark's own product page specifying its own product is a manufacturer specification, the same category already rated A for the Clearpath datasheets D2-S28–30).
22. **D2-S20, D2-S21:** source files renamed `ros2controllers_2024_diff_drive_controller.cpp` → `ros2controllers_2026_diff_drive_controller.cpp` and `ros2controllers_2024_diff_drive_parameters.yaml` → `ros2controllers_2026_diff_drive_parameters.yaml` to match the actual 2026 commit/access date; README's Sources table updated to the new filenames.
23. **D2-S09, D2-S22, D2-S31, D2-S36, D2-S38:** re-saved with site navigation/sidebar/banner chrome stripped (GitHub sign-in/Copilot banners, Sphinx doc-tree sidebar, AndyMark ~680-line nav menu, Nav2 sponsor banner, IEEE Spectrum site nav) so each file opens directly with the real heading and body text, per STANDARDS §3's "clean markdown/text" requirement for web documentation. All cited facts/quotes in each file were confirmed still present after trimming.

**Verified but no action needed:** all remaining "Verified" rows (116 of 137), the "0 failing sources" finding, the foundational-reference completeness finding (10/11 downloaded + 1/11 correctly marked not-downloaded), and the coverage finding (all 16 subtopics cited) from SOURCE_AUDIT.md required no changes.

**Mechanical check (post-correction):** `file` run on all 36 files in `sources/` — every type matches its extension (24 PDF, 7 Unicode/ASCII text .md, 4 ASCII .yaml, 2 C++ source .cpp/.hpp — note the renamed D2-S20/S21 files are now reported as "Unicode text, UTF-8 text" / "ASCII text" by `file`'s content-sniffing, same genuine source code as before, extension unchanged). Every one of the 36 downloaded files appears in the Sources table exactly once; D2-S19 and D2-S39 remain correctly marked "not downloaded." Every `[D2-S…]` ID cited anywhere in README.md exists as a row in the Sources table (verified by set comparison — zero cited IDs missing from the table). Source IDs run D2-S01–S09, D2-S11–S39 with no duplicates in the Sources table and no gap other than the pre-existing, documented non-use of D2-S10.

**Status:** README.md's Status field changed from **Draft → Verified** — after the above corrections, no "Not supported" or "Partly supported" claim and no failing source remain outstanding.
