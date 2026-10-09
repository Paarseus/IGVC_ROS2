# VERIFICATION — D4 Omnidirectional (mecanum/omni) drive

**Topic:** D4_omnidirectional_drive
**Date:** 2026-10-06
**Reviewer:** independent — claims (Step 4, STANDARDS.md §5). Sources were opened at the cited location with `pdftotext -layout` (PDFs) or HTML-to-text extraction and checked against each claim. Source quality/format/evidence-level is **not** graded here — that is `SOURCE_AUDIT.md`'s job (a separate reviewer).

## Counts

| Status | Count |
|---|---|
| Verified | 103 |
| Partly supported | 16 |
| Not supported | 5 |
| **Total claims checked** | **124** |

(Counts are of distinct claim-rows below, not of individual sentences; several README bullets bundle 2+ checkable sub-claims under one citation and are scored as one row for the dominant/riskiest sub-claim, with secondary issues noted in the Correction column.)

---

## Summary

| # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|
| 1 | Mecanum/omni = controlled-sliding constraint; contact handoff is root cause of vibration/odometry error/terrain sensitivity | D4-S09 ch.13 pp.513–518; D4-S08 pp.50–52; D4-S11 abstract | Verified | Bae abstract: "vertical and horizontal vibrations due to the sequential contacts between rollers and ground" (bae_2016...txt); Lynch&Park p.515: "work best on hard, flat ground"; Siegwart pp.50–52 gives the γ-constraint derivation. The specific "root cause of vibration/odometry/terrain" synthesis is a reasonable joint read of the three, not an exact quote from any one. | None — wording is attributed to the set jointly, not overstated. |
| 2 | Three rough-terrain papers reject/limit mecanum for outdoor use | D4-S14 abstract/§1; D4-S15 abstract; D4-S16 pp.12–16 | Verified | Iagnemma 2009 (iagnemma_2009...txt l.27-29): "slender rollers can easily become clogged with dirt and debris"; Ishigami 2012 l.83-84: "most of them are designed for... flat, smooth terrain, and are not feasible for outdoor usage"; Guzman Franco thesis pp.12–13 literature review confirmed. | None. |
| 3 | 4 IGVC design reports, split outcome (Oakland/Bluefield/Buffalo) | D4-S18 pp.2,16; D4-S20 p.2; D4-S19 pp.4–5 | Partly supported | p.16 Oakland quote verified verbatim (oakland...txt, "8 Conclusion" page); p.2 Bluefield quote verified verbatim ("had problems in grass fields... complications of a mecanum controller," Page 2 footer); Buffalo pp.4–5 verified. **But** Oakland's own p.2 (Introduction/team roster) contains no text supporting this sentence — it is only generic project framing, not the "ran the experiment" claim. | Drop "p.2" from the D4-S18 citation, or cite only p.16. |
| 4 | Palacín flower-trajectory calibration, 82.14% | D4-S12 abstract | Verified | "an average improvement of 82.14% in the estimation of the final position and orientation" (palacin...txt, abstract). | None. |
| 5 | ros2_controllers mecanum timeline; MPPI Omni model; DWB omni support | D4-S26a; D4-S26b; D4-S27; D4-S29 line 11 | Verified | CHANGELOG 4.17.0 "Add Mecanum Drive Controller (#512)"; Humble 2.43.0 backport confirmed; MPPI config guide motion_model options confirmed; DWB README line 11 confirmed verbatim. | None. |
| 6 | No catalog product validated for sustained outdoor grass/dirt/mud use | D4-S16 pp.12–16; D4-S23; D4-S24 | Verified | Synthesis of already-verified Findings §5, §8, §9, §11 rows below; no source found in this review contradicts it. | None. |
| 7a | Gfrerrer: torus is only an approximation of the real mecanum roller, exact only for 90° omni | D4-S31 Theorem 1 | Verified | "Exactly in case of Swedish wheels (δ = ±π/2) the torus surface T and the roll surface R are identical" (gfrerrer...txt l.405-406); "have contact of order 3" (l.401-403). | None. |
| 7b | Galati 2022: 20–60% higher motor current, ~13 dB/Hz higher vibration, asphalt vs. concrete | D4-S34 "Experimental results" | Verified | 5.4→6.5 A (+20%), 10.4→16.6 A (+60%), −49.29→−36.54 dB/Hz (Δ12.75≈13 dB/Hz) (galati...txt l.538-560). | None. |

---

## Foundational references

| # | Reference | Status | Evidence | Correction needed |
|---|---|---|---|---|
| 8 | D4-S01 Ilon patent 3,876,255, filed 1972 / issued 1975, "assignee: AB Mecanum" | Partly supported | Patent text confirms filing date "Nov. 13, 1972", issue date "Apr. 8, 1975", inventor "Bengt Erland Ilon" (ilon_1975...txt l.1-9). **No "Assignee" field or "Mecanum AB" string appears anywhere in the extracted patent text** — the patent lists only the individual inventor. | Either remove "(assignee: AB Mecanum)" or re-cite it to Lynch & Park's footnote (which does say Ilon worked at "the Swedish company Mecanum AB," ch.13 p.514 fn.1), not to the patent itself. |
| 9 | D4-S02/S03/S05/S06 "not downloaded" (paywalled / CAPTCHA-blocked) | Verified (for the checkable part) | Confirmed: no file for any of these four IDs exists in `sources/`, consistent with "not downloaded." The narrative detail (OSTI biblio record, AWS WAF CAPTCHA, cookie-primed requests) describes the research *process* and cannot be independently re-verified from local files or without live web access. | None for the stated status; this class of claim is inherently outside what a local-file check can confirm or refute. |
| 10 | D4-S04 Campion et al. — type (3,0) "omnimobile," δm=3/δs=0, worked example p.743 | Verified | Russian-translation text p.742: "Такие роботы носят название омнимобильные роботы" (robots of this type bear the name omnimobile robots) — and no other of the 5 types is given this name anywhere else in the document (checked). p.743 worked example: α=π/3,π,5π/3, β=γ=0, J2=diag(r) (campion...txt l.549-569). | None. |
| 11 | D4-S07 Diegel et al. 2002 — rollers-held-from-outside vs. centrally-mounted-split-roller design | Verified | "peripheral rollers held in place from the outside... the rim of the wheel can make contact with the surface instead of the roller" / "rollers split in two and centrally mounted... ensures that the rollers are always in contact" (diegel...txt). | None. |
| 12 | D4-S08 Siegwart & Nourbakhsh ch.3 — Swedish-wheel constraint, γ=0/π/2 degenerate cases | Verified | pp.50–52, eq. 3.18–3.19 exactly as described (siegwart...txt l.568-625). | None. |
| 13 | D4-S09 Lynch & Park ch.13 — H(φ) Jacobian, min. 3 wheels, p.515 "hard, flat ground" | Verified | "An omnidirectional mobile robot must have at least three wheels..." (l.23934-23936); "omniwheels and mecanum wheels work best on hard, flat ground" (p.515, l.23925-23926). | None. |
| 14 | D4-S31 Gfrerrer 2008 as "primary source" for roller geometry, first rigorous derivation | Verified | Confirmed content matches description (Theorem 1 derivation of the exact roll surface and torus approximation). | None. |

---

## Findings §1 — Omnidirectional wheel kinematics theory

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 15 | Swedish-wheel γ constraint; γ=0 → Swedish-90, γ=π/2 degenerate | D4-S08 pp.50–52, eq.3.18–3.19 | Verified | Exact match, siegwart...txt l.585-625. | None. |
| 16 | Lynch & Park: driven component vx+vy·tanγ, free component vy/cosγ; H(φ)q̇; min 3 wheels | D4-S09 ch.13 pp.515–518, eq.13.3–13.6 | Verified | Eq. 13.3–13.4 exact match (lynch...txt l.23949-23990); 3-wheel minimum exact match. | None. |
| 17 | Campion (3,0) classification, URANUS/UCL examples, distinct from swerve at (1,2) | D4-S04 p.742 | Partly supported | (3,0)="omnimobile," URANUS/UCL examples confirmed verbatim. **The paper never uses the word "swerve"** — mapping type (1,2) ("≥2 independently-oriented conventional centrally-orientable wheels") onto "swerve chassis" is the README's own (reasonable) technical inference, not stated in the source and not flagged as an inference. | Label the swerve↔(1,2) equivalence as an inference, or cite the sibling D1 topic where that mapping is presumably made explicit. |
| 18 | Campion worked example: 3-wheel (3,0), α=π/3,π,5π/3, β=γ=0, J1/J2=diag(r) | D4-S04 p.743 | Verified | Exact numeric match, campion...txt l.549-569. | None. |
| 19 | Roller passive rotation = unsensed DOF; "no sensor to confirm" slip | D4-S08 pp.50–52; D4-S09 ch.13 p.516 | Partly supported | Both sources confirm the driven/free-sliding split and that only the driven component is encoder-readable. The specific framing "the odometry model must assume... with no sensor to confirm it" is the README's own synthesis, not a quote from either text. | Minor — flag as inference, or soften to avoid implying it's a direct quote. |
| 20 | Tagliavini 2022: 3WD least affected by speed limits (but under-uses motors); 4WD better for preferential direction; both need high roller speed → lower efficiency/higher vibration tied to build quality | D4-S10 Abstract; §4 Conclusion | Verified | Exact quotes: "3WD locomotion system mobility seems less affected by the wheel speed limitation, even if... one or even two motors are not exploited"; "a 4WD robot is more appropriate when there is a preferential direction of motion"; "characterized by lower efficiency and a higher level of vibration, strongly related to the construction quality of the wheels" (tagliavini...txt l.760-769). | None. |

## Findings §2 — Wheel/mechanism design variants and roller geometry

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 21 | Mecanum "usually used for wheelchair/forklift"; omni usually 3–4 wheels, 4-wheel preferred for stability | D4-S11 Introduction | Verified | "it is usually used for a wheel chair or fork lift"; "four-wheeled vehicle is generally preferred for the stability of the platform" (bae_2016...txt Introduction). | None. |
| 22 | Diegel/Ilon: outer-mounted rollers → rim contact on uneven ground; split centrally-mounted rollers fix it; "lineage most later designs build on" | D4-S07 §2 | Partly supported | First two clauses verbatim-confirmed. The clause "is the lineage most later 'improved' mecanum designs build on" is not stated in D4-S07 (a 2002 paper can't characterize "later" literature) — it's the README's own broader-literature claim, uncited to the specific later papers it implies. | Cite this specific clause to Adamov 2024 (which does discuss the historical lineage) or mark as editorial framing. |
| 23 | Straight-line force cancellation; diagonal travel 2 wheels drag; lockable-roller and 135°-rotatable-roller fixes | D4-S07 §3–4 | Verified | "only a front and rear opposing wheels are spinning whilst the rollers on the other two wheels cause direct drag"; "pivoted through 135°" (diegel...txt). | None. |
| 24 | Vibration from sequential discontinuous roller-ground handoff | D4-S11 Abstract | Verified | "vertical and horizontal vibrations due to the sequential contacts between rollers and ground." | None. |
| 25 | Lynch & Park: omniwheel vs. mecanum roller-axis distinction; terminology "not completely standard"; "work best on hard, flat ground" | D4-S09 ch.13 pp.513–515 | Verified | All three clauses verbatim-confirmed (lynch...txt l.23903-23926). | None. |
| 26 | AndyMark 8" MK wheel: 12 rollers, brass-tube axle, nylon-core/80A TPU overmold | D4-S23 | Verified | "Rollers: 12", "Roller Axle: Brass Tube, 1/4 in.", "Roller Durometer: 80A"; "nylon core and a 80A durometer TPU overmold" (product page). | None. |
| 27 | Rotacaster roller counts/durometers at 125mm/35mm/50mm | D4-S24, "via distributor-reproduced spec pages" | Not supported | Searched both downloaded Rotacaster HTML files exhaustively for "40A," "85A," "99A," "35A," "60A," "95A," "8 roller(s)," "4 roller(s)," "polyurethane," "TPE" — **none of these strings appear in either file.** The cited source contains no page for any individual wheel size's roller spec, only top-level navigation listing diameters. | This entire bullet's figures are not traceable to any downloaded source. Either locate and download the actual distributor spec pages as a new, separately-ID'd source, or remove the figures. |
| 28 | TENTE: fully-custom Mecanum line, Novatech/Ultratech/Ultratech-plus, AGV/AMR framing, no outdoor claim | D4-S35 | Verified | "Novatech, Ultratech and Ultratech plus"; "automated guided vehicle (AGV) and autonomous mobile robot (AMR) applications"; no outdoor/rough-terrain language anywhere in the 2-page flyer (tente...txt). | None. |
| 29 | Gfrerrer: intuitive ellipse/torus assumption is wrong; exact surface derived from circular-contact-point condition; Theorem 1 | D4-S31 §1 Abstract, §3, Theorem 1 | Verified | Confirmed (see Foundational refs row 14 and Summary row 7a). The parenthetical gloss "('osculating')" is a loose synonym for "contact of order 3" — the paper uses "osculate" once, in an adjacent but not identical context (gfrerrer...txt l.320). | Minor terminology looseness; not a substantive error. |
| 30 | Torus = exact only at δ=±π/2 (plain omni), not at 45° mecanum | D4-S31 §3, Theorem 1 | Verified | "Exactly in case of Swedish wheels (δ = ±π/2) the torus surface T and the roll surface R are identical" (l.405-406). | None. |
| 31 | Adamov 2024: flat-surface roller geometry "studied in sufficient detail"; spherical-surface case "requires further solutions" | D4-S32 p.46 | Verified | Exact quote match, p.46 marker confirmed immediately above the text (adamov_2024...txt l.150-158). | None. |
| 32 | WPILib: swerve wheel force vector "straight forward... rather than at a 45 degree angle as in mecanum drive," "more efficient... and better traction" | D4-S37 | Verified | Exact quote match (wpilib...html). | None. |

## Findings §3 — Precision-limiting mechanical and control factors

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 33 | Bae & Kang: asymmetric vertical vibration, confirmed in RecurDyn simulation | D4-S11 Abstract | Verified | "vertical accelerations were asymmetric... confirmed through the dynamic simulations performed by RecurDyn." Affiliation "Kyungpook National University" confirmed. | None. |
| 34 | Optimal roller curvature "3rd–7th"; optimal fork spring 200–250 N/mm; optimal fillet radius 2–3 mm | D4-S11 §3–4, Table 2 | Verified | "optimal curvature is in the range between 3rd and 7th curvatures"; "optimal spring stiffness was determined at the range between 200 and 250 N/mm"; "optimal spring value of 250 N/mm was shown in between the fillet radii of 2 and 3 mm" (bae...txt l.367-472), Table 2 interpolation (201.21 N/mm @ r=0, 272.67 @ r=3) consistent. | None. |
| 35 | Diegel's straight-line loss exists even on a perfectly hard, flat, debris-free floor | D4-S07 §3 | Verified | Restating already-confirmed §3 text; the "not a terrain effect at all" framing is a fair paraphrase of a mechanism independent of surface. | None. |
| 36 | Tagliavini: high roller speed → lower efficiency/higher vibration tied to build quality | D4-S10 §4 Conclusion | Verified | Already confirmed above (Findings §1 row 20). | None. |
| 37 | Adamov & Saypulaev 2020: 3 models (1 nonholonomic, 2a linear-viscous, 2b Coulomb), f=0.5 | D4-S33 §4–§5, p.291 Abstract | Verified | "Model 1 is nonholonomic"; "linear friction (4.10) (Model 2a)"; "Coulomb friction (4.11), (4.17) (Model 2b)"; "we assume the value f = 0.5" (adamov_saypulaev...txt l.459-494, l.563). | None. |
| 38 | Contact-point shift reduces angular/lateral control efficiency, raises energy use, deforms path; "switching... causes vibrations" | D4-S33 p.306, §6 Conclusion | Verified | Verbatim quote match on p.306 (l.738-754). | None. |
| 39 | Galati 2022 "Omnibot" wheel-radius variation 0.1480–0.1524 m over a "half-roller-pitch (30°) cycle" on a 12-roller wheel | D4-S34 "Rolling radius" §, Table 3 | Partly supported | Numbers verified exactly (Table 3: 0.1524→0.1480→0.1524 across 0°→15°→30°); "twelve rollers" confirmed. **But 30° is the *full* roller pitch for a 12-roller wheel (360°/12=30°), not a "half" pitch** — the data in Table 3 spans exactly one complete pitch, not half of one. | Change "half-roller-pitch (30°) cycle" to "one full roller-pitch (30°) cycle." (The separate Key-numbers-table row for the same fact says "periodic every 30°" and is correct as written.) |
| 40 | Galati: slip-onset torque condition, µ=0.5 concrete / µ=1.0 asphalt (asphalt higher friction) | D4-S34 "Wheel slipping," eq.16 | Verified | "T ≤ Tmax = µFz sin(45)r" (eq.16, l.293); "µc = 0.5 for industrial concrete floor and µa = 1 for asphalt" (l.291). | None. |
| 41 | Galati: adaptive surface-classifier framework, Ac/Bc parameters | D4-S34 "Omnibot pose estimation..." § | Verified | "The parameters Ac and Bc are generated by a surface classifier" (l.475), eq.28. | None. |

## Findings §4 — Odometry and calibration methods

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 42 | Palacín 2022: 36 calibration trajectories, flower shape, human-sized 3-wheel personal-assistant robot | D4-S12 Abstract | Verified | "36 individual calibration trajectories which together depict a flower-shaped figure"; "designed as a versatile personal assistant tool" (palacin...txt). | None. |
| 43 | 82.14% improvement is systematic-error correction only, not terrain slip | D4-S12 Abstract | Verified | Title itself: "Systematic Odometry Error Evaluation and Correction..."; the inference that this doesn't characterize terrain-induced slip is a fair, clearly-labeled reading of the paper's stated scope. | None. |
| 44 | Lynch & Park front-matter: odometry "can be solved in the same way for both" robot classes, p.10 | D4-S09 front-matter preview, p.10 | Verified | Exact quote match, "10" page marker immediately precedes it (lynch...txt l.1001-1019). | None. |

## Findings §5 — Terrain interaction and traction limits

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 45 | Lynch & Park: "work best on hard, flat ground" | D4-S09 ch.13 p.515 | Verified | Confirmed above. | None. |
| 46 | Iagnemma 2009 (MIT/IIT/TARDEC): "nearly all designs... flat, smooth terrain"; "not suitable for outdoor... slender rollers... clogged with dirt and debris"; ASOC response | D4-S14 Abstract/§1 | Verified | Exact quote match (iagnemma...txt l.23-29). | None. |
| 47 | Ishigami 2012: "most... flat, smooth terrain, and are not feasible for outdoor usage"; "specialized wheel designs" incl. roller/Mecanum/spherical; parallel-link+shock-absorber suspension "to conform to uneven terrain" | D4-S15 Abstract/§1 | Verified | Exact quote match (ishigami...txt l.10,16,33,83-89). | None. |
| 48 | Guzman Franco thesis p.12: US Navy 39" wheel, "excellent capability... ramps, obstacles, mud and sand," 3" obstacles = 7.5% of diameter, "very limited" | D4-S16 p.12 | Verified | Verbatim quote match, p.12 marker confirmed (guzmanfranco...txt l.721-744). | None. |
| 49 | Guzman Franco p.13: EADS Astrium Mars Cruiser One — sinkage, lateral slippage, obstacle success up to roller diameter, failure at ~40% wheel diameter, best approach 45° | D4-S16 p.13 | Verified | Verbatim quote match, p.13 marker confirmed (l.745-769). | None. |
| 50 | Both studies "confirmed the expected limited capabilities... most critical... climb obstacle higher than the diameter of the peripheral rollers" | D4-S16 p.13 | Verified | Exact verbatim quote, immediately after p.13 marker (l.770-773). | None. |
| 51 | Gugumuck & Paugger: grass 92.17%/2.82in (mecanum) vs. 90.90%/3.23in (Solarbotics); stone/LEGO "N/A ... failed completely"; mecanum 84.71%/82.47% on its worst terrains; "performed similarly on flat and grassy"; grip/debris qualitative notes | D4-S13 §III, §IV.B, Tables I–II | Verified | All numbers match Tables I/II exactly; "failed completely on rocks and LEGO bricks" and "performed similarly on flat and grassy surfaces" both verbatim (gugumuck...txt l.31,153-166,191-196). | None. |
| 52 | Rotacaster "All Terrain Rotatruck" quote is about a hand-truck caster, not a 4-wheel drivetrain | D4-S24 | Verified | "With a 250mm (10") puncture proof rear wheel the All Terrain Rotatruck navigates with ease... Rotacaster multidirectional wheels guarantee high manoeuvrability" — confirmed this is the front-caster-only product, not a drive wheel (rotacaster_all_terrain...html). | None. |
| 53 | Galati: concrete-vs-asphalt current/vibration, "warehouses" framing (not unpaved) | D4-S34 "Experimental results," Figs.9,11–13 | Verified | All current/PSD numbers confirmed (see Findings §3 row 39-style checks above); "intended to operate in warehouses, where concrete and..." confirmed (l.494). | None. |
| 54 | Galati: sideways current ~2× forward on both surfaces, "due to the sliding of the rollers placed at 45°" | D4-S34 "Experimental results" | Verified | Exact quote match (l.544-545). | None. |

## Findings §6 — Failure modes, durability, maintenance

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 55 | Debris clogging is the IIT/MIT/TARDEC team's stated *deciding* design factor | D4-S14 §1 | Verified | §1 Introduction frames it as the direct reason the ASOC design was chosen instead of roller wheels. | None. |
| 56 | Buffalo Big Blue: 12 rollers/8.5" rims, grooves for traction, small-roller-diameter obstacle problem at zero-point turns, "no problems... competition," "larger rollers would improve... rougher terrain" | D4-S19 p.5 | Verified | All quotes verbatim-confirmed on p.5 (buffalo...txt). | None. |
| 57 | 48-roller component-count inference (4×12), no MTBF data found | — (self-labeled inference) | Verified | Correctly and explicitly labeled "[Inference from component counts... no field reliability/MTBF study... found]" — good practice, properly hedged. | None. |

## Findings §7 — Field reports from other domains

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 58 | Gugumuck & Paugger is the only controlled same-rig mecanum-vs-non-omni comparison incl. grass found | D4-S13 | Verified | Consistent with everything else reviewed; no contradicting source found. | None. |
| 59 | AGRIMARO.Q access-restricted; unverified summary suggests swerve not mecanum | (none — explicitly flagged as unverified) | Verified | Honestly self-labeled as "unverified search-result summaries, not... reading the paper," correctly placed as a non-finding. | None. |
| 60 | Rotacaster/TENTE = indoor-material-handling-market framing | D4-S24; D4-S35 | Verified | Both already confirmed (rows 27-partial and 28). | None. |
| 61 | Galati fills an industrial-heavy-duty gap but still scopes itself to "warehouses" | D4-S34 Introduction/"Experimental results" | Verified | "no heavy duty industrial example has been discussed" (l.40-41) and "warehouses" framing (l.494) both confirmed. | None. |
| 62 | EADS Astrium Mars Cruiser One reported secondhand via Calgary thesis, not read directly | D4-S16 p.13 | Verified | Correctly and explicitly flagged as secondhand; consistent with Sources table listing no separate Astrium-authored source. | None. |
| 63 | TARDEC-funded MIT/IIT ASOC rejects roller wheels for off-road UGV | D4-S14 Abstract/§1 | Verified | Already confirmed (row 46). | None. |

## Findings §8 — Off-the-shelf components

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 64 | AndyMark 8" wheel: 8in dia., 3.50in width, 4.58lb, 500lb load, 12 rollers, 1/4in brass axle, 80A TPU, polycarbonate hub w/ riveted steel side plates, $520/$133 | D4-S23 | Verified | Every figure matches the product page exactly, incl. "Strong, steel side plates are riveted to a black polycarbonate core" and "$133.00 USD – $520.00 USD." | None. |
| 65 | Rotacaster diameters incl. 125mm; durometer specifics | D4-S24 | Partly supported | The 2026 snapshot (rotatruck_product_range.html) lists **"127mm Omni Wheel,"** not 125mm; only the older 2018 snapshot (all_terrain_rotatruck_pro.html sidebar) says "125mm Rotacasters." The README states only "125mm" without flagging this cross-snapshot discrepancy. Durometer figures (40A/85A/99A etc.) are unsupported — see row 27. | Note both "125mm" (2018 snapshot) and "127mm" (2026 snapshot) or pick one and say so. |
| 66 | UBC Avalanche: AndyMark wheels + ToughBox, "proper gear configuration," "safe and reliable," $1,108 for 4 wheels+gearboxes (2nd-largest line item after $1,149 laptop) | D4-S21 p.4 (drivetrain), p.12 (cost table) | Verified | Both quotes verbatim-confirmed on p.4; cost table on p.12 confirms exact figures. The "2nd-largest" framing is justified by the report's own text: "the only costly components for this new vehicle were the Mecanum wheels and the new laptop" (p.12) — i.e. the $3,000 LIDAR line was pre-owned, not a new cost, so the report's own framing supports ranking mecanum 2nd among *new* purchases. | None (read with the source's own "only costly components" framing). |
| 67 | SuperDroid/Nexus pricing "could not be verified" (Cloudflare block) | — (explicitly flagged as not found) | Verified | Honestly self-labeled; no source file exists for this vendor, consistent with the claim. | None. |
| 68 | TENTE = 3rd wheel-only vendor, fully customized, no published price | D4-S35 | Verified | Confirmed (row 28). | None. |
| 69 | No wheel-only product ships an integrated encoder; odometry via separate motor/gearbox-with-encoder (UBC); "dedicated motor controller per wheel" is "the consistently documented pattern" across Oakland/Buffalo/UBC/KUKA | D4-S18 §Drive Control System; D4-S19 §2.2; D4-S21 p.4; D4-S09 ch.13 p.516 Fig.13.2 | **Not supported** | **(1)** Buffalo's actual motor-controller section (§2.6, not the cited §2.2) states: "Each controller has two channels capable of driving **two separate motors**" (buffalo...txt l.235-236) — i.e. Buffalo explicitly does **not** have one controller per wheel; it shares controllers across wheel pairs, directly contradicting the claim this source is cited for. **(2)** Oakland's cited §5.1 "Drive Control System" states all 4 wheels' PI loops run "on a PIC processor" (singular) — one centralized controller, not "dedicated... per wheel." **(3)** UBC Avalanche p.4 (the actual cited page) discusses only the wheel+gearbox pairing and never mentions motor controllers or encoders at all; its cost table (p.12, not p.4) lists only "Encoder: 2" for a 4-mecanum-wheel vehicle, not one encoder per wheel. **(4)** Lynch & Park ch.13 p.516/Fig.13.2 is a figure caption describing wheel motion, with no mention of controller architecture. | Remove or substantially rewrite this claim — at least 3 of its 4 citations do not support it, and the Buffalo source actively contradicts it. |

## Findings §9 — Complete mobile robot platforms

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 70 | Clearpath Ridgeback: 960×793×296mm, 135kg, 100kg payload, 1.1m/s, 2000W/800W, 24V 100Ah, encoders >250,000 CPM (1/wheel; "separately reported... as 35,840 pulses per revolution"), IMU, 18mm clearance, indoor-only marketing | D4-S22a (datasheet); D4-S22b (brochure) | Partly supported | Every other figure matches the datasheet exactly (dimensions, weight, payload, speed, power, battery, encoders, clearance, "Omnidirectional 'Swedish' wheels," no outdoor application listed). **"35,840 pulses per revolution" does not appear anywhere in either cited file** — searched both exhaustively, no match. | Remove the "35,840 pulses per revolution" parenthetical, or cite its actual source (it is not D4-S22a/b). |
| 71 | Research ASOC platform uses conventional wheels + suspension instead of rollers, 25kg prototype | D4-S14 Abstract; D4-S15 Abstract | Verified | Consistent with already-confirmed Iagnemma/Ishigami text. | None. |
| 72 | All 4 IGVC mecanum platforms are custom-built, not catalog products | D4-S18; D4-S19; D4-S21; D4-S20 | Verified | Consistent with all four design-report excerpts already read in full. | None. |

## Findings §10 — ROS 2 / ros2_controllers / Nav2 maturity

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 73 | mecanum_drive_controller: shared functionalities for 4-mecanum-wheel bases, generic odometry, TwistStamped-style command (x/y/z used, others ignored) | D4-S25 | Verified | "Library with shared functionalities for mobile robot controllers with mecanum drive (four mecanum wheels)... generic odometry"; "linear x, y, and angular z components are used. Values in other components are ignored" (userdoc html). | None. |
| 74 | Publishes Odometry + TF, configurable command timeout | D4-S25 | Verified | "odometry publishing as Odometry and TF message; input command timeout based on a parameter" (userdoc html). | None. |
| 75 | Controller added in 4.17.0 (2024-12-07, PR #512); Humble backport 2.43.0 (2025-03-17) | D4-S26a; D4-S26b | Verified | Both changelog entries match exactly. | None. |
| 76 | MPPI motion_model: DiffDrive/Omni/Ackermann, descriptions verbatim; Omni activates vy_max/vy_std, "otherwise unused/ignored for a non-holonomic model" | D4-S27 | Partly supported | motion_model description and vy_max's own doc text ("Target maximum lateral velocity, **if using "Omni" motion model**") both verbatim-confirmed. **But vy_std's own doc text is just "Sampling standard deviation for Vy"** — it does not itself state the Omni-only caveat the way vy_max's does. | Either cite the C++ source (D4-S28) for the vy_std behavior, or soften to "vy_max explicitly, vy_std by the same design pattern." |
| 77 | motion_models.hpp: OmniMotionModel/DiffDriveMotionModel are thin subclasses; only override is isHolonomic() (true/false); comment "using Y axis" | D4-S28 lines 339–376 | Verified | Exact code match, lines 339-377 (nav2_mppi_motion_models...hpp). | None. |
| 78 | DWB README: "trajectory generator plugins work for omnidirectional and differential drive robots"; Twirling critic "prevent[s] holonomic robots from spinning" | D4-S29 line 11 | Verified | Exact quote match at line 11; Twirling bullet verbatim-confirmed. | None. |
| 79 | No Humble-specific mecanum/omni bug reports found (negative claim) | — (open question, self-flagged) | Verified | Honest, appropriately hedged negative/absence claim. | None. |
| 80 | Per-package changelog: 2.43.0 bundled "Fix Odometry Initialization" (#1573) alongside #512; further Humble fixes at 2.49.0/2.50.1/2.52.1 | D4-S36 | Verified | All four changelog entries match exactly, verbatim (mecanum_drive_controller CHANGELOG humble file). | None. |
| 81 | Humble is "missing" 3 specific rolling-line items: set_odometry service (PR #2110, 6.4.0, 2026-03-12), velocity-limiting (PR #2313, 6.8.0, 2026-07-01), halt-logic safety fix (PR #2326, 6.9.0, 2026-08-12) | D4-S36 | **Not supported** | D4-S36, per the Sources table, is the **Humble-branch** per-package changelog only (confirmed: read the complete 82-line file; none of PR #2110/#2313/#2326 or versions 6.x appear in it — unsurprising, since it's the Humble file). **No rolling-branch mecanum_drive_controller changelog file exists anywhere in `sources/`** (checked directory listing). The specific PR numbers, release versions and dates attributed to "rolling" are therefore not traceable to any downloaded source. | Either download and separately cite the actual rolling-branch `mecanum_drive_controller/CHANGELOG.rst` as a new source ID, or remove the specific PR/version claims and keep only the (verifiable) "absent from Humble's own changelog" half. |

## Findings §11 — Net verdict literature

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 82 | IIT/MIT/TARDEC: clearest explicit "unsuitable for outdoor" position | D4-S14 §1 | Verified | Already confirmed (row 46). | None. |
| 83 | Bluefield: "had problems in grass fields... complications of a mecanum controller"; "KISSL" | D4-S20 p.2 | Verified | Both exact quotes verbatim on p.2 (bluefield...txt). | None. |
| 84 | Buffalo: "no problems moving over the terrain... larger rollers would improve... rougher terrain," conditioned on custom traction-groove wheel | D4-S19 p.5 | Verified | Already confirmed (row 56). | None. |
| 85 | Oakland: self-framed as "an experiment," "it is believed... provided enough research effort" | D4-S18 p.16 | Verified | Exact verbatim quote, p.16 (row 3 above). | None. |
| 86 | No rough-terrain mecanum redesign found is a catalog product; contrasts with AndyMark/Rotacaster (no outdoor validation) | D4-S16 pp.12–16; D4-S23; D4-S24 | Verified | Consistent synthesis of already-verified rows. | None. |

---

## Recommended practice

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 87 | Lock rollers straight-line / dynamically-reorienting roller pivot | D4-S07 §4 | Verified | Confirmed verbatim (diegel...txt). | None. |
| 88 | Tune fork stiffness to 200–250 N/mm (fillet 2–3mm); roller curvature 3rd–7th | D4-S11 §3–4 | Verified | Confirmed verbatim (row 34). | None. |
| 89 | Flower-shaped calibration trajectories, 82.14% improvement on one 3-wheel platform | D4-S12 Abstract | Verified | Confirmed (row 42-43). | None. |
| 90 | Approach rigid obstacles at ~45° outdoors | D4-S16 p.13 | Verified | "The best results for obstacle climbing were obtained when the vehicle approach the obstacle at an angle of 45°" (row 49). | None. |
| 91 | Check whether cited rough-terrain validation regime (e.g. <40%-of-diameter obstacles) actually covers your case | D4-S16 pp.12–13 | Verified | Consistent with the 7.5%/40%-of-diameter figures already confirmed. | None. |

---

## Key numbers

| # | Quantity | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 92 | Min. wheel count 3 for 3-DOF holonomic | D4-S09 ch.13 p.515 | Verified | Confirmed (row 16). | None. |
| 93 | Mecanum γ typically ±45° | D4-S09 ch.13 pp.515–516 | Verified | Confirmed (row 16/22). | None. |
| 94 | Odometry calibration improvement 82.14% | D4-S12 Abstract | Verified | Confirmed (row 42-43). | None. |
| 95 | Optimal fork spring stiffness 200–250 N/mm | D4-S11 §4.1 | Verified | Confirmed (row 34). | None. |
| 96 | Optimal fillet radius 2–3 mm | D4-S11 §4.2, Table 2 | Verified | Confirmed (row 34). | None. |
| 97 | US Navy ODV wheel 39in dia., 3in obstacle = 7.5% | D4-S16 p.12 | Verified | Confirmed (row 48). | None. |
| 98 | EADS Astrium obstacle-failure height ~40% of wheel diameter | D4-S16 p.13 | Verified | Confirmed (row 49). | None. |
| 99 | Mecanum 92.17% vs. Solarbotics 90.90% on grass, 36in path | D4-S13 Table I–II | Verified | Confirmed (row 51). | None. |
| 100 | AndyMark load rating 500 lb/wheel | D4-S23 | Verified | Confirmed (row 64). | None. |
| 101 | AndyMark 12 rollers, 80A TPU | D4-S23 | Verified | Confirmed (row 64). | None. |
| 102 | AndyMark set-of-4 price $520.00 | D4-S23 | Verified | Confirmed (row 64). | None. |
| 103 | UBC Avalanche 4-wheel+gearbox cost $1,108 | D4-S21 | Verified | Confirmed (row 66). | None. |
| 104 | Buffalo battery-life gain, stated two inconsistent ways (150% longer / factor of 1.5) | D4-S19 p.5 ("150% longer"); p.13 ("factor of 1.5") | Verified | Both quotes verbatim-confirmed at the cited pages, and the README's own observation that they are mathematically different multiples ("150% longer" implies ×2.5 total vs. "factor of 1.5" implies ×1.5) is a correct, well-caught internal inconsistency in the primary source. | None — this is a genuinely good catch by the researcher. |
| 105 | Clearpath Ridgeback: 1.1 m/s / 100 kg / 18 mm | D4-S22a | Verified | Confirmed (row 70). | None. |
| 106 | mecanum_drive_controller first release 4.17.0 (2024-12-07) | D4-S26a | Verified | Confirmed (row 75). | None. |
| 107 | Humble backport 2.43.0 (2025-03-17) | D4-S26b | Verified | Confirmed (row 75). | None. |
| 108 | Torus-vs-exact-surface coincide only at δ=±π/2 | D4-S31 Theorem 1 | Verified | Confirmed (row 30). | None. |
| 109 | 4-wheel/3-wheel power utilization: 50%/71% vs 47%/68% | D4-S34 Introduction | Verified | "up to 50%, when translating along the axis of a wheel, and to 71%, when translating 45 deg..."; "up to 47%... and up to 68%, when translating 30 deg..." (galati...txt l.35-38) — exact match. | None. |
| 110 | Omnibot current, straight: 5.4 A concrete / 6.5 A asphalt | D4-S34 "Experimental results" | Verified | Confirmed (Summary row 7b). | None. |
| 111 | Omnibot current, sideways: 10.4 A / 16.6 A (peak 25A) | D4-S34 "Experimental results" | Verified | Confirmed (Summary row 7b). | None. |
| 112 | Omnibot vibration PSD: −49.29 / −36.54 dB/Hz | D4-S34 "Experimental results" | Verified | Confirmed (Summary row 7b). | None. |
| 113 | Omnibot wheel-radius variation 0.1480–0.1524 m, 12-roller wheel, "periodic every 30°" | D4-S34 "Rolling radius," Table 3 | Verified | This row's own phrasing ("periodic every 30°") is correct, unlike the Findings §3 bullet's "half-roller-pitch" phrasing for the same fact (see row 39). | None (this specific row is fine). |
| 114 | Omnibot friction: µ=0.5 concrete, µ=1.0 asphalt | D4-S34 "Wheel slipping" | Verified | Confirmed (row 40). | None. |
| 115 | Halt-logic fix PR #2326 on rolling 6.9.0 (2026-08-12), absent from Humble changelog as of 65db736 | D4-S36 | Partly supported | The "absent from the Humble changelog as of commit 65db736" half is independently verifiable and true (checked the full 82-line file; no match). The specific identity of the rolling-side fix (PR #2326, release 6.9.0, date 2026-08-12) is not supported by D4-S36, which contains no rolling-branch content at all. | Same fix as row 81 — cite an actual rolling-branch changelog source for the rolling-side half of this row. |

---

## How it is tested

| # | Test | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 116 | Vertical-vibration bench + RecurDyn, vs. roller curvature/fork stiffness | D4-S11 §3–4 | Verified | Confirmed (row 34). | None. |
| 117 | Flower-shaped calibration trajectory, 82.14% average improvement | D4-S12 Abstract | Verified | Confirmed (rows 42-43). | None. |
| 118 | Fixed-distance accuracy/deviation across 4 terrains, 20 runs/wheel type | D4-S13 §III, §IV.B, Tables I–II | Verified | Confirmed (row 51); formula and 99.63%/82.47% figures both confirmed in source. | None. |
| 119 | Obstacle-crossing vs. height/approach angle (secondary citation of Navy/Astrium tests) | D4-S16 pp.12–13 | Verified | Confirmed (rows 48-49). | None. |
| 120 | ASOC "omnidirectional mobility index" (RMS error, degrees): <0.1° w/o compliant suspension, 1.6° with | D4-S15 §4.1/§4.3, Table 2 | Verified | "it is less than 0.1 degrees in the case of the robot without the compliant suspension, and 1.6 degrees with" — exact match (ishigami...txt l.423-426). | None. |
| 121 | Same-robot concrete-vs-asphalt current/vibration PSD logging | D4-S34 "Experimental results" | Verified | Confirmed (Summary row 7b). | None. |

---

## Common mistakes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 122 | Omnidirectional ≠ holonomic; swerve is δm=1 type (1,2), not "omnimobile" (3,0) | D4-S04 p.742 | Partly supported | (3,0)/"omnimobile" naming verbatim-confirmed; type (1,2)'s existence and parameters verbatim-confirmed. The explicit label "swerve chassis" for type (1,2) is again (as in row 17) the README's own mapping, not the source's own terminology. | Same correction as row 17. |
| 123 | Roller rotation not observable from drive-wheel encoder | D4-S08 pp.50–52; D4-S09 ch.13 p.516 | Verified | Confirmed (row 19). | None. |
| 124 | Catalog wheel ≠ Buffalo's custom traction wheel; "COTS wheel rollers... continuous... meant for smooth, indoor surfaces" | D4-S19 p.5 | Verified | Exact quote match, p.5 (row 56). | None. |
| 125 | Rotacaster "All Terrain" claim is about a hand-truck caster wheel, not a drive wheel | D4-S24 | Verified | Confirmed (row 52). | None. |
| 126 | Humble lacked mecanum_drive_controller until March 2025, ~3 yrs after 2022 release | D4-S26a; D4-S26b | Verified | Confirmed (row 75). | None. |
| 127 | Torus roller shape is only approximate (exact only for 90° omni) | D4-S31 Theorem 1 | Verified | Confirmed (rows 29-30). | None. |

---

## Disagreements between sources

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 128 | Does mecanum perform acceptably on grass? Gugumuck (similar to rubber-tread) vs. Bluefield (had problems) vs. general outdoor-rejection literature | D4-S13; D4-S20 p.2; D4-S14; D4-S16 | Verified | All four underlying quotes independently verified above (rows 51, 83, 46, 48-50); the disagreement is accurately characterized, and the hedge about Gugumuck being hobby-scale/lowest-evidence is itself accurate (it's the only D-level source among the four). | None. |
| 129 | Is a custom traction-modified wheel a viable fix, or is the mechanism fundamentally limited? Buffalo (positive) vs. IIT/MIT/TARDEC + Calgary thesis (abandon rollers) | D4-S19; D4-S14; D4-S16 pp.12–16 | Verified | All underlying quotes independently verified (rows 56, 46, 48-50); correctly notes no head-to-head test exists between the two positions. | None. |

---

## Open questions (cited)

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 130 | Galati 2022 narrows (doesn't close) the outdoor-AGV-reliability gap — still "warehouses," not unpaved ground | D4-S34 | Verified | "intended to operate in warehouses" confirmed (row 61). | None. |
| 131 | Humble mecanum controller "narrowed, not fully resolved" by D4-S36 showing a missing safety fix + 2 feature additions vs. rolling | D4-S36 | **Not supported** | Same issue as row 81/115: D4-S36 is Humble-only; it cannot by itself show what exists on "rolling." The "missing... vs. rolling" comparison is not traceable to any file in `sources/`. | Same correction as row 81. |
| 132 | Hendzel & Rykała 2017 (second Lagrangian dynamics paper) located but inaccessible (CAPTCHA wall); not cited as a finding | — (bibliographic note only, not a finding) | Verified | Appropriately and explicitly not used as a finding; this is a transparent "could not access" note rather than a sourced claim, consistent with how D4-S02/S03/S05/S06 are handled. | None. |

---

## Overall notes for the Correct step

The **two highest-priority corrections** are:

1. **Finding §8, "dedicated motor controller per wheel" claim** (row 69) — actively contradicted by its own cited Buffalo source (2 motors share each controller channel pair), and the other 3 citations (Oakland §5.1, UBC p.4, Lynch & Park Fig.13.2) do not support it either. This should be removed or rewritten, not merely softened.
2. **The three "D4-S36 shows what's missing vs. rolling" claims** (rows 81, 115, 131, all stemming from the same root cause) — D4-S36 is a Humble-only changelog file; no rolling-branch `mecanum_drive_controller` changelog was ever downloaded to `sources/`, so the specific rolling-side PR numbers/versions/dates cannot be verified from any cited source. Either fetch and cite the actual rolling changelog, or drop the rolling-specific detail and keep only the "absent from Humble" half (which is independently verifiable).

Secondary corrections: the Rotacaster roller-durometer figures (row 27/65), the Ridgeback "35,840 pulses per revolution" figure (row 70), the Ilon-patent "assignee: AB Mecanum" detail (row 8), and the "half-roller-pitch" vs. "full-pitch" mischaracterization of Galati's 30° wheel-radius cycle (row 39) are each small, locally-containable fixes.

---

## Corrections applied (2026-10-06)

All findings marked **Not supported** and **Partly supported** above, and every item in SOURCE_AUDIT.md's "Needs correction" list, were applied to `README.md` (and `SCOPE.md` where noted). No finding remains marked Not supported or Partly supported, and no source remains failing, after these edits — `README.md`'s Status is now **Verified**.

**Claims — removed (Not supported, no replacement source available):**
- Row 27: the Rotacaster roller-count/durometer figures for the 125mm/35mm/50mm wheels (Findings §2) — deleted outright; neither downloaded Rotacaster file contains these figures and no replacement source was found in `sources/`.

**Claims — rewritten to state exactly what the source supports:**
- Row 3 (Summary): dropped the unsupported "D4-S18, p. 2" citation from the IGVC split-outcome bullet; kept "D4-S18, p. 16," which is verbatim-supported.
- Row 8 (Foundational references, D4-S01): removed the unsupported "(assignee: AB Mecanum)" parenthetical from both the Foundational-references entry and the Sources-table citation; the patent text itself names only inventor Bengt Erland Ilon. Added, properly sourced, the fact that Lynch & Park's own footnote (not the patent) states Ilon worked for "the Swedish company Mecanum AB" [D4-S09, ch. 13, p. 514, fn. 1].
- Row 17 / Row 122 (Findings §1 and Common mistakes): the mapping of Campion's type-(1,2) class onto "swerve chassis" is now explicitly flagged in both places as this research's own technical inference — the paper itself never uses the word "swerve."
- Row 19 (Findings §1): softened the "no sensor to confirm it" framing to state plainly that neither Siegwart nor Lynch & Park says this in so many words — it is this research's own reading of why the two texts' driven/free-sliding split matters for odometry.
- Row 22 (Findings §2): the claim that the split-roller design "is the lineage most later 'improved' mecanum designs build on" is now flagged as this research's own editorial framing, not a claim D4-S07 (2002) makes about literature that postdates it.
- Row 39 (Findings §3): "half-roller-pitch (30°) cycle" corrected to "one full roller-pitch (30°) cycle" — 30° is a full pitch on a 12-roller wheel (360°/12), matching Table 3's own 0°→15°→30° span.
- Row 65 (Findings §8): rewrote the Rotacaster diameter-list bullet to state plainly that the two downloaded snapshots disagree ("125mm," 2018 snapshot vs. "127mm," 2026 snapshot) and removed the unsupported per-size durometer figures from this bullet as well.
- Row 69 (Findings §8): rewrote the "dedicated motor controller per wheel... consistently documented pattern" claim, which its own cited Buffalo source contradicted. Replaced with an accurate statement: Buffalo's controllers each drive two motors (shared across a wheel pair, per §2.6), Oakland centralizes all four wheels' PI loops on one PIC processor (§5.1), and UBC's cost table lists only "Encoder: 2" for 4 wheels (p. 12) — i.e. no one-controller/one-encoder-per-wheel pattern is actually documented.
- Row 70 (Findings §9): removed the unsupported "(separately reported elsewhere as 35,840 pulses per revolution)" parenthetical from the Clearpath Ridgeback bullet; no cited file contains this figure.
- Row 76 (Findings §10): softened the `vy_std` Omni-only claim — `vy_max`'s doc text explicitly says "if using 'Omni' motion model," but `vy_std`'s own doc text does not; the bullet now says this is presumed by the same code-level `isHolonomic()` gating pattern (D4-S28), not independently confirmed for `vy_std` itself.
- Rows 81 / 115 / 131 (Findings §10, Key numbers, Open questions — same root cause): removed every rolling-branch-specific detail (PR numbers, release versions, dates) that is not traceable to any downloaded source. All three locations now state only the verifiable half: these three items (set_odometry service, velocity-limiting feature, halt-logic safety fix) are absent **by name from the Humble changelog, D4-S36**, with no claim about what exists on `rolling` since no rolling-branch changelog for this package was ever downloaded.

**Sources — fixed (SOURCE_AUDIT.md's "Needs correction" list):**
1. D4-S04 (Campion et al.): Sources-table Link corrected from the English-original URL (which was never the file actually downloaded) to the Russian-translation DOI (`10.20537/nd1104002`), matching the file on disk.
2. D4-S33: co-author's name corrected "Saypulaev" → "Saipulaev" throughout `README.md` (Sources table, two Findings bullets, one Open-questions bullet) and `SCOPE.md`'s gap-check note, matching the paper's own author line. The source file itself was renamed `sources/adamov_saypulaev_2020_mecanum_dynamics_slippage.pdf` → `sources/adamov_saipulaev_2020_mecanum_dynamics_slippage.pdf`, and the Sources-table File column updated to match.
3. D4-S36 / rolling-branch specifics: see rows 81/115/131 above — resolved by removing the unsourced rolling-branch claims rather than by downloading a new source (no rolling-branch `mecanum_drive_controller` changelog was added to `sources/` in this correction pass).
4. D4-S31 (Gfrerrer) foundational-table inconsistency: removed the D4-S31 row from README's "## Foundational references" table (it remains fully cited in Findings §2) and replaced it with a short explanatory note, reconciling README.md with SCOPE.md's own statement that Gfrerrer was "added as a supporting reference rather than retroactively renumbering the Foundational References table."
5. D4-S15 (Ishigami et al. 2012): upgraded from Level C to Level A. Independently confirmed via Crossref (2026-10-06) that this exact paper has a peer-reviewed Springer proceedings version — "Field and Service Robotics," Springer Tracts in Advanced Robotics vol. 92, 2014, pp. 599–611, doi:`10.1007/978-3-642-40686-7_40` — per STANDARDS §2's rule that a sponsor-archive copy is Level A once a peer-reviewed venue is confirmed and cited. The Sources-table citation now cites that Springer venue; the DTIC/Wayback PDF already on file is kept as the accessible copy.
6. D4-S22a (Clearpath datasheet): added a one-line disclosure to the Sources table that this file is hosted via a third-party reseller mirror (mybotshop.de), not Clearpath's own domain — consistent with how D4-S07/D4-S24 already disclose their own mirror-hosting.

**Mechanical checks (re-run after all edits):** `file` on every one of the 36 files in `sources/` matches its extension (no mismatches); every file in `sources/` is referenced in the README Sources table and every referenced path resolves to a file on disk (zero orphans either direction); every `D4-Snn` ID cited anywhere in `README.md` exists in the Sources table (zero missing); the Sources-table ID sequence `D4-S01`–`D4-S37` (with `S22a/S22b` and `S26a/S26b` sub-splits, 39 rows total) has no gaps and no duplicates.

**Not changed:** the body of this file (VERIFICATION.md) and of SOURCE_AUDIT.md above this section are left as the original reviewers' records and were not edited to match the corrected README — only this log was appended.
