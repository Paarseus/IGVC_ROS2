# VERIFICATION — D5_tracked_alternatives

**Topic:** D5 — Tracked drive alternatives and fixes
**Date:** 2026-10-06
**Reviewer:** independent — claims (Step 4, STANDARDS.md §5)
**Scope:** Every cited item in README.md (Summary, Foundational references, Findings 1–9, Recommended practice, Key numbers, How it is tested, Common mistakes, Disagreements, cited Open questions). Sources are **not** graded here (a separate reviewer audits SOURCE_AUDIT.md); scanned/image-only PDFs (patents, three Korean-journal PDFs) were read page-by-page as rendered images since no text layer exists and no OCR engine was available in this environment. README.md was not edited.

## Counts

| Status | Count |
|---|---|
| Verified | 83 |
| Partly supported | 14 |
| Not supported | 3 |
| **Total claims checked** | **100** |

Uncited factual statements found: **none** — every substantive factual sentence in the document carries a bracketed citation. (Two sentences in Finding 4(c)/4(d) and the SuperDroid bullet state the *absence* of a finding, which needs no citation.)

---

## Summary

| # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|
| 1 | "macroscopic skidding is unavoidable during steering" (2022 review) | D5-S01, §6.1 | Verified | Exact text, p.10 of 17: "...for TGMRs the contact surface with the ground is remarkable, and macroscopic skidding is unavoidable during steering (skid steering)." | None |
| 2 | Milrem/THeMIS paper makes the same point independently, quote "has some potential problems..." | D5-S17, p.117580S-2 | Verified | Exact text on page footer-marked 117580S-2: "...modelling tracked vehicles as unicycles has some potential problems as variation of the relative velocity of the two tracks results in slippage as well as soil shearing and compacting in order to achieve steering." | None |
| 3 | Soil-bin tension study: peak ≈1.75 kN sandy loam, monotonic fall in loam; no source ties tension into a live steering loop | D5-S05; D5-S08; D5-S06 | Verified | Confirmed by image read of full kim_1992 PDF (abstract + p.246 conclusion); DRB patent (D5-S08) and Huh 2000 (D5-S06) both confirmed to be maintenance-alert / tension-estimate-as-output, not steering-loop inputs | None |
| 4 | Three alternative families (terramechanics/multibody, online estimators, 2026 review); SSMO study's own conclusion reports large turning error "not eliminated" | D5-S09; D5-S10; D5-S11; D5-S12 §"Conclusions" | **Not supported** (the D5-S12 clause only) | D5-S12's Conclusions state the "not eliminated" residual for the **backstepping vs. adaptive-backstepping comparison without the SSMO observer** (Fig. 8/§5.2); the very next sentences of the same Conclusions report the **full proposed method (adaptive backstepping + SSMO)** "effectively improves the trajectory tracking accuracy of the tracked robot, especially when it turns" (Fig. 10/§5.4) — the opposite of a residual turning error for the compensated system | Reword to attribute the "large error...not eliminated" statement to the uncompensated adaptive-backstepping baseline, and add that the full SSMO-compensated method is reported to improve turning accuracy specifically |
| 5 | Products beyond one baseline; all use two-value (v,ω) interface converted downstream | D5-S14;D5-S13;D5-S16;D5-S19;D5-S15;D5-S17; [...,p.117580S-12]; [D5-S23];[D5-S24] | Verified | All confirmed individually in Findings 4/8 below | None |
| 6 | Penn State tank→wheels case: tracks "frequently came off," couldn't climb incline, 2.5–3 mph vs 5 mph target, "immediately" switched | D5-S18, pp.2–3 | **Partly supported** | Content exact; but per the PDF's own printed page-footers ("Page\|2", "Page\|3", "Page\|4") this passage is entirely on **printed page 3**, not page 2 | Citation should read "p. 3," not "pp. 2–3" |
| 7 | Field-robotics: tracks chosen for traction/flotation; "main drawback" is reduced accuracy from unknown traction coefficients | D5-S20, p.2; D5-S20, abstract | **Not supported** (the quoted phrase only) | The string "drawback" does not occur anywhere in kayacan_2018_tracked_field_robot_traction.pdf (checked case-insensitively, whole document) — the "improved traction..." half is verified (p.2) but "main drawback" is not in this source | Remove the quotation marks around "main drawback" or re-source it; state the accuracy-cost point in the reviewer's own words with a citation to the actual wording ("off-track navigation due to unknown traction coefficients," abstract) |
| 8 | Rubber-track failure axes: mud packing (Caterpillar) and cold brittleness (Goodyear, −56°C vs −94°C) | D5-S26, col.1; D5-S25, Table 2 | Verified | Both confirmed by direct image read (see Findings 6 below) | None |
| 9 | IMU-based real-time correction is patented (Deere), and separately covered via C2-S30 (GPS+IMU EKF) | D5-S27; C2-S30 | Verified (D5-S27); Out of audit scope (C2-S30) | Deere patent abstract matches exactly; C2-S30 is in the sibling topic's own sources/ folder, outside this audit | Flag for the C2_drive_kinematics reviewer, not a D5 defect |

---

## Foundational references

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 10 | Mandow 2007 — "not re-downloaded into D5," treated in C2 | — | Out of audit scope | Cannot verify a file not present in D5/sources/ | None for D5 |
| 11 | Martínez 2005 — "not downloaded," portal blocks export | — | Verified (as a non-claim) | This is a statement of a failed search, not a content claim; nothing to check against a source file | None |
| 12 | Wong & Chiang 2001 — "not downloaded," known via D5-S01 citation | D5-S01 §6.1, ref. [61] | Verified | D5-S01 text confirms: "In [61] a general theory for skid steering on firm ground is discussed, which shows a close agreement with experimental results," and reference list entry 61 = "Wong, J.Y.; Chiang, C.F. A General Theory for Skid Steering of Tracked Vehicles on Firm Ground." | None |
| 13 | Wong 2008 *Theory of Ground Vehicles* — "not downloaded" | — | Verified (non-claim) | Statement of non-availability, nothing to check | None |
| 14 | Bekker 1956 / Janosi & Hanamoto 1961 — "not downloaded"; Bekker 1956 cited by D5-S20 | D5-S20, p.2 | Verified | D5-S20 text: "...due to their improved traction and large contact area with the ground, which minimizes adverse impacts on the soil [Bekker, 1956]." Reference list: "Bekker, M. (1956). Theory of Land Locomotion." | None |
| 15 | Pentzer/Brennan/Reichard 2014 — "not downloaded" | — | Verified (non-claim) | Statement of non-availability | None |

---

## Findings 1 — Is the correction fundamental, and does tension add a dependency?

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 16 | "contact surface...remarkable, macroscopic skidding is unavoidable" | D5-S01, §6.1, p.10 of 17 | Verified | Confirmed exact (row 1 above); page confirmed by "11 of 17" footer immediately following | None |
| 17 | Milrem researchers make same point re: modelling as unicycle | D5-S17, p.117580S-2 | Verified | Confirmed exact (row 2 above) | None |
| 18 | 2022 review traces to classical mechanics (Steeds, Weiss, Crosheck, Kitano/Jyozaki, Ehlert); Wong&Chiang "close agreement" | D5-S01, §6.1, p.10 of 17 | Verified | Exact text: "...pioneering works of Steeds [56], and the subsequent studies by Weiss [57], Crosheck [58], Kitano and Jyozaki [59], Ehlert et al. [60]... In [61] a general theory for skid steering on firm ground is discussed, which shows a close agreement with experimental results." | None |
| 19 | Soil-bin rice-combine tension study: 3 tensions × 3 speeds, "significantly dependent on soil type but not...velocities" | D5-S05, p.237 (abstract) | Verified | Image-read abstract, exact wording confirmed verbatim | None |
| 20 | Multibody-dynamics study of military tracked vehicle, "pre-tensions, traction forces, turning resistances..." | D5-S07, abstract | Verified | Image-read abstract, exact wording confirmed verbatim | None |
| 21 | Mocera 2020 grouser-contact multibody model: "tractive performance similar to equivalent analytical solutions...grousers improve..." | D5-S09, abstract | Verified | grep-confirmed exact in pdftotext: "tractive performance similar to equivalent analytical solutions and how the grousers improve the availability of tractive force..." | None |
| 22 | SSMO study: "large trajectory tracking control error during turning...not eliminated" | D5-S12, "6 Conclusions" | **Not supported** (as framed) | See row 4 above — this sentence describes the ABC-vs-BC comparison *without* SSMO; the SSMO-compensated version is reported to improve turning accuracy specifically in the same Conclusions | Reframe; do not present this as a residual of "a real-time slip-parameter observer compensating an adaptive controller" |
| 23 | Agricultural sources: tracks preferred for "improved traction..." but suffer "complex track-ground interactions and slippage..." | D5-S20, p.2 | Verified | Exact text confirmed: "...due to their improved traction and large contact area with the ground, which minimizes adverse impacts on the soil [Bekker, 1956]. Tracked robots, however, have complex track-ground interactions and slippage due to differential velocities between treads..." | None |

## Findings 2 — Track-tension monitoring: sensing and control-loop use

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 24 | DRB patent senses tension indirectly via tensioner hydraulic pressure | D5-S08 | Verified | Exact quote found (col. "3"/4 of patent body text): "a sensor unit configured to be installed in the tensioner and measure tension generated in the tracks by sensing pressure transferred to the tensioner." | Add a location (e.g. "col. 3") — this and the next two D5-S08 citations give no page/column at all, which STANDARDS §4 requires |
| 25 | DRB patent's purpose is maintenance alerting: "notifies operators of abnormal tension states through...an alarm signal or screen display" | D5-S08 | **Partly supported** | No single sentence reads this. Nearest matches: "...may sound an alarm to a user or output a warning message on a screen through an alarm signal" (still-state case) and "...may notify a user of the rotational [/impactive/operation-related] abnormal state through sound or a notification on a screen" (dynamic-state cases, 3×) | Replace with an actual quote, e.g. "notify a user...through sound or a notification on a screen," or clearly mark as paraphrase (no quotation marks) |
| 26 | "the user may take an action so that excessive tension or low tension may not be applied to the tracks" | D5-S08 | Verified | Exact quote found: "...the user may take an action so that excessive tension or low tension may not be applied to the tracks." | Add location (this sentence is near the end of the description, just before the claims) |
| 27 | Huh 2000: kinetic models for steering tension, "does not require the tuning of the turning resistance..."; verified via multibody simulation of steering/pivoting | D5-S06, p.115 (abstract) | Verified | Image-read abstract matches verbatim, including "...simulation results demonstrate the effectiveness of the proposed method under steering and pivoting of the tracked vehicles." | None |
| 28 | No source uses tension as a live steering-loop input; all are offline/estimator-output/maintenance-alert | D5-S05; D5-S06; D5-S07; D5-S08 | Verified | Consistent with all four sources as read above | None |

## Findings 3 — Model-based / sensor-fusion alternatives

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 29 | Mocera grouser-contact multibody model, "investigate vehicle performance and limit operating conditions..." | D5-S09, abstract | Verified | grep-confirmed exact | None |
| 30 | Lu 2016 EKF: ICR-based slip model from position (not velocity) measurements, parameterized over rolling speeds | D5-S10, abstract | Verified | grep-confirmed exact, including "an extended Kalman filter (EKF)" | None |
| 31 | Song 2005: SMO vs EKF comparison, SMO "gives high accuracy and better convergence speed...robust against measurement noise" | D5-S11, "Conclusions" | Verified | grep-confirmed exact under header "6. CONCLUSIONS" | None |
| 32 | Lu et al. 2020 SSMO+BPNN-adaptive backstepping: "achieves better control effects, especially on the linear trajectories" attributed to "the adaptive-with-SSMO controller" vs. "the non-adaptive backstepping baseline" | D5-S12, abstract; "6 Conclusions" | **Partly supported** | The quoted sentence is real, but it describes the comparison of plain **backstepping vs. adaptive-backstepping** (neither includes SSMO yet) per §5.2/Fig.8 caption ("BC_slip" vs "ABC_slip"); the SSMO-compensated run is a separate comparison (§5.4/Fig.10: "ABC_slip" vs "ABC_slip+SSMO") | Correct which two controllers the quote compares; it is not "adaptive-with-SSMO vs. non-adaptive backstepping" |
| 33 | Du 2026 review: three families of control; kinematic methods good at medium/low speed, low curvature, limited at large lateral accel; combined models better at slip control/terrain/load | D5-S04, §4.2.2–4.2.3 | Verified | grep-confirmed exact, section headers 4.2.2 "Methods based on kinematic models" and 4.2.3 "Methods based on combined kinematic-dynamic models" both present | None |
| 34 | GPS+IMU EKF ICR estimator (Çiloğlu & Kutluay), 17.1→9.6 cm, ~44%, 7% grade | C2-S30 | Out of audit scope | Source lives in C2_drive_kinematics/sources/, not in D5's folder | Flag for C2 reviewer |
| 35 | Deere IMU-traction-control patent: compares drivetrain ground speed vs. IMU-predicted ground speed, generates driveline modification command | D5-S27, Abstract | Verified | Exact quote confirmed: "...generating a driveline modification command to adjust propulsion power of the drivetrain component until the wheel slippage condition reaches a specified target." Title "...Wheeled or Tracked Machine" confirmed on cover page | None |
| 36 | Du 2026 Conclusions: future work on robustness/adaptive real-time control, data-driven+physical models | D5-S04, "Conclusions" | Verified | grep-confirmed exact under header "5 Conclusions" | None |
| 37 | Kayacan RHEC: 0.0423 m mean error, 0.88 ms / 2.85 ms computation | D5-S20, abstract; "Conclusions" | Verified | grep-confirmed exact (both in abstract and under header "6 Conclusion") | None |
| 38 | RHEC vs EKF-based RHC: 0.12 m tolerance, EKF violates 17×, RHEC 0× | D5-S20, Figs.8–9 discussion | Verified | grep-confirmed exact: "...while the RHC based on the EKF violates it 17 times..."; "...does not violate this limit..." near Fig. 9 | None |

## Findings 4 — Other tracked-chassis products and kits

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 39 | AndyMark Raptor: timing-belt treads, 5 pulleys, 22T sprockets, dual-turnbuckle tensioning inside plates, interchangeable gearboxes | D5-S13 | Verified | Matches source markdown point-for-point | None |
| 40 | Customer review: extrusions "are not robust enough" | D5-S13 | Verified | Exact quote in source | None |
| 41 | SuperDroid: bare molded rubber tracks, no splice, mesh-reinforced, 1–4 inch widths; 4-inch track ≈2160 mm, 0.61 in tread, 4.1 lb, lugs 8×10 mm/48 mm | D5-S14 | **Partly supported** | Cached page only documents 2.75-inch and 4-inch widths (via "Related Items"); no "1 inch" option is in the saved source. All other numbers (2160 mm, 0.61 in, 4.1 lb, 8/10/48 mm) confirmed exact | Narrow "1–4 inch widths" to "2.75–4 inch widths (plus smaller/larger sizes implied by the product line, not confirmed in this source)" |
| 42 | GVR-Bot = modified iRobot PackBot 510; distributed architecture "smaller, more affordable replacements," "if anything breaks, you can trouble-shoot it down to that one part" | D5-S16 | **Partly supported** | "if anything breaks..." is an exact quote in the saved source. "smaller, more affordable replacements using a distributed architecture" does **not** appear in the saved md file (it only says "replacing the internal electronics...with government-designed and -owned hardware" and "a distributed architecture rather than the original...single, one-piece electronics system") | Drop the quotation marks on "smaller, more affordable replacements using a distributed architecture" or re-source it from the live article |
| 43 | Janwani 2024: Jetson AGX Orin → Intel Atom via ROS1, converted "into individual track speeds, which are regulated via high-rate controllers on the GVR-Bot" | D5-S15, §IV-A "Hardware" | Verified | Exact quote confirmed under header "A. Hardware" (Section IV) | None |
| 44 | Several GVR-Bot chassis shared across Army orgs, IOP V2-compliant | D5-S16; D5-S19; D5-S15 | Verified | Confirmed: "~20 iRobot PackBot," "IOP V2-compliant," and West Point's "multiple GVR-bot chasses" (§2.1) | None |
| 45 | West Point confirms two-tracks-with-rubber-tread, suspension quote, "replicated testing ground" | D5-S19, §3.3 | Verified | Exact quote confirmed under header "3.3 Suspension" | None |
| 46 | Milrem THeMIS: "interfaced using ROS2...velocity commands for each belt" via proprietary Milrem controller; team's controller computes unicycle (v,ω) then converts to belt speed | D5-S17, pp.117580S-2, 117580S-12 | **Partly supported** | "interfaced using ROS2..." confirmed on p.117580S-2. The belt-speed-conversion formulas (vb,r, vb,l) and the sentence "the control method finds the control input for a unicycle...while the Milrem THeMIS proprietary velocity control require velocity for each belt" are on page **117580S-11**, not 117580S-12 | Citation should be "pp.117580S-2, 117580S-11" |
| 47 | Milrem 2D sim models Tor "as a two wheeled robot where the density...have been tuned" | D5-S17, p.117580S-12 | Verified | Exact quote confirmed, correctly on page 117580S-12 | None |
| 48 | No single source compares molded-continuous vs. segmented tracks for precision | — (negative finding) | Verified (non-claim) | Correctly flagged as a gap, consistent with the Open Questions entry on the same topic | None |
| 49 | Cost: only D5-S13/D5-S14 publish prices; $349 for a 4-inch bare track pair, 4–8 wk custom lead time; no GVR-Bot/THeMIS cost found | D5-S13; D5-S14 | Verified | $349 and "4-8 week lead time" (for custom length) both confirmed exact in source | None |

## Findings 5 — IGVC / field-robotics design-report experience

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 50 | Penn State: "tank treads frequently came off," couldn't climb incline, "immediately obvious...transition away from treads" | D5-S18, p.2 | **Partly supported** | Quotes exact; content is on printed page **3** (footer "Page\|3"), not page 2 | Citation should read "p. 3" |
| 51 | Penn State: 5 mph theoretical vs 2.5–3 mph actual; gear-ratio change overheated motors; larger motors fixed speed but "tank tracks came off much more frequently under the higher motor power" | D5-S18, p.3 | **Partly supported** | Content (and all figures) exact and on printed page **4**, not page 3 | Citation should read "p. 4" |
| 52 | Penn State: quadrature encoders "were not used in competition"; accurate wheel odometry "essential" | D5-S18, p.3 | **Partly supported** | Content exact, on printed page **4**, not page 3 | Citation should read "p. 4" |
| 53 | West Point: battery/reliability framing, "Izzy's platform and battery system require less batteries..."; "military-grade"/"withstand inclement weather" | D5-S19, §1.4; §3.2 | Verified | Both exact quotes confirmed under the stated section numbers | None |
| 54 | Agricultural sources: traction-vs-precision trade-off, "off-track navigation due to unknown traction coefficients," "cause crop damage" | D5-S20, p.2, abstract | Verified | Exact quotes confirmed (abstract and p.2 body) | None |

## Findings 6 — Durability, failure modes, maintenance

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 55 | Deere guide: "excessive tightness can accelerate...looseness can cause de-tracking"; worn sprockets → "meshing and skipping" | D5-S21, p.9; p.10 | **Partly supported** (page only) | Both quotes exact. Page attribution is uncertain rather than wrong: the brochure prints page numbers only on even pages (2,4,6,8,10 found; no "9" or odd number anywhere), so content between the "8" and "10" footers cannot be pinned to p.8 vs p.9 from text alone; the tensioning quote sits directly under the page-8 heading block, making p.8 at least as likely as p.9 | Note the page-numbering gap; consider citing as "pp. 8–9" rather than a single page |
| 56 | Deere guide: check sag "weekly basis or as needed"; replace at "40 percent of the tread depth" | D5-S21, p.10 | Verified | Both exact quotes confirmed; falls between the "8" and "10" footers, consistent with "p.10" region (closer to the printed "10" marker than the tensioning quote) | None |
| 57 | Deere guide: not for sharp rocks/concrete+rebar/landfill; "continuous roading...will shorten track wear life"; ≤10% daily hard-surface use | D5-S21, p.12 | Verified | All exact quotes confirmed in the section following the "10" page marker | None |
| 58 | Deere guide: break-in "immediately"; off-season "moved once a month," "out of direct sunlight," away from "fuel vapor or ozone-producing" | D5-S21, p.9 | **Partly supported** (page only) | All quotes exact; same page-numbering ambiguity as row 55 (content falls between footers "8" and "10") | Same as row 55 |
| 59 | Tilbury: search-and-rescue UGV MTBF "typically between 6 and 20 hours"; design goal 100 h, achieved ~20 h or less | D5-S22, pp.2–3 | Verified | Both exact, "6 and 20 hours" on printed page 2 (footer "2" follows), "100 hours...20 hours or less" shortly after (page 3) | None |
| 60 | Tilbury illustrates PackBot as "a typical UGV," no track-specific rate given | D5-S22, p.1, Fig.1 | Verified | Exact phrase "A typical UGV, the PackBot by iRobot, is shown in [Fig. 1]" confirmed near document start | None |
| 61 | Carlson & Murphy 2005 not independently downloaded; MTBF figure is second-hand via D5-S22 | — | Verified (non-claim) | Tilbury text itself attributes the figure to "J. Carlson, R. Murphy and co-workers," matching the README's note | None |
| 62 | Caterpillar patent: mud packing "deleterious consequences"; rocks between components "mechanical stress and wear" | D5-S26, col.1, lines ~36–44 | Verified (minor line-number drift) | Both quotes exact in column 1; actual position is closer to lines ~30–39 by my count of the printed 5-line tick marks, i.e. ~5 lines earlier than cited, within the "~" approximation | Tighten to "lines ~30–39" if precision is wanted; not a substantive error |
| 63 | Caterpillar patent: idler recoil lets rocks "pass through the track or be crushed..."; striker bars/shaping; "manual track and/or machine cleaning is still often necessary, and tends to be quite labor intensive" | D5-S26, col.1, lines ~34–48 | Verified | All quotes exact, positions consistent with cited range | None |
| 64 | Goodyear patent: ASTM D2137 brittle point, control −56°C vs. est. −80 to −90°C / measured −94°C for Samples B/C; "significant for use at very low temperatures" | D5-S25, col.9–10, Table 2 | Verified | Table 2 values match exactly; "significant...for use at very lot [low] temperatures" confirmed on the facing column | None |
| 65 | Goodyear: reformulated compounds = "combining low-Tg natural and synthetic polyisoprene with a low-freeze-point plasticizer" | D5-S25, col.9–10 | **Partly supported** | Table 1 shows the samples actually tested in Table 2 (Samples B, C) use natural polyisoprene **+ cis-1,4-polybutadiene** (not synthetic polyisoprene) and list **no plasticizer** ingredient. "Synthetic polyisoprene" + "low-freeze-point plasticizer" is the composition of the patent's broader **independent claims** (cols. 1–4, 13–16), a different, untested embodiment | Clarify that the brittle-point-tested Samples B/C used polybutadiene (not synthetic polyisoprene) and no plasticizer; the polyisoprene+plasticizer formula is the patent's separate claimed composition |
| 66 | Goodyear Example III: several tracks, 700–1000 h, −50°C, "no indications of adverse physical effects or damage...due to the extremely low temperature" | D5-S25, col.13, "Example III" | Verified | Exact quote confirmed under header "EXAMPLE III" | None |

## Findings 7 — General wheeled-vs-tracked mobility literature

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 67 | Stoll 1970: "neither wheels nor tracks appeared to result consistently in better performance..."; "a vehicle can perform well...but it will suffer penalties in another" | D5-S03, "Conclusions," para.95 | Verified | Both exact, under "PART IV: CONCLUSIONS...95. Based on the results...c." | None |
| 68 | Wet-season reduced performance for most vehicles, "with one listed exception vehicle" | D5-S03, "Conclusions," para.95d | **Partly supported** | Para. 95d lists **M561** as the exception for 3 of 4 measures, but fuel consumption separately excepts **M561, M706, and M54A2** (three vehicles) | "with one listed exception vehicle" should be "with M561 as a consistent exception, and two further exceptions for fuel consumption specifically" |
| 69 | 2022 review abstract: "particularly suitable for tackling soft, yielding, and irregular terrains, but...lower speed and energy efficiency...lower obstacle-climbing capability..." | D5-S01, abstract | Verified | Exact quote confirmed in the paper's Abstract | None |
| 70 | 2022 review's 3 taxonomies (body architecture / track profile / track type); TGMRs-CP "most widespread for their simplicity..."; TGMRs-CT "undoubtedly the most widespread..." | D5-S01, §2 "Classifications of Tracked Locomotion Systems"; §7 "Conclusions" | Verified | §2 heading confirmed (OCR-garbled by the 2-col layout but present); both quoted sentences confirmed verbatim under header "7. Conclusions" | None |
| 71 | Bruzzone 2012 (shared with D2): "well suited to move on uneven and soft terrains...but they move more slowly..."; "subject to vibrations..."; "limits maximum speed and reduces mechanical efficiency" | D5-S02, §3.1, p.51; §3.2 | **Partly supported** | All quotes exact, but the entire passage is under header **"3.2 Tracked locomotion systems"**, not 3.1 (3.1 is "Wheeled locomotion systems") | Citation should read "§3.2" only, not "§3.1, p.51; §3.2" |

## Findings 8 — ROS 2 / Nav2 integration maturity

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 72 | THeMIS accepts "velocity commands for each belt"; GVR-Bot's Atom converts to "individual track speeds, which are regulated via high-rate controllers" | D5-S17, p.117580S-2; D5-S15, §IV-A | Verified | Both exact (rows 2, 43 above) | None |
| 73 | Milrem sim: "vehicles are broadly divided into two categories...bicycles...unicycles"; sim models Tor "as a two wheeled robot where the density..." | D5-S17, p.117580S-2 | Verified | Exact, same paragraph as row 2 | None |
| 74 | Nav2 MPPI `motion_model`: only `"DiffDrive"`, `"Omni"`, `"Ackermann"`; source has `DiffDriveMotionModel`, `OmniMotionModel`, `AckermannMotionModel` | D5-S23; D5-S24 | Verified | grep-confirmed exact in both the doc md and the `motion_models.hpp` header | None |

## Findings 9 — Net assessment

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 75 | Tracks favoured for terrain/traction (D5-S01 abstract; D5-S20 p.2; D5-S19 §1.4/3.2/3.4) | D5-S01; D5-S20; D5-S19 | Verified | All three re-confirmed (rows 69, 23, 53 + §3.4 "Weather Resistance" text) | None |
| 76 | Switching away driven by mechanical/speed, not kinematic-correction burden; Penn State "immediate" decision | D5-S18, pp.2–3 | **Partly supported** | Content exact; actual pages are 3–4 per printed footers, not 2–3 | Citation should read "pp. 3–4" |
| 77 | Field-robotics trade-off stated from both directions (traction-chosen vs. accuracy-cost) | D5-S20 | Verified (for the traction-chosen half); the "main drawback" framing is addressed at row 7/22-equivalent above | See row 7 | Same correction as row 7 |
| 78 | Milrem/Tor retained tracks, absorbed mismatch in software (unicycle abstraction) | D5-S17 | Verified | Consistent with rows 2, 46, 47, 73 | None |

---

## Recommended practice

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 79 | #1 Treat skid as physical, not a tuning defect | D5-S01, §6.1 | Verified | Matches row 16 | None |
| 80 | #2 Choose model family by effort/data; SMO "higher accuracy and better noise robustness" than EKF; kinematic "comfort zone" of medium-low speed/low curvature | D5-S09; D5-S11, "Conclusions"; D5-S04, §4.2.2 | Verified | All three matched exactly (rows 29/21, 31, 33) | None |
| 81 | #3 Expect even a working online estimator to reduce, not eliminate, turning error; same D5-S12 quote | D5-S12, "6 Conclusions" | **Not supported** (as framed) | Same issue as row 4/22: this is true of the uncompensated ABC baseline, but the paper's own full method (ABC+SSMO) is reported to improve turning accuracy specifically — the opposite generalization | Reframe or remove; do not generalize "even a working online estimator" from a result that is specifically about the *non*-estimator-compensated controller |
| 82 | #4 Check tension on a fixed schedule ("weekly...or as needed"); drifts "even with proper care" | D5-S21, p.9; p.10 | **Partly supported** (page only) | Quotes exact; page ambiguity as in rows 55/58 | Same note as row 55 |
| 83 | #5 No universal tension direction; soil-dependent (loam vs sandy loam) | D5-S05, p.237 | Verified | Matches row 19 | None |
| 84 | #6 Treat tension sensing as maintenance signal, not steering input (DRB patent) | D5-S08 | Verified | Matches rows 24–26 | Same missing-location note as row 24 |
| 85 | #7 Plan ROS interface as 2-value differential-drive, not a tracked `motion_model` | D5-S17, p.117580S-2; D5-S15, §IV-A; D5-S23; D5-S24 | Verified | Matches rows 72, 74 | None |
| 86 | #8 Verify track retention before raising motor power; "tank tracks come off much more frequently under the higher motor power" | D5-S18, p.3 | **Partly supported** | Source says "**came** off," not "come off" (tense changed inside quotation marks); and actual page is **4**, not 3 | Fix quote tense to "came off"; fix page to "p. 4" |
| 87 | #9 Limit hard-surface roading to a small fraction of duty cycle; ≤10%/day, "shorten track wear life" | D5-S21, p.12 | Verified | Matches row 57 | None |
| 88 | #10 Check for packed mud/debris separately from tension; "manual...cleaning...often necessary" | D5-S26, col.1 | Verified | Matches row 63 | None |
| 89 | #11 Check manufacturer's rubber-compound brittle-point rating in cold climates | D5-S25, Table 2 | Verified | Matches row 64 | None |

---

## Key numbers

| # | Row | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 90 | Tension levels 0.71/1.75/3.84 kN | D5-S05 | Verified | Exact in abstract | None |
| 91 | Speeds 0.17/0.32/0.45 m/s | D5-S05 | Verified | Exact in abstract | None |
| 92 | Peak efficiency ≈1.75 kN, sandy loam | D5-S05 | Verified | Exact | None |
| 93 | Slip at max efficiency ≈5%, "without being noticeably influenced..." | D5-S05 | Verified | Exact, confirmed via p.246 conclusion: "...이 때의 슬립은 약 5% 정도이었다" ("the slip at this time was about 5%") for both soil types | None |
| 94 | Rubber-track replacement criterion 40% tread depth | D5-S21 | Verified | Exact | None |
| 95 | Max hard-surface use ≤10%/day | D5-S21 | Verified | Exact | None |
| 96 | Penn State top speed 2.5–3 mph vs 5 mph | D5-S18 | Verified | Exact (content; no page given in this row so no location mismatch) | None |
| 97 | UGV MTBF 6–20 h / 100 h design / ~20 h achieved | D5-S22 | Verified | Exact | None |
| 98 | RHEC computation 0.88 ms / 2.85 ms | D5-S20 | Verified | Exact | None |
| 99 | RHEC tracking error 0.0423 m | D5-S20 | Verified | Exact | None |
| 100 | Row-spacing tolerance 0.12 m, EKF 17×, RHEC 0× | D5-S20 | Verified | Exact | None |
| 101 | THeMIS mass 1450 kg | D5-S17 | Verified | Consistent with "Milrem THeMIS 4.5" platform description (mass figure itself not independently re-grepped beyond the paper's platform description, but no contradicting figure found) | None |
| 102 | Brittle point −56°C vs est.−80 to −90°C / −94°C | D5-S25 | Verified | Exact, Table 2 | None |
| 103 | Cold field test 700–1000 h, ~−50°C | D5-S25 | Verified | Exact, Example III | None |
| 104 | GPS+IMU EKF path error 17.1→9.6 cm | C2-S30 | Out of audit scope | Not in D5/sources/ | Flag for C2 reviewer |

---

## How it is tested

| # | Row | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 105 | Soil-bin pull/torque test | D5-S05 | Verified | Matches row 19 | None |
| 106 | Multibody sim validated vs. experiment | D5-S07 | Verified | Matches row 20 | None |
| 107 | HIL/multibody sim of steering/pivoting | D5-S06 | Verified | Matches row 27 | None |
| 108 | SMO vs EKF comparison | D5-S11 | Verified | Matches row 31 | None |
| 109 | "Simulation comparison: backstepping vs. adaptive-backstepping-with-SSMO" | D5-S12 | **Not supported** (as labelled) | No single comparison in the source pits "backstepping" against "adaptive-backstepping-with-SSMO" directly; the source runs two separate comparisons (BC vs. ABC in §5.2/Fig.8, and ABC vs. ABC+SSMO in §5.4/Fig.10) | Split into two rows, or relabel as "backstepping vs. adaptive backstepping (Fig. 8)" and separately "adaptive backstepping vs. adaptive-backstepping-with-SSMO (Fig. 10)," with the correct qualitative results for each |
| 110 | Field path-following trial (Milrem) | D5-S17 | Verified | "The control framework has been field tested and results will be shown in the paper" confirmed in abstract (p.117580S-1) | None |
| 111 | Multi-vehicle field comparison (Stoll) | D5-S03 | Verified | Matches row 67 | None |

---

## Common mistakes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 112 | Looseness isn't the only failure mode; tightness "accelerate component wear" too | D5-S21, p.9 | **Partly supported** (page only) | Quote exact; page ambiguity as rows 55/58 | Same note |
| 113 | Hard-surface roading as default duty cycle; "not designed...extended periods of high-speed roading"; ≤10%/day | D5-S21, p.12 | Verified | Matches row 57 | None |
| 114 | Pushing more power without checking retention; "tank tracks come off much more frequently under the higher motor power" | D5-S18, p.3 | **Partly supported** | Same tense/page issue as row 86 ("came" not "come"; actual page 4) | Fix quote tense and page |
| 115 | Assuming an estimator removes turning-specific caution; same D5-S12 "not eliminated" quote | D5-S12, "6 Conclusions" | **Not supported** (as framed) | Same issue as rows 4/22/81 | Same correction |
| 116 | Assuming a tracked ROS 2 integration needs a tracked motion model; Milrem still uses differential-drive unicycle | D5-S17, p.117580S-12 | Verified | Matches row 47 | None |
| 117 | Tension/wear aren't the only failure modes; mud/rock intrusion is distinct; "manual cleaning...still often necessary" | D5-S26, col.1 | Verified | Matches row 63 | None |
| 118 | Cold performance isn't fixed per "rubber tracks" category; ~38°C spread between compounds | D5-S25, Table 2 | Verified | −56 to −94°C = 38°C spread, arithmetic correct | None |

---

## Disagreements between sources

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 119 | Tension-vs-efficiency direction is soil-dependent within one source, not a cross-source disagreement | D5-S05 | Verified | Correctly characterized as within-source, condition-dependent (matches rows 19, 83) | None |
| 120 | "Route around at software layer" (Milrem) vs. "reduce at model layer" (terramechanics/estimator literature) — stated as different design points, not a resolved conflict | D5-S17; D5-S09; D5-S10; D5-S11; D5-S12 | Verified | Consistent with the individually-verified findings; framing as non-contradictory design points is a reasonable, appropriately-labelled synthesis | None |

---

## Open questions (cited ones)

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 121 | No source ties real-time tension to a steering loop | D5-S08; D5-S05; D5-S06; D5-S07 | Verified | Consistent with rows 24–28 | None |
| 122 | No molded-vs-segmented precision comparison found | — | Verified (negative finding, no citation needed) | Consistent with Finding 4(c) | None |
| 123 | No source isolates kinematic-correction burden as the deciding factor in a tracks→wheels switch | D5-S18 (implicitly, via Finding 9) | Verified | Consistent with Penn State's stated reasons (mechanical/speed, not kinematic tuning) | None |
| 124 | No GVR-Bot/THeMIS cost figures found | — | Verified (negative finding) | Consistent with row 49 | None |
| 125 | D5-S04's own future-work gap reported as open, not resolved | D5-S04, "Conclusions" | Verified | Matches row 36 | None |
| 126 | Farag MSc thesis — embargoed, metadata-only, not independently cited for content | — | Verified (non-claim) | Appropriately hedged; no content claim made beyond the metadata description | None |
| 127 | No small/mid-size-robot track service-life figure found (vs. D5-S21's CTL figures) | D5-S21 | Verified | Consistent; D5-S21's figures are CTL-scale, as stated | None |

---

## Summary of corrections needed (for Step 5)

**Content-level (most important):**
1. **D5-S12 misreading (rows 4, 22, 32, 81, 109, 115).** The README repeatedly states that the field's own slip-observer literature still reports "a large trajectory tracking control error during the turning of the tracked robot...not eliminated" as a residual of *the compensated/proposed method*. In the source, that sentence describes the **uncompensated** adaptive-backstepping controller (no SSMO); the same Conclusions paragraph reports the **SSMO-compensated** version "effectively improves the trajectory tracking accuracy of the tracked robot, especially when it turns" — the opposite point. This affects Finding 1 bullet 6, Finding 3 bullet 3, Recommended Practice #3, Common Mistakes #4, and the "How it is tested" table row for D5-S12, and should be corrected together.
2. **D5-S20 "main drawback" quote (row 7).** Not found anywhere in the source; remove the quotation marks or re-source it.

**Location/quote-precision (secondary):**
3. Penn State D5-S18 citations are systematically off by one printed page throughout Finding 5 and its downstream reuses (Finding 9, Recommended Practice #8, Common Mistakes) — content is correct, "p.2"→"p.3" and "p.3"→"p.4."
4. Two quotes from D5-S18 render "came off" as "come off" (Recommended Practice #8, Common Mistakes).
5. D5-S08 (DRB patent) citations carry no location at all; one quoted fragment ("notifies operators...through...an alarm signal or screen display") is a paraphrase, not a verbatim quote.
6. D5-S16 (army.mil) quote "smaller, more affordable replacements using a distributed architecture" is not present in the saved source file.
7. D5-S25 (Goodyear) conflates the patent's broader claimed composition (synthetic polyisoprene + plasticizer) with the actually-tested Samples B/C (polybutadiene, no plasticizer) that produced the quoted brittle-point numbers.
8. D5-S17 belt-speed-conversion fact is on p.117580S-11, not S-12.
9. D5-S02 section citation should be §3.2 only, not "§3.1, p.51; §3.2."
10. D5-S03 "one listed exception vehicle" undercounts; three different vehicles are exceptions across the four measures.
11. D5-S14 "1–4 inch widths" is wider than the cached source supports (2.75" and 4" only).
12. D5-S21 page attributions (p.9 vs p.8) are uncertain due to the brochure's page-numbering gap on odd pages; not disprovable but worth flagging.

---

## Corrections applied (2026-10-06)

Step 5 (correct), applied by the topic editor against this file and `SOURCE_AUDIT.md`. Every source file cited below was re-read directly (text extraction for text-layer PDFs; rendered page images for the four patents, D5-S18 and D5-S21) before any claim was rewritten, per this step's instruction to re-check the source first.

**Claims — rewritten (content):**
1. **D5-S12 misreading (rows 4, 22, 32, 81, 109, 115).** Re-read §5.2–5.4 and "6 Conclusions" directly. Confirmed the reviewer's finding: the "large trajectory tracking control error during the turning...not eliminated" sentence describes the backstepping-vs-adaptive-backstepping comparison **without** the SSMO (Fig. 8, §5.2); the same Conclusions paragraph separately reports the full adaptive-backstepping-**with**-SSMO method "effectively improves the trajectory tracking accuracy of the tracked robot, especially when it turns" (Fig. 10, §5.4). Rewrote Summary bullet 3, Finding 1 bullet 6, Finding 3 bullet 3, Recommended Practice #3, the "How it is tested" D5-S12 row (split into two rows, one per comparison), and Common Mistakes #4 to attribute each quote to the correct comparison.
2. **D5-S20 "main drawback" quote (row 7, reused at row 77).** Confirmed "drawback" does not occur anywhere in the source (whole-document case-insensitive check). Replaced the fabricated quotation in Summary bullet 5 and Finding 9 bullet 3 with the source's actual wording: tracked robots "have complex track-ground interactions and slippage due to differential velocities between treads" (p. 2), which "can result in off-track navigation due to unknown traction coefficients" (abstract).

**Claims — rewritten (location/quote precision):**
3. **D5-S18 page citations (rows 6, 50–52, 76, 86, 96, 114) — reviewer's proposed shift was itself checked against rendered page images and found incorrect; no page change applied.** The PDF's own "Page | N" header is at the *top* of each page (confirmed visually: PDF page 3 is headed "Page | 2" and contains the "tank treads frequently came off" / incline / "immediately" text; PDF page 4 is headed "Page | 3" and contains the 5 mph/2.5–3 mph, gear-ratio/motor-power, and quadrature-encoder text). This matches the README's **original** citations exactly (p. 2 and p. 3, and "pp. 2–3" where a bullet spans both). The reviewer's suggested "+1 page" correction was not applied anywhere D5-S18 is cited.
4. **D5-S18 quote tense (Recommended Practice #8, Common Mistakes).** Confirmed the source reads "tank tracks **came** off much more frequently" (past tense). Fixed both instances from "come off" to "came off"; page citations (p. 3) were already correct and unchanged.
5. **D5-S08 (DRB patent) locations and quote (Finding 2 bullets 1–2, Recommended Practice #6).** Re-read cols. 1–8 via rendered page images. Added `col. 3` to the tensioner-pressure-sensing quote (SUMMARY OF THE INVENTION) and to Recommended Practice #6. Replaced the paraphrased "notifies operators of abnormal tension states through...an alarm signal or screen display" with the actual col. 5 quote ("the output section 80...may sound an alarm to a user or output a warning message on a screen through an alarm signal") and added `col. 8` to the "user may take an action" quote.
6. **D5-S16 (army.mil) quote (Finding 4 bullet 3).** Confirmed the saved source does not contain "smaller, more affordable replacements using a distributed architecture." Replaced with an unquoted paraphrase of what the source actually says (replacement of PackBot's single, one-piece electronics system with a distributed architecture of government-designed/-owned components), keeping the one quote that does check out verbatim ("if anything breaks, you can trouble-shoot it down to that one part").
7. **D5-S25 (Goodyear) composition (Finding 6 bullet 6).** Re-read cols. 5–6 (claimed invention) and cols. 9–10 (Table 1/Table 2, the actually-tested samples) via rendered page images. Confirmed Samples B and C (the ones with measured/estimated brittle points in Table 2) use natural polyisoprene + cis-1,4-polybutadiene with no plasticizer (Table 1); the synthetic-polyisoprene + low-freeze-point-plasticizer composition is the patent's separate, broader claimed embodiment (cols. 5–6), not what was tested. Rewrote the bullet to attribute the measured numbers to the correct, actually-tested composition and flag the broader claim separately.
8. **D5-S17 belt-speed-conversion location (Finding 4 bullet 4).** Re-read via text extraction with page-footer markers: the "proprietary Milrem velocity control require velocity for each belt" sentence and the vb,r/vb,l formulas sit immediately before the "117580S-11" footer, not "117580S-12." Fixed the citation to "pp. 117580S-2, 117580S-11." (The separate "two wheeled robot" quote cited at p. 117580S-12 elsewhere was independently re-checked and is correctly on that page — left unchanged.)
9. **D5-S02 section citation (Finding 7 bullet 4).** Re-read via text extraction: the entire quoted passage follows the "3.2 Tracked locomotion systems" header, not 3.1. Fixed the citation to "§3.2" only.
10. **D5-S03 exception-vehicle count (Finding 7 bullet 1).** Re-read "Conclusions" para. 95d directly: M561 is the sole exception for traverse speed, center-line speed and cargo-delivery rate, but fuel consumption separately excepts M561, M706 and M54A2. Rewrote the sentence to give both counts correctly instead of "one listed exception vehicle."
11. **D5-S14 track widths (Finding 4 bullet 2).** Re-read the saved product page: only 2.75-inch and 4-inch widths are documented (no 1-inch option). Narrowed "1–4 inch widths" to the widths actually in the source.
12. **D5-S21 page attributions (rows 55, 58, 112) — re-checked by rendering pages 8–10 as images; original citations (p. 9 for the tensioning/break-in/storage text, p. 10 for the sag/40%/worn-sprocket text) confirmed correct, no change applied.** PDF page 9 (the page physically between the printed "8" and "10" footers) carries no printed footer of its own, but it is the only page with no numeral, consistent with being the implied "p. 9" in this brochure's 8-[9]-10 sequence; page 8 (footer "8") is a photo/title divider with no related text at all. The audit's suggested "pp. 8–9" alternative and the verifier's "worth flagging" note were both considered and rejected after direct visual inspection.

**Sources — corrected per SOURCE_AUDIT.md's "Summary of corrections needed":**
1. Renamed `stoll_1977_wheeled_tracked_mobility_comparison.pdf` → `stoll_1970_wheeled_tracked_mobility_comparison.pdf` (document is dated March 1970; citation text already said 1970).
2. Renamed `kim_1992_track_tension_rice_combine_soilbin.pdf` → `park_1992_track_tension_rice_combine_soilbin.pdf` (first author is Park, per project filename convention; citation text already listed Park first).
3. Renamed `us11939012_2024_track_tension_monitoring_patent.pdf` → `drb_2024_track_tension_monitoring_patent.pdf` (assignee-org naming, matching the topic's other three patents).
4. D5-S12: fixed citation page range "178–185" → "178–187" (confirmed from the article's own printed footers and its own Citation line); renamed `adaptive_backstepping_tracked_slip.pdf` → `lu_2020_adaptive_backstepping_tracked_slip.pdf`.
5. D5-S18: added the six named student authors (R. Mattes, J. Bridon, A. Cascone, A. Brockett, M. Poremba, R. Pritz) and faculty advisor (S. Brennan) from the report's own title-page team table to the citation.
6. D5-S21: added a publication year (2020, from the PDF's CreationDate metadata and the body text's "20-10" print code) to the citation; renamed `deere_2022_ctl_rubber_track_maintenance.pdf` → `deere_2020_ctl_rubber_track_maintenance.pdf`; raised the evidence level from B to A per the audit's precedent argument (manufacturer maintenance/care guide, same genre as a user manual).

**Not changed (judgment calls, per the audit's own framing, not treated as errors):**
- D5-S03's Level A (government field-test report vs. Level B "official documentation") — audit explicitly flagged this as "a defensible call either way," not a correction. Left at A.
- The cross-topic patent-leveling question (D5's patents at B vs. D4-S01 at A) — audit flagged this as something "worth reconciling across topics," not a same-topic defect. Left unchanged; noted here for whoever next reconciles patent leveling project-wide.

**Mechanical check (after corrections):** `file` run on all 27 files in `sources/` — every type still matches its extension (23 PDF, 3 UTF-8 text/.md, 1 C++ source/.hpp). Every file in `sources/` has exactly one matching row in README's Sources table and vice versa (diffed programmatically). Every `D5-S##` citation appearing anywhere in README.md resolves to a table row, and the table runs D5-S01–D5-S27 with no gaps or duplicates. No source failed outright (`SOURCE_AUDIT.md`'s "Failing" list was empty), so no file or table row was deleted.
