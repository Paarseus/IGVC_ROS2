# D1 — Swerve drive modules: source audit

**Topic:** D1_swerve_drive
**Date:** 2026-10-06
**Reviewer:** independent — sources (step 4 of STANDARDS.md §5)

Scope: STANDARDS.md §§2–5 applied to `README.md`, `SCOPE.md` and `sources/` for this topic only. README.md was not edited. 43 source rows (D1-S01–D1-S43), 47 downloaded files checked.

## Source table

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| D1-S01 | campion_1996_classification_wmr.pdf | *IEEE Trans. Robotics and Automation* / Russian J. Nonlinear Dynamics (kept translation) | A | Yes | General | Yes | Peer-reviewed, DOI, content verified (pdftotext) |
| D1-S02 | siegwart_2004_amr_ch3_mobile_robot_kinematics.pdf | MIT Press (CMU-hosted ch. 3) | A | Yes | General | Yes, note | Textbook, content verified. Cites "2004" 1st ed.; SCOPE.md's foundational entry names the 2nd ed. (2011, Siegwart/Nourbakhsh/Scaramuzza) — edition not reconciled |
| D1-S03 | lynch_2017_modern_robotics.pdf | Cambridge Univ. Press (authors' preprint) | A | Yes | General | Yes | Textbook, author-authorized open copy, content verified |
| D1-S04 | not downloaded | CMU-RI tech report / *J. Robotic Systems* | A | Yes | General | Yes | Correctly marked not-downloaded with reason (copyright restriction, re-confirmed via Unpaywall); not cited for content |
| D1-S05 | ether_2011_derivation_inverse_kinematics_swerve.pdf; ether_2011_swerveN_general_case.pdf | Self-published, Chief Delphi ("Ether") | D | **Yes (SCOPE.md lists it as foundational — missing from README's Foundational references table)** | General | Yes, note | Pseudonymous author, but this is the de facto originating source for the near-universal algorithm (A–C sources don't exist for it); content verified genuine (pdftotext). D-level use matches STANDARDS' "used only when A–C are missing," though the one worked example given (GitHub/Discourse maintainer answers) doesn't literally cover a self-published whitepaper — reasonable but worth a standards-owner nod |
| D1-S06 | not downloaded | *The Journal of Engineering* (IET) | A | Yes | General | Yes, note | Peer-reviewed, DOAJ-listed OA, but only the abstract was read (full text blocked by Cloudflare on every route) — borders STANDARDS' "Content" checklist item rejecting "abstract only when the finding needs the full text"; disclosed every time, finding stays within what the abstract states |
| D1-S07 | dietrich_2011_singularity_avoidance_variable_footprint.pdf | IEEE ICRA 2011 | A | Yes | General | Yes | Peer-reviewed, DOI, content verified |
| D1-S08 | not downloaded | IEEE SMC 2005 | A | Yes | General | Yes | Correctly marked not-downloaded (NTRS record confirmed, no file); explicitly not cited for content |
| D1-S09 | not downloaded | SAE International | A | Yes | General | Yes | Correctly marked not-downloaded (paywalled textbook); explicitly not cited |
| D1-S10 | wpilib_2026_swerve_drive_kinematics.rst; wpilib_2026_intro_chassis_velocities.rst | WPILib docs (official FRC project) | B | No | General (software) | Yes | Official project docs, pinned commits, content spot-checked verbatim against Findings §2 claims |
| D1-S11 | wpilib_2026_swerve_drive_odometry.rst | WPILib docs | B | No | General (software) | Yes | Pinned commit; quoted text verified |
| D1-S12 | pedersen_2022_swerve_drive_second_order_kinematics.pdf | FRC Team 449 (The Blair Robot Project) | C | No | General | Yes | Named author/team report; title/author/date verified by pdftotext against citation |
| D1-S13 | yagsl_2026_swerve_drive_kinematics.md; yagsl_2026_chassis_control.md | YAGSL docs (community library) | C | No | General (software) | Yes | Open-source project docs, correctly leveled C (not B — not an official ROS/FRC project) |
| D1-S14 | yagsl_2026_swerve_modules.md; yagsl_2026_swerve_drift_causes.md | YAGSL docs | C | No | General (software) | Yes | Same as above |
| D1-S15 | clavien_2010_icr_estimation_omnidirectional_robot.pdf | IEEE ICRA 2010 (author copy) | A | No | General | Yes | Peer-reviewed, content verified |
| D1-S16 | shanjaya_2024_three_wheel_swerve_kinematics_matrix.pdf | *Jurnal Elkolind* (open access, DOI) | C | No | General | Yes | DOI present but venue not established-tier (not in STANDARDS' A examples) — conservatively leveled C, appropriate |
| D1-S17 | ros2controllers_2026_steering_controllers_library_userdoc.rst | `ros2_controllers` (official ROS 2 project) | B | No | General (software) | Yes | Pinned commit; quoted text (Bicycle/Tricycle/Ackermann, non-holonomic framing) verified verbatim |
| D1-S18 | nav2_2026_mppi_controller_readme.md; nav2_2026_mppi_motion_models.hpp | Nav2 (official project) | B | No | General (software) | Yes | Pinned commits; `OmniMotionModel`/`isHolonomic()` claims verified verbatim against the .hpp file |
| D1-S19 | rev_2026_maxswerve_module.md | REV Robotics | A | No | Product | Yes, note | Manufacturer spec; all quoted numbers (ratio, weight, price, footprint) verified verbatim. Page links an un-downloaded PDF (`REV-21-3005-DR.pdf`) — see Format section |
| D1-S20 | rev_2026_easyswerve_module.md | REV Robotics | A | No | Product | Yes | Manufacturer spec, content verified |
| D1-S21 | sds_2026_mk4n_module.md | Swerve Drive Specialties (via AndyMark) | A | No | Product | Yes | Steering ratio (18.75:1) and price band verified verbatim |
| D1-S22 | sds_2026_mk4c_module.md | Swerve Drive Specialties | A | No | Product | Yes | Manufacturer spec, genuine content |
| D1-S23 | sds_2026_mk4i_module.md | Swerve Drive Specialties | A | No | Product | Yes | Steering ratio (150/7:1) and belt detail verified verbatim |
| D1-S24 | wcp_2026_swervex2_*.md (5 files) | West Coast Products | A | No | Product | Yes | GitBook-exported markdown; genuine, on-topic content (see Files section re: `file` mismatches) |
| D1-S25 | thriftybot_2026_thrifty_swerve_module.md | ThriftyBot | A | No | Product | Yes | Manufacturer spec, genuine content |
| D1-S26 | armabot_2026_differential_swerve_drive.md | ARMABOT | A | No | Product | Yes, note | Genuine (if thin/confusing) product page. Links 3 further PDFs (Outline Drawing, Speed Chart, Installation Instructions) not downloaded — see Format section; Speed Chart likely contains the gear-ratio data the README says is missing |
| D1-S27 | liftaloft_2013_omnidirectional_drive_steering_unit_patent.pdf | US Patent 8,393,431 B2 (USPTO) | A | No | Product | Yes | Granted patent, named inventors/assignee, content verified |
| D1-S28 | liftandaccess_2019_liftaloft_spw16spl_crab_steering.md | *Lift and Access* (trade press) | D | No | Product | Yes, note | Date verified (29 Mar 2019). Trade press isn't one of STANDARDS' Level-D worked examples; used only because no official spec sheet exists — reasonable but a borderline extension of the rule, same as D1-S05 |
| D1-S29 | gatech_2022_swervi_igvc_design_report.pdf | RoboJackets, IGVC 2022 design report | C | No | General (project) | Yes | Exact STANDARDS Level-C example ("IGVC design reports"); author/team verified by pdftotext |
| D1-S30 | sooner_2025_twistopher_igvc_design_report.pdf | Sooner Competitive Robotics, IGVC 2025 design report | C | No | General (project) | Yes | Same, verified |
| D1-S31 | baby_2024_4wis4wid_agricultural_field_robot_drl.pdf | arXiv preprint | C | No | General | Yes | Preprint rule correctly applied; re-checked today (WebSearch) — still no peer-reviewed venue found, correctly C |
| D1-S32 | caran_2025_4wis4wid_odometry_calibration_pose_estimation.pdf | Accepted IEEE ECMR 2025 (arXiv preprint) | A | No | General | Yes | Preprint-with-confirmed-venue correctly graded A per STANDARDS §2 |
| D1-S33 | wcp_2026_swervex_*.md (4 files) | West Coast Products | A | No | Product | Yes | Genuine content (see Files section re: naming of "Flipped" as a configuration, not a separate product) |
| D1-S34 | nav2_2026_regulated_pure_pursuit_readme.md | Nav2 (official project) | B | No | General (software) | Yes | Pinned commit; holonomic/nonholonomic scoping quote verified verbatim |
| D1-S35 | zinger_2024_swerve_controller_readme.md | `pvandervelde/zinger_swerve_controller` (community) | C | No | General (software) | Yes | Correctly leveled C (not an official `ros-controls` package); repo metadata independently checkable |
| D1-S36 | calcmogul_2024_sysid_swerve_chiefdelphi.json | Chief Delphi forum ("calcmogul") | D | No | General | Yes, note | Legitimate D-level maintainer-adjacent community answer, used because no official doc covers this. **Filename says 2024 but the cited posts are dated 2023-01-20 to 2023-02-06** (verified directly from the JSON's own timestamps) — rename/re-date |
| D1-S37 | teal_2025_cosmos_differential_holonomic_drive.pdf | engrxiv preprint (UT Dallas VEXU team) | C | No | General (project) | Yes | Preprint rule correctly applied; re-checked today (WebSearch) — still no peer-reviewed venue, correctly C |
| D1-S38 | ctre_2026_canbus_utilization.html | CTR Electronics (CTRE) | A | No | Product | Yes | Manufacturer spec, content verified; downstream 8-module arithmetic correctly labeled as this research's own derivation, not a vendor figure |
| D1-S39 | not downloaded | IEEE ICRA 2016 | A | No | General | Yes, note | Correctly marked not-downloaded (HAL bot-wall); abstract-only finding, same caveat as D1-S06 |
| D1-S40 | not downloaded | *Robotics and Autonomous Systems* (Elsevier) | A | No | General | Yes, note | Correctly marked not-downloaded (paywalled); abstract-only finding, same caveat |
| D1-S41 | not downloaded | *Robotics and Autonomous Systems* (Elsevier) | A | No | General | Yes, note | Same as D1-S40 |
| D1-S42 | shamah_1999_skid_vs_explicit_steering_thesis.pdf | CMU-RI master's thesis (CMU-RI-TR-99-06) | A | No | General | Yes, note | Content verified genuine and directly on-point. **Level consistency flag:** this exact thesis is graded A here and in `C2_drive_kinematics/README.md`, but **C** in this repo's own `D2_wheeled_skid_steer/README.md` (D2-S34) for the identical document; `D3_ackermann_steering` grades two other CMU-RI theses (Holand, Deshpande) as C. STANDARDS' Level-A row names "peer-reviewed research" and "textbooks," not theses specifically — the project has not settled a single rule for CMU-RI tech-report theses |
| D1-S43 | wpilib_2026_sysid_introduction.html | WPILib docs (official FRC project) | B | No | General (software) | Yes | Official docs; "three mechanism types, no swerve mode" claim verified verbatim |

**Failing (remove):** none. No source in this topic is an error/login/bot-check page, fabricated, off-topic, or otherwise unrecoverable under the checklist.

**Needs correction (pass, but fix something):** D1-S02, D1-S05, D1-S06, D1-S19, D1-S26, D1-S28, D1-S36, D1-S39, D1-S40, D1-S41, D1-S42 — see Reason column above and the sections below.

## Files

`file` was run on all 47 downloaded files (see command output this session). Result: **every file is genuine content matching its topic** — no error pages, login walls, or bot-check pages were found anywhere in `sources/`. Spot-checked every PDF's first page with `pdftotext` (18 PDFs) and every flagged text file's head; all show real, on-topic text matching their citation.

Six files were flagged by `file`'s type guess but are **not actually mismatched** on inspection:
- `wcp_2026_swervex2_general_specs.md`, `wcp_2026_swervex2_ratio_options.md`, `wcp_2026_swervex_individual_components.md` → reported as "HTML document." Cause: these are genuine GitBook-exported Markdown pages (confirmed by their own `> ... available as [Markdown](...)` header) that embed raw HTML (`<figure>`, `<table>`, `<img>`) inline, which is normal GitBook Markdown and is dense enough to flip `file`'s heuristic. Real content, correctly saved per STANDARDS §3 ("web documentation → clean markdown").
- `wpilib_2026_swerve_drive_odometry.rst` → reported as "Python script." Cause: Sphinx `:ref:` role syntax in genuine WPILib RST source, misread by `file`'s heuristics.
- `yagsl_2026_swerve_drive_kinematics.md` → reported as "Java source." Cause: same GitBook-Markdown-with-embedded-HTML pattern as above.

No action needed on these six beyond this note — they are not bot-check/error pages.

**Row/file correspondence:** every one of the 48 `sources/...` paths referenced in the README's Sources table resolves to an existing file, and every file in `sources/` is referenced by exactly one Source-table row (checked both directions programmatically — zero orphans either way). Every row without a file is explicitly marked "not downloaded" (D1-S04, S06, S08, S09, S39, S40, S41 — 7 rows, each with a stated reason).

## Format

STANDARDS §3 prefers an open PDF over saved web text for "papers, books, theses, standards, datasheets ... whenever any open PDF exists." All papers, theses, and the one patent in this topic are already correctly saved as PDF. Two product pages, however, **link further PDF documents that were not fetched**, found by grepping the saved Markdown for embedded PDF URLs:

- **D1-S26 (ARMABOT, `armabot_2026_differential_swerve_drive.md`)** links three PDFs that were not downloaded: `Armabot_A0067_Differential_Swerve_Outline_Drawing.PDF`, `Armabot_A0067_Differential_Swerve_Speed_Chart.PDF`, `Armabot_A0067_Differential_Swerve_Installation_Instructions.PDF` (all at `cdn.shopify.com/s/files/1/1518/8108/files/...`). This matters beyond a format nitpick: the README's own Findings §10 says ARMABOT's "product page gives no gear ratio, module weight, or sealing specification" — the **Speed Chart** PDF is exactly the kind of document likely to carry gear-ratio/speed data, so this gap may be an artifact of not fetching the linked datasheet rather than a genuine vendor silence. Recommend fetching these three PDFs and updating Findings §10 / Key numbers if they add data.
- **D1-S19 (REV MAXSwerve, `rev_2026_maxswerve_module.md`)** links one PDF not downloaded: `https://revrobotics.com/content/docs/REV-21-3005-DR.pdf` (likely a dimensional/reference drawing, given the "-DR" suffix and REV's own SKU). Lower priority than the ARMABOT case since the page's own spec text already gives detailed numbers, but still an open PDF that STANDARDS' format rule prefers to capture.

No other `sources/*.md` file (checked: all REV, SDS, WCP, ThriftyBot, Nav2, YAGSL, zinger pages) contains an embedded link to a further PDF.

All web documentation proper (REV/SDS/WCP/ThriftyBot/ARMABOT/CTRE/WPILib/Nav2/YAGSL/zinger/ros2_controllers pages) is correctly saved as Markdown, RST, or HTML text rather than screenshotted/PDF'd, per STANDARDS §3's "web documentation → clean markdown or text" rule — there is no open standalone PDF *of the page itself* for any of these (the two gaps above are supplementary linked documents, not the pages themselves).

## Foundational

SCOPE.md identifies **9** foundational references for this topic. All 9 are present in the Sources table with a status:

| Reference | Status |
|---|---|
| Campion, Bastin, d'Andréa-Novel 1996 (D1-S01) | Downloaded, cited |
| Muir & Neuman 1986/87 (D1-S04) | Not downloaded, reason given (copyright restriction + Unpaywall re-check), not cited for content |
| Siegwart & Nourbakhsh (D1-S02) | Downloaded, cited |
| Lynch & Park 2017 (D1-S03) | Downloaded, cited |
| "Ether" whitepaper (D1-S05) | Downloaded, cited |
| Lee & Li 2015, 4WIS4WID (D1-S06) | Abstract-only (full text blocked), cited at abstract level |
| Dietrich et al. 2011 (D1-S07) | Downloaded, cited |
| Lindemann & Voorhees 2005, MER (D1-S08) | Not downloaded, reason given, not cited for content |
| Gillespie 1992 (D1-S09) | Not downloaded, reason given, not cited for content |

6 of 9 are downloaded and used; 3 are correctly marked "not downloaded" with a stated reason (one of those three, D1-S06, is further nuanced as "abstract only," a middle state STANDARDS doesn't explicitly name but which is transparently disclosed).

**Gap found:** the README's own "Foundational references" table (lines 23–32) lists only **8** of these 9 — it omits the "Ether" whitepaper (D1-S05) entirely, even though SCOPE.md explicitly names it foundational, it is downloaded, and it is one of the most heavily-cited sources in Findings §2 (inverse kinematics). This should be added to that table. No other foundational reference is missing.

## Coverage

All 17 subtopics listed in SCOPE.md's Subtopics table (10 from the initial map, plus 7 added at the 2026-10-06 gap check: steering/drive-feedforward characterization, Nav2 non-MPPI controllers, community `ros2_control` package maturity, differential-swerve academic treatment, singularity literature beyond Dietrich, CAN-bus data loading, and the same-vehicle steered-vs-skid precision comparison) have cited findings in the README, each traceable to the Findings section and source IDs checked above. Spot-checked a representative sample of quoted text/numbers against the underlying files (WPILib kinematics/odometry, `ros2_controllers` userdoc, Nav2 MPPI `.hpp` and README, Nav2 RPP README, REV MAXSwerve specs, SDS MK4n/MK4i specs, Pedersen paper, ARMABOT page, Lift and Access date, calcmogul post dates) — every one matched the README's quotation or figure exactly.

Findings are ordered general-first: Findings §1–9 (classification, kinematics, singularities, module control, odometry, failure modes, testing, software, outdoor deployment) are general/method-level, and the fully product-specific material (§10, off-the-shelf vendor modules) is deliberately placed last, per STANDARDS §1's "general first, specific second" rule. Section 4 (closed-loop module control) interleaves general architecture with specific vendor encoder examples, which is reasonable illustrative use rather than a violation (the general principle is always stated first within each bullet).

**General vs. product-specific split**, counting all 43 source rows (including the 7 not-downloaded ones, by what they would be if obtained): **31 general (≈72%)**, **12 product-specific (≈28%)** — D1-S19–S28, D1-S33, D1-S38. This reflects STANDARDS §1's instruction that vendor/product material should be "one section, not the whole topic," and matches the README's own structure.

No subtopic is left with zero findings. The remaining honestly-unanswered points (a numeric steering-rate-saturation-vs-tracking-error figure; a vendor-module steering-lag figure; brownout-specific current-draw quantification; automotive-style scrub-radius analysis; mud/standing-water ingress reports; non-manufacturer durability data) are correctly carried in the README's "Open questions" section with citations showing what *was* searched, rather than silently dropped — this matches STANDARDS §4 ("Nothing is stated without a source. Anything that cannot be sourced is dropped, or listed under Open questions").
