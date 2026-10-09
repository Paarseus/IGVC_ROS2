# C2 — Drive kinematics: source audit

| | |
|---|---|
| **Topic** | C2 — Drive kinematics |
| **Date** | 2026-09-28 |
| **Reviewer** | independent — sources (STANDARDS.md §2–5, step 4) |
| **Scope** | README.md Sources table (C2-S01 … C2-S41), sources/ (41 files), SCOPE.md foundational list and subtopics 1–14 |

**Result:** 41 sources. None fail the acceptance checklist, so none must be removed. 17 need a correction (citation details, evidence level, pinned commit, file format or file bytes). Share of sources: 30 general (73 %) and 11 product-specific (27 %).

## Source table

Foundational means listed in SCOPE.md's *Foundational references* or the README's *Foundational references*. Product-specific means code, configuration or documentation for one software project or product line.

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| C2-S01 | mandow_2007_experimental_kinematics_skid_steer.pdf | IEEE/RSJ IROS 2007 (author copy, Univ. Málaga) | A | Foundational | General | Yes | Seminal paper; real 6-page PDF. **Correction:** add doi:10.1109/IROS.2007.4399139, which SCOPE gives and the README row leaves out. |
| C2-S02 | kozlowski_2004_skid_steering_model_control.pdf | Int. J. Appl. Math. Comput. Sci. (AMCS), open access | A | Foundational | General | Yes | Real 20-page pdfTeX PDF. **Correction:** `file` reports `data` because the file starts with a stray space byte before `%PDF-1.3`, so the type check fails. Remove the leading byte or download again. Also add the DOI or journal URL (amcs.uz.zgora.pl). |
| C2-S03 | baril_2020_skid_steer_kinematic_models.pdf | CRV 2020 (IEEE), kept as arXiv v1 | A | Supporting | General | Yes | The peer-reviewed venue is correctly cited. The open NORLAB PDF of the CRV version (https://norlab.ulaval.ca/pdf/Baril2020.pdf) could replace the arXiv copy (optional). |
| C2-S04 | wang_2015_skid_steer_laser_kinematics.md | Sensors (MDPI, DOI) | A | Supporting | General | Yes | **Correction (format):** this is a journal paper saved as Markdown, and the formulas are flattened. An open PDF exists: https://www.mdpi.com/1424-8220/15/5/9681/pdf |
| C2-S05 | rabiee_2019_friction_based_skid_steer_kinematics.pdf | IEEE ICRA 2019 (author copy) | A | Supporting | General | Yes | — |
| C2-S06 | dixit_2020_quasistatic_tracked_kinematics.pdf | arXiv preprint (Caltech) | C | Supporting | General | Yes | Level C is correct: no peer-reviewed version was found. **Correction:** the latest arXiv version of 2004.05176 is retitled "The Kinematics of Tracked Vehicles via the Power Dissipation Method". Note the retitling, and that the file kept is v2. |
| C2-S07 | zuo_2019_vins_skid_steer_icr.pdf | ISRR 2019, Springer Proceedings in Advanced Robotics (kept as arXiv v1) | A | Supporting | General | Yes | **Correction:** add the SPAR volume (vol. 20, *Robotics Research*) and its publication year (2022), and the DOI of the Springer chapter. The pages 741–756 could not be confirmed. |
| C2-S08 | samson_2025_drive_slip_protocol.pdf | IEEE Trans. Field Robotics (kept as arXiv v1) | A | Supporting | General | Yes | Acceptance in T-FR is confirmed. **Correction:** the volume and pages "2:380–399" could not be confirmed from open sources. Confirm them against IEEE Xplore or drop them and keep the DOI. |
| C2-S09 | shamah_1999_skid_vs_explicit_steering.pdf | CMU Robotics Institute, M.S. thesis / TR CMU-RI-TR-99-06 | A (thesis) | Supporting | General | Yes | **Correction (level):** STANDARDS §2 does not list theses as level A. State why a master's thesis is treated as A, or grade it consistently with other topics (B or C). |
| C2-S10 | ros2controllers_2024_diff_drive_controller.cpp | ros2_controllers (official project) | C | Supporting | Product | Yes | **Correction:** STANDARDS §2 lists ros2_controllers source as level **B**, not C. It is cited at the moving `humble` branch; pin a tag or commit. |
| C2-S11 | ros2controllers_2024_diff_drive_parameters.yaml | ros2_controllers | C | Supporting | Product | Yes | **Correction:** same as C2-S10 (level B; pin a commit). |
| C2-S12 | ros2controllers_2024_diff_drive_odometry.cpp | ros2_controllers | C | Supporting | Product | Yes | **Correction:** same as C2-S10 (level B; pin a commit). |
| C2-S13 | ros2controllers_2024_mobile_robot_kinematics.rst | ros2_controllers docs | B | Supporting | Product | Yes | **Correction:** it is cited at the `master` branch, a moving branch, while the robot runs Humble. Pin a commit and state the version. |
| C2-S14 | roscontrollers_2020_noetic_diff_drive_controller.cpp | ros_controllers (ROS 1, official) | C | Supporting | Product | Yes | ROS 1 is superseded but is kept for history, and that is acceptable. **Correction:** official-project source is level B; pin a commit or tag instead of `noetic-devel`. |
| C2-S15 | clearpath_2024_a200_husky_control.yaml | Clearpath clearpath_common | C | Supporting | Product | Yes | Level C is correct. **Correction:** pin a commit or tag instead of the `humble` branch. |
| C2-S16 | clearpath_2024_j100_jackal_control.yaml | Clearpath clearpath_common | C | Supporting | Product | Yes | **Correction:** pin a commit or tag. |
| C2-S17 | husky_2023_humble_devel_control.yaml | Clearpath husky repo | C | Supporting | Product | Yes | **Correction:** pin a commit or tag instead of `humble-devel`. |
| C2-S18 | husky_2021_noetic_control.yaml | Clearpath husky repo (ROS 1) | C | Supporting | Product | Yes | Superseded ROS 1 config, kept as historical evidence for the 1.875 value, which is acceptable. **Correction:** pin a commit or tag. |
| C2-S19 | nav2_2026_setup_transforms.md | Nav2 official documentation | B | Supporting | Product | Yes | — (a documentation page saved as Markdown is correct) |
| C2-S20 | nav2_2026_setup_footprint.md | Nav2 official documentation | B | Supporting | Product | Yes | — |
| C2-S21 | siegwart_2004_amr_ch3_mobile_robot_kinematics.pdf | MIT Press textbook (chapter hosted at CMU) | A | Foundational | General | Yes | — |
| C2-S22 | lynch_2017_modern_robotics.pdf | Cambridge Univ. Press textbook (authors' preprint) | A | Foundational | General | Yes | — |
| C2-S23 | yu_2011_skid_steer_dynamic_power_modeling.pdf | InTech open-access book chapter | A (book chapter) | Supporting | General | Yes | Acceptable. The README also relies on it for Wong / Wong–Chiang content, which it reports second-hand. |
| C2-S24 | yu_2009_skid_steer_dynamic_model_verification.pdf | IEEE/RSJ IROS 2009 | A | Supporting (conference version of a foundational work) | General | Yes | The journal version (IEEE T-RO 26(2):340–353, 2010) is correctly listed as not downloaded. |
| C2-S25 | endo_2007_tracked_slip_compensating_odometry.pdf | IEEE/RSJ IROS 2007 (author copy) | A | Supporting | General | Yes | **Correction:** the citation has no DOI; add it after confirming it on IEEE Xplore. |
| C2-S26 | helmick_2006_slip_compensated_path_following.pdf | Advanced Robotics (JPL copy) | A | Supporting | General | Yes | — |
| C2-S27 | angelova_2006_learning_to_predict_slip.pdf | IEEE ICRA 2006 (JPL copy) | A | Supporting | General | Yes | **Correction:** the citation has no pages or DOI; add them. |
| C2-S28 | seegmiller_2013_vehicle_model_identification_ipem.pdf | IJRR (CMU preprint) | A | Foundational | General | Yes | — |
| C2-S29 | galati_2019_tracked_skid_steer_terrain_awareness.pdf | Frontiers in Robotics and AI | A | Supporting | General | Yes | — |
| C2-S30 | ciloglu_2025_tracked_pure_pursuit_slip_ekf.md | Sensors (MDPI, DOI) | A | Supporting | General | Yes | **Correction (format):** this is a journal paper saved as text with the equations removed. An open PDF exists: https://www.mdpi.com/1424-8220/25/14/4242/pdf |
| C2-S31 | liu_2026_tracked_agricultural_slip_aware_tracking.pdf | Frontiers in Plant Science | A | Supporting | General | Yes | Checked against the PDF (Front. Plant Sci. 16:1754679, published 5 Jan 2026). |
| C2-S32 | chamorro_2025_rubber_track_crawler_mbs.pdf | Scientific Reports (Nature) | A | Supporting | General | Yes | — |
| C2-S33 | okawara_2025_neural_kinematic_model_lio_wheel.pdf | Robotics and Autonomous Systems (kept as arXiv v5) | A | Supporting | General | Yes | **Correction:** SCOPE's gap-check note says C2-S08 gained a T-FR venue and C2-S33 an RAS venue, and the README matches this. Add the RAS DOI to the README row. |
| C2-S34 | jia_2012_terramechanics_wheel_terrain_model.pdf | Robotica (author copy) | A | Foundational (stand-in for Bekker / Janosi–Hanamoto) | General | Yes | — |
| C2-S35 | campion_1996_classification_wmr.pdf | IEEE T-RA 1996; kept as the authorised Russian translation (Rus. J. Nonlinear Dyn. 2011) | A | Foundational | General | Yes | The translation is disclosed and page numbers refer to it, which is acceptable. English readers cannot check the quoted content directly. |
| C2-S36 | caracciolo_1999_four_wheel_skid_steer_tracking.pdf | IEEE ICRA 1999 (Sapienza DIAG scan) | A | Foundational | General | Yes | The PDF is image-only (CCITT scan, no text layer). This is disclosed. OCR would make the citations searchable (optional). |
| C2-S37 | martinez_2017_inertia_based_icr_tracked.pdf | IEEE SSRR 2017 (RIUMA author copy) | A | Supporting (stand-in for Martínez 2005) | General | Yes | The PDF is image-only. This is disclosed. |
| C2-S38 | martinez_2024_icr_kinematics_firm_slopes.pdf | IEEE/RSJ IROS 2024 (RIUMA author copy) | A | Supporting | General | Yes | — |
| C2-S39 | zhou_2022_large_skid_steer_ugv_slippage.pdf | Scientific Reports (Nature) | A | Supporting | General | Yes | — |
| C2-S40 | focchi_2026_pseudo_kinematic_tracked_control.pdf | Robotics and Autonomous Systems (Elsevier; IRIS copy) | A | Supporting | General | Yes | — |
| C2-S41 | trivedi_2024_probabilistic_skid_steer_motion_model.pdf | IEEE ICRA 2024 (kept as arXiv v2) | A | Supporting | General | Yes | **Correction:** the README row calls the kept copy "a 4-page version", but the file has 7 pages (pdfinfo). Fix the description. |

## Files
- `file` was run on all 41 files in `sources/`. 40 match their extension: 28 are PDFs; the rest are UTF-8 Markdown, ASCII YAML, C++ source, and one `.rst` that `file` reports as "LaTeX document", which is expected for reStructuredText.
- **Type mismatch:** `kozlowski_2004_skid_steering_model_control.pdf` is reported as `data`. The cause is one leading space byte before `%PDF-1.3`. `pdfinfo` and `pdftotext` read it as a real 20-page article, so this is a correction, not a failure. Strip the byte or download the file again.
- No error, login or bot-check pages were found. The text of the first pages was checked for every PDF. Two PDFs are image-only scans with no text layer (C2-S36, C2-S37); their content is real.
- Every file maps to exactly one row of the Sources table, and every row (S01–S41) has a file. No row is marked "not downloaded". Foundational works that were not downloaded appear only in the Foundational references table, where they are marked *not downloaded*.
- File names follow `<author>_<year>_<topic>.<ext>`. One note: `ros2controllers_2024_mobile_robot_kinematics.rst` comes from `master`, so its year is unknown until a commit is pinned.

## Format
Papers saved as text although an open PDF exists:
- C2-S04 Wang et al. 2015 (Sensors): https://www.mdpi.com/1424-8220/15/5/9681/pdf
- C2-S30 Çiloğlu & Kutluay 2025 (Sensors): https://www.mdpi.com/1424-8220/25/14/4242/pdf

All other papers, theses and books are saved as PDFs. Documentation (Nav2 `.md`, ros2_controllers `.rst`) and code/config (`.cpp`, `.yaml`) are in the correct format.

## Foundational
| Foundational reference (SCOPE.md) | Status |
|---|---|
| Campion et al. 1996 | Downloaded and cited as C2-S35 (authorised translation) |
| Siegwart & Nourbakhsh 2004 | Downloaded and cited as C2-S21 |
| Lynch & Park 2017 | Downloaded and cited as C2-S22 |
| Caracciolo et al. 1999 | Downloaded and cited as C2-S36 |
| Kozłowski & Pazderski 2004 | Downloaded and cited as C2-S02 (fix the file bytes) |
| Martínez et al. 2005 (IJRR) | Marked *not downloaded*, with the reason (no open copy); C2-S37/S38 stand in |
| Mandow et al. 2007 | Downloaded and cited as C2-S01 |
| Wong & Chiang 2001; Wong 2008 textbook | Marked *not downloaded*, with the reason (no open copy) |
| Bekker 1956; Janosi & Hanamoto 1961 | Marked *not downloaded*, with the reason; summarised through C2-S34 |
| Steeds 1950; Kitano & Kuma 1977 | Marked *not downloaded*, with the reason |
| Yu et al. 2010 (T-RO) | Marked *not downloaded*; the conference version C2-S24 is cited |
| Pentzer et al. 2014 (JFR) | Marked *not downloaded*, with the reason; reported second-hand through C2-S03 |
| Seegmiller et al. 2013 | Downloaded and cited as C2-S28, but **missing from the README's Foundational references table** although SCOPE lists it as foundational. Add it. |

## Coverage
| # | Subtopic | Cited findings | Main sources |
|---|---|---|---|
| 1 | Classification and constraints | Yes | S21, S22, S35, S02, S36 |
| 2 | ICR kinematics | Yes | S01, S02, S03, S04, S06, S25 |
| 3 | Effective width / multiplier | Yes | S01, S03, S04, S05, S29; products S10–S18 |
| 4 | What changes the effective width | Partly (no identified value on grass for rubber tracks; no payload sweep on a single vehicle; track tension and footprint ratio missing). The gaps are stated in Open questions. | S01, S05, S08, S29, S37, S38, S39 |
| 5 | Terramechanics | Partly; only secondary sources (primary works not downloaded; no K/μ values for turf) | S23, S32, S34 |
| 6 | Dynamic and power models | Yes | S23, S24, S02, S36 |
| 7 | Offline calibration | Yes (thin on grass) | S01, S03, S04, S08, S28, S41 |
| 8 | Online estimation and learning | Yes | S07, S25, S26, S27, S28, S30, S33 |
| 9 | Accuracy and failure modes | Partly (no consolidated benchmark protocol) | S03, S08, S28, S41 |
| 10 | Model in motion control | Yes | S25, S26, S30, S31, S40 |
| 11 | Reference implementations (product-specific, last) | Yes | S10–S20 |
| 12 | Slopes and load transfer | Yes | S38, S40 |
| 13 | Inertia and speed dependence | Yes | S36, S37, S40 |
| 14 | Data-driven models on grass | Yes | S05, S08, S33, S41 |

- **General first:** the findings sections put general and academic material first. Product-specific controller and config material is limited to the last findings section ("Reference controllers and configs … (product-specific)") and a mention in the Summary. This satisfies the rule.
- **Share:** 30 general (73 %) and 11 product-specific (27 %: C2-S10–S20).
- Every source S01–S41 is cited in the findings at least once.
