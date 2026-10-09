# L4 — Sensor fusion: source audit

- **Topic:** L4 — Sensor fusion
- **Date:** 2026-09-28
- **Reviewer:** independent — sources (STANDARDS.md §2–5, step 4)
- **Scope of check:** README.md Sources table (L4-S01 – L4-S62), SCOPE.md foundational references and subtopics, every file in `sources/` (59 files). `file` and `pdfinfo`/`pdftotext` were run on every file; pinned tags were checked with `git ls-remote` (robot_localization 3.5.4 = 8696ee5, common_interfaces 4.2.4 = 53761cb, clearpath_common 1.3.9 = bd39583, nmea_navsat_driver 2.0.1 = 861323c, autoware_core 1.9.0 = f25f83c — all exist).

**Result:** 62 sources; **0 failing**; **9 need correction**. No file is an error, login or bot-check page.

## Source table

F = foundational, S = supporting. Gen = general principle / theory / standard; Prod = product-specific (robot_localization, Nav2, Autoware, Clearpath, drivers).

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| L4-S01 | kalman_1960_new_approach_linear_filtering.pdf | Trans. ASME J. Basic Eng. (via UNC copy, web.archive) | A | F | Gen | Yes | Real paper (12 pp.). Copy is a retyped reproduction ("Kalman15.doc"), so PDF pages ≠ printed pp. 35–45; README already cites by PDF page. |
| L4-S02 | julier_1997_new_extension_kalman_filter.pdf | Proc. SPIE 3068 (AeroSense) | A | F | Gen | Yes — needs correction | DOI missing: add doi:10.1117/12.280797. |
| L4-S03 | julier_2004_unscented_filtering.pdf | Proc. IEEE 92(3) | A | F | Gen | Yes | Citation correct. |
| L4-S04 | smith_1986_spatial_uncertainty.pdf | IJRR 5(4) | A | F | Gen | Yes | 14-page copy carries IJRR vol. 5 no. 4 running heads. |
| L4-S05 | not downloaded | Wiley book | A | F | Gen | Yes | No open copy; cited only in Open questions. Correct. |
| L4-S06 | not downloaded | MIT Press book | A | F | Gen | Yes | No authorised open copy; not cited for findings. Correct. |
| L4-S07 | moore_2014_generalized_ekf_ros.pdf | IAS-13, AISC 302, Springer | A | F | Prod | Yes | Revised author copy (6 pp.) hosted in official docs; citation correct. |
| L4-S08 | ros_rep103_units_coordinates.rst | ROS REP (pinned commit) | A | F | Gen | Yes | Authors Foote & Purvis match file header. (SCOPE.md wrongly lists "Foote, Meeussen" — README is right.) |
| L4-S09 | ros_rep105_coordinate_frames.rst | ROS REP (pinned commit) | A | F | Gen | Yes | Correct. |
| L4-S10 | moore_2015_roscon_robot_localization.pdf | ROSCon 2015 (official) | B | F | Prod | Yes | Real 18-slide deck by maintainer. |
| L4-S11 | not downloaded | IEEE TAC | A | F | Gen | Yes | No open copy; only in Open questions. |
| L4-S12 | not downloaded | IEEE TAES | A | F | Gen | Yes | No open copy; only in Open questions. |
| L4-S13 | larsen_1998_time_delayed_measurements.pdf | Proc. 37th IEEE CDC (DTU Orbit, version of record) | A | F | Gen | Yes | Correct. |
| L4-S14 | huang_2010_observability_consistent_ekf.pdf | IJRR 29(5) (author copy) | A | F | Gen | Yes | Correct. |
| L4-S15 | robotlocalization_3.5.4_state_estimation_nodes.rst | robot_localization official docs, tag 3.5.4 | B | F | Prod | Yes | Tag verified. |
| L4-S16 | robotlocalization_3.5.4_preparing_sensor_data.rst | same | B | F | Prod | Yes | — |
| L4-S17 | robotlocalization_3.5.4_configuring_robot_localization.rst | same | B | F | Prod | Yes | — |
| L4-S18 | robotlocalization_3.5.4_integrating_gps.rst | same | B | F | Prod | Yes | — |
| L4-S19 | robotlocalization_3.5.4_navsat_transform_node.rst | same | B | F | Prod | Yes | — |
| L4-S20 | robotlocalization_3.5.4_params_ekf.yaml | official repo, tag 3.5.4 | B | F | Prod | Yes | — |
| L4-S21 | robotlocalization_3.5.4_params_dual_ekf_navsat_example.yaml | same | B | F | Prod | Yes | — |
| L4-S22 | robotlocalization_3.5.4_params_navsat_transform.yaml | same | B | F | Prod | Yes | — |
| L4-S23 | robotlocalization_3.5.4_filter_base.cpp | same | B | F | Prod | Yes | — |
| L4-S24 | robotlocalization_3.5.4_ekf.cpp | same | B | F | Prod | Yes | — |
| L4-S25 | robotlocalization_3.5.4_ros_filter.cpp | same | B | F | Prod | Yes | — |
| L4-S26 | robotlocalization_3.5.4_navsat_transform.cpp | same | B | F | Prod | Yes | — |
| L4-S27 | robotlocalization_3.5.4_CHANGELOG.rst | same | B | F | Prod | Yes | — |
| L4-S28 | ros_rep145_imu_driver_conventions.rst | ROS REP (pinned commit) | A | S | Gen | Yes — needs correction | Author missing: file header gives Paul Bovbel; REP status is **Draft** — state this in the citation (level A as a draft REP should be noted). |
| L4-S29 | ros2_common_interfaces_4.2.4_NavSatFix.msg; …NavSatStatus.msg | ROS 2 official repo, tag 4.2.4 | B | S | Prod | Yes | Tag verified. |
| L4-S30 | nav2_docs_setup_transforms.md | Nav2 official docs (pinned commit) | B | S | Prod | Yes | — |
| L4-S31 | nav2_docs_setup_robot_localization.md | Nav2 official docs | B | S | Prod | Yes | — |
| L4-S32 | nav2_docs_navigation2_with_gps.md | Nav2 official docs | B | S | Prod | Yes | Author "Pedro Gonzalez, Kiwibot" confirmed in file. |
| L4-S33 | nav2tutorials_gps_demo_dual_ekf_navsat_params.yaml | navigation2_tutorials (official, pinned commit) | B | S | Prod | Yes | — |
| L4-S34 | clearpath_common_a200_localization.yaml | Clearpath Robotics repo, tag 1.3.9 | C | S | Prod | Yes | Tag verified; level C matches STANDARDS example ("Clearpath configs"). |
| L4-S35 | welch_2006_intro_kalman_filter.pdf | UNC Tech. Rep. TR 95-041 | C | S | Gen | Yes | University tech report; C is consistent (not peer reviewed). |
| L4-S36 | maybeck_1979_stochastic_models_ch1.pdf | Academic Press textbook, ch. 1 | A | S | Gen | Yes | — |
| L4-S37 | labbe_kbfp_07_kalman_filter_math.md | Open book (CC-BY), GitHub pinned commit | C | S | Gen | Yes | `file` reports "LaTeX document" — heuristic only; content is the real notebook text. No PDF edition exists. |
| L4-S38 | labbe_kbfp_08_designing_kalman_filters.md | same | C | S | Gen | Yes | `file` reports "Python script" — heuristic on code cells; content is real. |
| L4-S39 | macenski_2023_desks_of_ros_maintainers.pdf | Robotics and Autonomous Systems 168 (arXiv v2 copy) | A | S | Gen | Yes | Peer-reviewed version cited correctly. |
| L4-S40 | locusrobotics_fuse_README.md | fuse official repo (rolling, commit 40e31ec) | B | S | Prod | Yes — needs correction | README on `rolling` states the ROS 2 port is "work in progress … **not** expected to work". README §2 cites it for "the ROS 2 `fuse` package implements a fixed-lag smoother". Either pin a released ROS 2 tag of fuse or state the WIP status in the citation/finding. |
| L4-S41 | barrau_2017_invariant_ekf_stable_observer.pdf | IEEE TAC 62(4) (arXiv v4 copy) | A | S | Gen | Yes | Peer-reviewed version cited correctly. |
| L4-S42 | sola_2017_quaternion_kinematics_eskf.pdf | arXiv 1711.02508 | C | S | Gen | Yes | No peer-reviewed version; C is correct per the preprint rule. |
| L4-S43 | chen_2018_weak_in_the_nees.pdf | FUSION 2018 (arXiv v1 copy) | A | S | Gen | Yes | Peer-reviewed version cited. |
| L4-S44 | jiang_2018_adaptively_robust_mahalanobis_gps_ins.pdf | Sensors 18(3), 695 (MDPI, DOI) | A | S | Gen | Yes | DOI confirmed in PDF. |
| L4-S45 | gao_2019_robust_ckf_mahalanobis_ins_gnss.pdf | Sensors 19(23), 5149 (MDPI) | A | S | Gen | Yes | DOI confirmed in PDF. |
| L4-S46 | dissanayake_2001_vehicle_model_constraints.pdf | IEEE Trans. Robotics & Automation 17(5) | A | S | Gen | Yes | — |
| L4-S47 | rakun_2022_rovitis_vineyard_fusion.pdf | Int. J. Agric. & Biol. Eng. 15(6) | A | S | Gen | Yes | Content correct (pp. 91–95); PDF metadata title is an unrelated article ("Misfiring Fault Diagnosis…") — harmless publisher error. |
| L4-S48 | sunderhauf_2012_gnss_multipath_robust_optimization.pdf | IEEE IV 2012 (author copy) | A | S | Gen | Yes | DOI printed on copy. |
| L4-S49 | sunderhauf_2013_factor_graph_unknown_delays.pdf | ESA ASTRA 2013 (author copy) | A | S | Gen | Yes — needs correction | ASTRA is an ESA symposium with abstract-based selection, not a clearly peer-reviewed full-paper venue; level A is not justified by §2. Downgrade to C, or cite evidence of full-paper review. |
| L4-S50 | sturm_2012_tum_rgbd_benchmark.pdf | IEEE/RSJ IROS 2012 (author copy) | A | S | Gen | Yes | Used only for ATE/RPE definitions. |
| L4-S51 | nmea_navsat_driver_2.0.1_driver.py | ros-drivers repo, tag 2.0.1 | C | S | Prod | Yes | Tag verified. |
| L4-S52 | rl_issue_417.md | GitHub issue, maintainer reply | D | S | Prod | Yes | Maintainer answer; used where A–C are silent. |
| L4-S53 | rl_issue_630.md | GitHub issue, maintainer reply | D | S | Prod | Yes | Same. |
| L4-S54 | anderson_1979_optimal_filtering.pdf | Prentice-Hall textbook (authors' ANU scan, 367 pp.) | A | F | Gen | Yes | Full book, real content. |
| L4-S55 | zhang_2020_noise_covariance_identification.md | IEEE Access 8 (open access) | A | S | Gen | Yes — needs correction | Paper saved as converted text though an open PDF exists (see Format). Replace with PDF and cite by page. |
| L4-S56 | ajgl_2014_covariance_intersection_dynamical_systems.pdf | FUSION 2014 (KIT ISAS author copy) | A | S | Gen | Yes | — |
| L4-S57 | zhu_2018_gnss_integrity_urban_review.pdf | IEEE T-ITS 19(9) (HAL copy) | A | S | Gen | Yes | — |
| L4-S58 | liu_2018_tightly_coupled_ppp_ins_land_vehicle.pdf | Sensors 18(12), 4305 | A | S | Gen | Yes | DOI confirmed in PDF. |
| L4-S59 | autoware_docs_coordinate_system.md | Autoware official docs (pinned commit) | B | S | Prod | Yes | — |
| L4-S60 | autoware_docs_localization_design.md | Autoware official docs, "architecture v1" | B | S | Prod | Yes — needs correction | Currency: this is the legacy *architecture v1* design doc; the citation should say it is the v1 (superseded) design so readers do not take it as the current Autoware architecture. |
| L4-S61 | autoware_core_1.9.0_ekf_localizer_README.md | autoware_core official repo, tag 1.9.0 | B | S | Prod | Yes | Tag verified. |
| L4-S62 | autoware_core_1.9.0_gnss_poser_README.md | same | B | S | Prod | Yes | — |

### Failing (must be removed)

None.

### Needs correction

1. **L4-S02** — add DOI 10.1117/12.280797.
2. **L4-S28** — add author (P. Bovbel) and note REP-145 status "Draft".
3. **L4-S40** — rolling-branch README says the ROS 2 port is WIP and not expected to work; pin a released ROS 2 tag or qualify the §2 finding.
4. **L4-S49** — ASTRA 2013 symposium: level A not justified; downgrade to C (or document full-paper peer review).
5. **L4-S55** — saved as converted text; download the open-access PDF and switch citations to page numbers.
6. **L4-S60** — mark as Autoware *architecture v1* (legacy) design doc.
7. **README Foundational references table** — SCOPE.md foundational items Groves 2013, Julier & Uhlmann 1997 (Covariance Intersection, ACC) and Fitzgerald 1971 are missing from the README's Foundational references and Sources tables; add them as *not downloaded* with the reason (no open copy).
8. **SCOPE.md** (informational, not README) — REP-103 authors listed as "Foote, Meeussen"; correct is Foote & Purvis.
9. **L4-S01** (minor) — note in the citation that the file is a retyped reproduction, so PDF pages do not match printed pp. 35–45.

## Files

- 59 files in `sources/`; all 59 map to a Sources-table row (L4-S29 has two files). Every table row has a file except L4-S05, S06, S11, S12, which are marked *not downloaded* with a reason.
- All 26 `.pdf` files are real PDFs (`file`: "PDF document") with the expected title/authors on page 1. No HTML, error, login or bot-check pages.
- `.rst`, `.yaml`, `.msg`, `.cpp`, `.py` files are the raw source/doc files. `.md` files are clean text; three `file` labels ("exported SGML", "LaTeX document", "Python script") come from `file`'s content guesses on HTML comments, LaTeX maths and code cells, not from a wrong format — checked by reading the heads.
- `file` reports odd page counts for some PDFs (e.g. 2 pages for huang, 21 for anderson); `pdfinfo` gives the true counts (20, 367). No truncated files.
- File names follow `<author-or-org>_<year>_<topic>` except the GitHub issues (`rl_issue_417.md`, `rl_issue_630.md`) and Labbe (`labbe_kbfp_07…`), which lack a year — minor; suggest `moore_2018_rl_issue_417.md`, `moore_2021_rl_issue_630.md`, `labbe_2020_kbfp_07…` if renamed.

## Format

Papers, books, theses, standards or datasheets saved as text where an open PDF exists:

| ID | Current file | Open PDF |
|---|---|---|
| L4-S55 | zhang_2020_noise_covariance_identification.md | Open-access IEEE Access PDF via https://doi.org/10.1109/ACCESS.2020.2982407 ; also Europe PMC render https://europepmc.org/articles/PMC8638515?pdf=render |

No others. Labbe (S37/S38) has no PDF edition; REPs and robot_localization docs are `.rst` sources (correct per §3).

## Foundational

| SCOPE.md foundational reference | Status |
|---|---|
| Kalman 1960 | Downloaded, cited (S01) |
| Julier & Uhlmann 1997 / 2004 | Downloaded, cited (S02, S03) |
| Smith & Cheeseman 1986 | Downloaded, cited (S04) |
| Bar-Shalom, Li & Kirubarajan 2001 | Not downloaded, reason given (S05) |
| Thrun, Burgard & Fox 2005 | Not downloaded, reason given (S06) |
| Moore & Stouch 2014/2016 | Downloaded, cited (S07) |
| REP-103, REP-105 | Downloaded, cited (S08, S09) |
| robot_localization 3.5.4 docs/source/configs | Downloaded, cited (S15–S27) |
| Moore ROSCon 2015 | Downloaded, cited (S10) |
| Mehra 1970 | Not downloaded, reason given (S11) |
| Bar-Shalom 2002 OOSM | Not downloaded, reason given (S12) |
| Larsen et al. 1998 | Downloaded, cited (S13) |
| Huang, Mourikis & Roumeliotis 2010 | Downloaded, cited (S14) |
| Anderson & Moore 1979 | Downloaded, cited (S54) |
| Groves 2013 | **Missing from README** — marked not downloaded only in SCOPE.md |
| Julier & Uhlmann 1997 (CI, ACC) | **Missing from README** — not downloaded; CI covered via S56 |
| Fitzgerald 1971 | **Missing from README** — not downloaded; divergence covered via S54 |

## Coverage

- All 19 SCOPE.md subtopics have cited findings (subtopics 13–19 are folded into README §§1–12: Joseph form §2; divergence/whiteness §§5, 9, 10; CI §2; loose/tight coupling §4; integrity §§6–7; Autoware §§1, 5–8, 10–11; Q/R identification §§5, 9).
- Partly answered questions (already listed as README Open questions or visible gaps): 1(e) PAL and planetary-rover frames; 11(b) only Clearpath A200/Husky, no Jackal/Warthog; 11(c) no IGVC team reports cited; 12(c) no tested recipe for fusing a GNSS/INS unit's fused output; 5(e)/8(a) Mehra, ALS and Bar-Shalom OOSM only via secondary sources.
- **General first:** most sections open with general principles, but §3 (motion models), §4 (measurement modelling) and §7 (GNSS integration) open with robot_localization-specific findings before the general ones; §11 is product-configs by nature. Reorder §3/§4 so general principles (e.g. Dissanayake S46 vehicle constraints, Larsen/Anderson-style theory) come first.
- **Share of sources:** 33 general (53 %) vs 29 product-specific (47 %) of 62; of the 58 downloaded, 29 general / 29 product-specific (50 %/50 %). Foundational: 28 (S01–S27 and S54), supporting: 34.
