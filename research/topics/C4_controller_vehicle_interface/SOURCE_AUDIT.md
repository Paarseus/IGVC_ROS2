# C4 — Controller–vehicle interface: source audit

| | |
|---|---|
| **Topic** | C4 — Controller–vehicle interface |
| **Date** | 2026-09-28 |
| **Reviewer** | independent — sources (step 4, STANDARDS.md sections 2–5) |
| **Scope** | README.md Sources table (C4-S01 … C4-S54), SCOPE.md, every file in `sources/` |

**Result.** 54 sources (51 downloaded, 3 marked *not downloaded*). **No source fails** the acceptance checklist. 28 sources need a correction (level consistency, unpinned code, missing citation details, preprint venue, file names). All 57 files are real content of the right type; every file maps to a table row and every row has a file or "not downloaded". No paper is saved as text, so no PDF upgrades are needed. All 16 SCOPE subtopics have cited findings.

Checklist columns: author/publisher, venue, traceability, relevance, currency, content. "Pass*" = passes but listed under *Needs correction*.

## Source-by-source

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| C4-S01 | williams_2018_it_mpc_tro.pdf | IEEE T-RO 34(6) 2018 (arXiv 1707.02342v1 file) | A | Foundational | General | Pass* | Real content, 20 pp. File is arXiv v1 manuscript; state that page numbers refer to the arXiv PDF |
| C4-S02 | not downloaded | IEEE ICRA 2016 | A | Foundational | General | Pass | No open copy after repeated search; not cited for details (correct handling) |
| C4-S03 | williams_2017_it_mpc_mbrl.pdf | IEEE ICRA 2017 (author copy, Boots homepage) | A | Foundational | General | Pass | Citation correct |
| C4-S04 | gandhi_2021_robust_mppi.pdf | IEEE RA-L 6(2) 2021 (arXiv accepted version) | A | Foundational | General | Pass | Citation correct |
| C4-S05 | rawlings_2017_mpc_textbook.pdf | Nob Hill Publishing, 2nd ed. (author-hosted, 821 pp.) | A | Foundational | General | Pass | Textbook, full copy |
| C4-S06 | coulter_1992_pure_pursuit.pdf | CMU Robotics Institute tech report CMU-RI-TR-92-01 | A | Foundational | General | Pass* | Tech report is not peer reviewed; A is not in the STANDARDS A list and is inconsistent with C4-S53 (tech report, C). Justify as standard reference or set consistently |
| C4-S07 | macenski_2023_regulated_pure_pursuit.pdf | Autonomous Robots 47:685–694, 2023 (arXiv) | A | Foundational | General (Nav2 algorithm) | Pass* | Add DOI 10.1007/s10514-023-10097-6; page numbers are arXiv's |
| C4-S08 | paden_2016_motion_planning_control_survey.pdf | IEEE T-IV 1(1) 2016 (arXiv) | A | Foundational | General | Pass | Citation correct |
| C4-S09 | not downloaded | Springer 2012 textbook | A | Foundational | General | Pass | No open copy; not cited for details |
| C4-S10 | not downloaded | IJRR 24(10) 2005 | A | Foundational | General | Pass | No verified open copy; method covered via C4-S19 |
| C4-S11 | ostafew_2016_learning_nmpc_path_tracking.pdf | J. Field Robotics 33(1) 2016 (Dynsyslab author copy) | A | Foundational | General | Pass | Four authors incl. Collier confirmed on p. 1 |
| C4-S12 | kayacan_2018_tracked_robots_traction.pdf | J. Field Robotics 35(7) 2018 (arXiv 2103.11294) | A | Foundational | General | Pass* | Add DOI; page numbers (e.g. p. 12, p. 17) are the arXiv PDF's, not JFR 1050–1062 — say so |
| C4-S13 | williams_2018_robust_sampling_mpc_tube.pdf | RSS XIV 2018 | A | Supporting | General | Pass | — |
| C4-S14 | snider_2009_automatic_steering_methods.pdf | CMU-RI-TR-09-08 tech report | A | Supporting | General | Pass* | Tech report, same level inconsistency as C4-S06 |
| C4-S15 | liniger_2015_autonomous_racing_rc_cars.pdf | Optim. Control Appl. Methods 36(5) 2015 (arXiv) | A | Supporting | General | Pass | — |
| C4-S16 | kalaria_2022_delay_aware_robust_control.pdf | IEEE IV 2022 (arXiv v3, Oct 2023) | A | Supporting | General | Pass* | File is arXiv v3, revised after the IV paper; note that pages/content follow v3 |
| C4-S17 | seegmiller_2013_vehicle_model_identification_ipem.pdf | IJRR 32(8) 2013 (CMU author copy) | A | Supporting | General | Pass | — |
| C4-S18 | seegmiller_2014_enhanced_3d_kinematics.pdf | RSS X 2014 | A | Supporting | General | Pass | — |
| C4-S19 | mandow_2007_skid_steer_experimental_kinematics.pdf | IEEE/RSJ IROS 2007 (author copy) | A | Supporting | General | Pass | — |
| C4-S20 | baril_2020_skid_steer_models_subarctic.pdf | CRV 2020 (arXiv) | A | Supporting | General | Pass* | Add proceedings pages and DOI (IEEE CRV 2020) |
| C4-S21 | samson_2025_drive_skid_steer_identification.pdf | IEEE Trans. Field Robotics 2, 2025 (arXiv 2506.16593) | A | Supporting | General | Pass | File is the arXiv template (DOI placeholder); page numbers are arXiv's |
| C4-S22 | kayacan_2015_tube_nmpc_tractor.pdf | IEEE/ASME TMECH 2015 (arXiv 2104.02063) | A | Supporting | General | Pass* | Citation incomplete: add vol. 20(1), pp. 447–456, DOI. Note the arXiv header's "vol. 23, pp. 197-205" is wrong |
| C4-S23 | hung_2023_path_following_review.pdf | J. Field Robotics 2023 (arXiv) | A | Supporting | General | Pass* | Add 40(3):747–779, doi:10.1002/rob.22142 |
| C4-S24 | goldfain_2019_autorally_platform.pdf | IEEE Control Systems Magazine 39(1) 2019 (arXiv) | A | Supporting | Product-specific (AutoRally) | Pass | — |
| C4-S25 | nav2_humble_mppi_readme.md | Nav2 official repo | B | Supporting | Product-specific | Pass* | Cited at moving `humble` branch; pin commit |
| C4-S26 | nav2_humble_mppi_optimizer.cpp; nav2_humble_mppi_motion_models.hpp | Nav2 official repo | **C → B** | Supporting | Product-specific | Pass* | Official project source code is level B per STANDARDS; branch not pinned |
| C4-S27 | nav2_main_mppi_readme.md | Nav2 official repo | B | Supporting | Product-specific | Pass* | `main` branch not pinned |
| C4-S28 | nav2_main_mppi_optimizer.cpp; nav2_main_mppi_motion_models.hpp | Nav2 official repo | **C → B** | Supporting | Product-specific | Pass* | Level should be B; `main` not pinned |
| C4-S29 | nav2_docs_configuring_mppic.md | docs.nav2.org source (rolling) | B | Supporting | Product-specific | Pass* | Pin docs commit; file name lacks year |
| C4-S30 | nav2_docs_migration_{iron_to_jazzy,jazzy_to_kilted,kilted_to_lyrical}.md | docs.nav2.org source | B | Supporting | Product-specific | Pass* | Pin commit; names lack year |
| C4-S31 | nav2_docs_velocity_smoother.md | docs.nav2.org source | B | Supporting | Product-specific | Pass* | Pin commit; name lacks year |
| C4-S32 | nav2_docs_configuring_regulated_pp.md | docs.nav2.org source | B | Supporting | Product-specific | Pass* | `file` says "HTML document" only because of inline tags; content is real markdown. Pin commit |
| C4-S33 | nav2_docs_tuning_guide.md | docs.nav2.org source | B | Supporting | Product-specific | Pass* | Pin commit |
| C4-S34 | nav2_humble_controller_server.cpp; nav2_humble_odom_subscriber.hpp | Nav2 official repo | **C → B** | Supporting | Product-specific | Pass* | Level should be B; cites "line 478" on a moving branch — pin commit so line numbers stay valid |
| C4-S35 | nav2_gh_5524.md | navigation2 GitHub issue | D | Supporting | Product-specific | Pass | Maintainer discussion |
| C4-S36 | nav2_gh_6065.md | navigation2 GitHub issue | D | Supporting | Product-specific | Pass* | Opening post is a user feature request; only the maintainer reply qualifies as D — cite maintainer statements only |
| C4-S37 | nav2_gh_6154.md | navigation2 GitHub PR | D | Supporting | Product-specific | Pass* | README says "merged"; saved file shows only "State: closed". Record the merge commit (feature confirmed by C4-S29/S30) |
| C4-S38 | autoware_mpc_lateral_controller_readme.md | Autoware Foundation official repo | B | Supporting | Product-specific | Pass* | `main` not pinned; name lacks year. `file` "SGML" is due to HTML comments, content is real |
| C4-S39 | ros2_controllers_diff_drive_params.yaml | ros2_controllers official repo | **C → B** | Supporting | Product-specific | Pass* | STANDARDS lists ros2_controllers source as B; `humble` not pinned |
| C4-S40 | clearpath_a200_husky_nav2.yaml; clearpath_j100_jackal_nav2.yaml | Clearpath Robotics repo | C | Supporting | Product-specific | Pass* | `humble` not pinned; names lack year |
| C4-S41 | ohnishi_2026_dynamic_window_pure_pursuit.pdf | arXiv 2601.15006 (preprint) | C | Supporting | General | Pass* | A peer-reviewed version exists: "Dynamic Window Pure Pursuit for Robot Path Tracking Considering Velocity and Acceleration Constraints," Proc. 19th Int. Conf. Intelligent Autonomous Systems (IAS-19), Genoa, 2025. Cite that venue; A if content matches, otherwise keep C for the extended arXiv content and say so |
| C4-S42 | nav2_gh_5617.md | navigation2 GitHub PR | D | Supporting | Product-specific | Pass* | "merged" not shown in saved file (State: closed); record merge commit |
| C4-S43 | helmick_2006_slip_compensated_path_following.pdf | Advanced Robotics 20(11) 2006 (JPL copy) | A | Supporting | General | Pass | — |
| C4-S44 | seiffer_2023_stanley_system_delay.pdf | MDPI Vehicles 5(2) 2023, DOI (KITopen) | A | Supporting | General | Pass | — |
| C4-S45 | kabzan_2019_learning_based_mpc_racing.pdf | IEEE RA-L 4(4) 2019 (ETH accepted version) | A | Supporting | General | Pass | — |
| C4-S46 | kim_2022_smooth_mppi.pdf | IEEE RA-L 7(4) 2022 (arXiv v8) | A | Supporting | General | Pass | — |
| C4-S47 | nav2_gh_3351.md | navigation2 GitHub issue (maintainer-authored) | D | Supporting | Product-specific | Pass | — |
| C4-S48 | nav2_gh_5266.md | navigation2 GitHub PR | D | Supporting | Product-specific | Pass* | README cites contributor (non-maintainer) reports (findings "big snake", ground-truth-odometry jitter); STANDARDS D covers maintainer answers only — label or drop those |
| C4-S49 | williams_2017_mppi_theory_parallel_computation.pdf | JGCD 40(2) 2017 cited; file is arXiv 1509.01149v3 (2015) | A | Foundational (added in README) | General | Pass* | File is a different, earlier preprint ("…using covariance variable importance sampling"), not an open copy of the JGCD article. Findings rest on the 2015 preprint: cite it as the source read (level C, or A only if content is confirmed identical) and rename file `williams_2015_mppi_covariance_importance_sampling.pdf` |
| C4-S50 | mckinnon_2019_learn_fast_forget_slow.pdf | IEEE RA-L 4(2) 2019 (author copy) | A | Supporting | General | Pass | — |
| C4-S51 | pravitra_2020_l1_adaptive_mppi.pdf | IEEE/RSJ IROS 2020 (arXiv) | A | Supporting | General | Pass | Multirotor; vehicle type stated |
| C4-S52 | pannocchia_2015_offset_free_tracking_review.pdf | ECC 2015 (Pisa repository) | A | Supporting | General | Pass | — |
| C4-S53 | kuntz_2024_offset_free_mpc_mismatch.pdf | TWCCC tech report / arXiv 2412.08104v2 | C | Supporting | General | Pass | Checked: submitted to IEEE TAC, listed "in progress" on the Rawlings group page; no published version, C is correct |
| C4-S54 | song_2026_icode_mppi_residual_learning.pdf | arXiv 2605.03260v1 (preprint, simulation only) | C | Supporting | General | Pass | No peer-reviewed version found; C correct and flagged as preprint in README |

### Level consistency
- **Official project source code rated C** (C4-S26, S28, S34, S39) while the official READMEs/docs of the same projects are rated B. STANDARDS section 2 puts "ros2_controllers, robot_localization source" and official-project source code at **B**. Change all four to B.
- **Technical reports**: Coulter (S06) and Snider (S14) are rated A; Kuntz & Rawlings (S53) is rated C. None is peer reviewed. Use one rule for all three, and record it.
- **Preprint pagination**: S01, S07, S08, S12, S15, S20–S24, S46 are cited with page numbers from the arXiv PDF but without saying so (S45, S49, S51, S52 do say so). Add the note to each row.

## Files
- `file` run on all 57 files in `sources/`: 33 PDFs are "PDF document"; `pdfinfo` page counts (6–821 pp.) and first-page text match each cited paper. The `file` page counts for some PDFs (e.g. 1 page for C4-S49, 2 for C4-S12) are wrong because of the PDF page tree; the real counts are 9 and 20.
- Text files: `.md` files are real markdown (some flagged by `file` as HTML/SGML/LaTeX because of inline HTML comments or `$` signs: autoware README, nav2_docs_configuring_regulated_pp.md, nav2_gh_3351/5266/5617/6154). `.cpp/.hpp` are C/C++ source, `.yaml` are ASCII configs. No error, login or bot-check pages (grep for html/captcha/access denied/sign-in: no hits).
- Every file appears in the Sources table; every table row has a file or "not downloaded" (S02, S09, S10).
- File-name convention (`<author-or-org>_<year>_<short-topic>`) not followed by 23 files: all `nav2_*`, `autoware_*`, `clearpath_*`, `ros2_controllers_*` lack a year. `williams_2017_mppi_theory_parallel_computation.pdf` holds the 2015 preprint with a different title; `ohnishi_2026_*` has a 2025 peer-reviewed version.

## Format
- All papers, textbooks and tech reports are saved as PDF. No paper, book, thesis, standard or datasheet is saved as text. **PDF upgrades needed: 0.**
- Web documentation and GitHub discussions are saved as markdown, code and configs as raw files, as STANDARDS section 3 requires.

## Foundational
- SCOPE.md lists 12 foundational references. 9 are downloaded and cited (S01, S03–S08, S11, S12). 3 are marked *not downloaded* with the reason (S02 Williams ICRA 2016, S09 Rajamani 2012, S10 Martínez IJRR 2005) and are not cited for details. This complies with STANDARDS section 3.
- README adds C4-S49 (Williams et al. JGCD 2017) as foundational. It is not in SCOPE.md's foundational table, and the downloaded file is the 2015 preprint, not the JGCD article (see S49 above).

## Coverage
| Subtopic (SCOPE.md) | Cited findings in README | Main sources |
|---|---|---|
| 1 MPPI formulation | Yes (§1, 18 bullets) | S01, S49, S04, S46 |
| 2 Other trackers | Yes (§2) | S06, S07, S14, S41, S44 |
| 3 Motion models | Yes (§3) | S17, S18, S24, S45, S23 |
| 4 Identification | Yes (§4) | S17, S21, S20, S45, S46 |
| 5 Limits vs chassis | Yes (§5, 8 bullets) | S41, S40, S30 |
| 6 Delay, lag, under-delivery | Yes (§6, 31 bullets) | S16, S17, S15, S50, S51, S52, S05 |
| 7 Velocity feedback | Yes (§7) | S34, S35, S39, S48 (mostly B–D) |
| 8 Slip | Yes (§8) | S12, S11, S20, S22, S43 |
| 9 Robustness | Yes (§9, 7 bullets) | S04, S13, S16, S01 |
| 10 Testing and metrics | Yes (§10, 5 bullets; sim-to-real part thin) | S41, S07, S01, S14 |
| 11 Product specifics | Yes (§11, last) | S25–S40 |
| 12 Command smoothness | Yes (§1, Common mistakes) | S46, S47 |
| 13 Delay magnitudes / geometric trackers | Yes (§2, §6, Key numbers) | S44 |
| 14 Slip detection with independent sensing | Yes (§8) | S43 |
| 15 Online residual learning | Yes (§3, §4) | S45, S50, S54 |
| 16 Noisy vs delayed feedback | Yes but only level D (§7) | S48, S35 |

- **General first:** yes. Sections 1–10 are general theory and cross-vehicle studies (racing cars, tractors, planetary rovers, tracked field robots, multirotor); product material is confined to §11 and Key numbers rows marked as such.
- **Share:** of 54 sources, 34 general (63%) and 20 product-specific (37%: S24–S40, S42, S47, S48). Of the 51 downloaded, 31 general (61%), 20 product-specific (39%).
- **Weak spots:** subtopic 16 rests only on GitHub discussion (level D, partly contributor reports); subtopic 7 relies on B–D Nav2 material with no peer-reviewed source on noisy-feedback effects in MPPI (the README records this as an open question).

## Lists
**Failing (remove):** none.

**Needs correction:** S01, S06, S07, S12, S14, S16, S20, S22, S23, S25–S34, S36, S37, S38, S39, S40, S41, S42, S48, S49 (details in the table above).
