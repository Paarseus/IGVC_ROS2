# L1 — Wheel odometry: source audit

| | |
|---|---|
| **Topic** | L1 — Wheel odometry |
| **Date** | 2026-09-27 |
| **Reviewer** | independent — sources (STANDARDS.md §5 step 4) |
| **Scope of audit** | README.md Sources table (L1-S01…S42 + "not downloaded" table), SCOPE.md foundational list and subtopics, every file in `sources/` |

Summary: 42 source IDs, 47 files. All files are real content of the declared type; every file maps to a table row and every row has a file. No paper is saved as text. Two citation errors (S01 issue number, S20 missing peer-reviewed venue), one evidence-level inconsistency (technical reports S03/S05/S06), and minor metadata gaps (S02, S11, S41). All foundational references are downloaded or listed as not downloaded with a reason.

## Source grading

Level column: level as given in README → auditor's view where it differs. "Foundational" = listed in SCOPE.md's foundational table (F) or supporting (S). "General" = applies across vehicles/software; "ROS-general" = official ROS framework material (general to any ROS robot); "product" = one vendor's product.

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| L1-S01 | borenstein_1996_correction_systematic_odometry_errors.pdf | IEEE T-RA (author preprint, 26 pp.) | A | F | General | Yes, citation fix needed | Real paper. **Citation error:** published in *IEEE T-RA* **12(6)**:869–880, Dec. 1996 (DOI 10.1109/70.544770), not 12(5). The author copy's own header says "Vol 12, No 5, October 1996", which is the source of the error; SCOPE.md repeats it. |
| L1-S02 | borenstein_1995_umbmark.pdf | Proc. SPIE Mobile Robots X, 1995 (author copy, 12 pp.) | A | F | General | Yes, metadata incomplete | Real paper. Citation lacks SPIE volume (2591), pages and DOI; add them. |
| L1-S03 | borenstein_1996_where_am_i.pdf | Univ. of Michigan tech. report for ORNL/DOE, 1996 (282 pp.) | A → see note | F | General | Yes | Real, complete report. Not peer-reviewed; STANDARDS §2 has no row for technical reports. Graded A here but S06 (same kind of document) is graded B — inconsistent (see Level consistency). |
| L1-S04 | kelly_2004_linearized_error_propagation_odometry.pdf | IJRR 23(2):179–218, 2004 (SAGE, 41 pp.) | A | F | General | Yes | Citation, DOI 10.1177/0278364904041326 correct. |
| L1-S05 | chong_1996_accurate_odometry_error_modelling.pdf | Monash tech. report MECSE-1996-6 (peer-reviewed as ICRA 1997, pp. 2783–2788) | A | F | General | Yes | A is defensible only because a peer-reviewed ICRA 1997 version exists and is cited; the file is the tech report, so page cites refer to it. |
| L1-S06 | kleeman_1995_odometry_error_covariance.pdf | Monash tech. report MECSE-95-1, 1995 | B → inconsistent | F | General | Yes | Real content. Level B is meant for manufacturer/official-project docs; a university tech report does not fit B. Treat tech reports consistently with S03/S05 (see below). |
| L1-S07 | siegwart_2004_amr_ch5_localization.pdf | MIT Press 2004, 1st ed., ch. 5 (author-distributed chapter, printed pp. 159–230) | A | F (1st ed. substitute for 2nd ed.) | General | Yes, with caveat | Superseded edition, but stated as a substitute for the 2nd ed. (not downloadable), which the currency rule allows. Footer "R. Siegwart, EPFL, Illah Nourbakhsh, CMU" suggests a pre-print draft of the chapter; page numbers may differ slightly from the printed book. |
| L1-S08 | censi_2013_simultaneous_calibration_odometry_sensor.pdf | IEEE T-RO 29(2):475–492, 2013 (author copy, 13 pp., PDF dated 2011) | A | F | General | Yes | Citation and DOI 10.1109/TRO.2012.2226380 correct. File is the accepted-manuscript draft, not the published layout. SCOPE said "no verified open copy"; one was found — good. |
| L1-S09 | rep_2010_0105_coordinate_frames.rst | ROS REP (official standard), 2010, Active | A | F | ROS-general | Yes | Original .rst source; author W. Meeussen correct. |
| L1-S10 | rep_2010_0103_units_conventions.rst | ROS REP, 2010, Active | A | S | ROS-general | Yes | Authors T. Foote, M. Purvis correct. |
| L1-S11 | ros2_2022_nav_msgs_odometry.msg | ros2/common_interfaces, pinned commit 08434490 | B | F | ROS-general | Yes, minor | Real message file. File-name year "2022" does not match the citation (file last changed 2020); cosmetic. |
| L1-S12 | martinelli_2003_estimating_odometry_error_navigation.pdf | ECMR 2003 (EPFL Infoscience) | A | S (substitute for Martinelli 2007) | General | Yes | Real paper, peer-reviewed conference. |
| L1-S13 | doh_2006_relative_localization_path_odometry.pdf | Autonomous Robots 21:143–154, 2006 (Springer) | A | S | General | Yes | DOI matches first page. |
| L1-S14 | seegmiller_2013_vehicle_model_identification_ipem.pdf | IJRR 32(8):912–931, 2013 (author preprint) | A | S | General | Yes | Citation correct; preprint of a published journal paper. |
| L1-S15 | borenstein_1996_gyrodometry.pdf | IEEE ICRA 1996, pp. 423–428 | A | S | General | Yes | First page confirms venue/pages. |
| L1-S16 | ojeda_2004_odometry_errors_over_constrained.pdf | Autonomous Robots 16:273–286, 2004 (author copy) | A | S | General | Yes | Header confirms. |
| L1-S17 | ojeda_2006_current_based_slippage_detection.pdf | IEEE T-RO 22(2):366–378, 2006 | A | S | General | Yes | Publisher layout; citation correct. |
| L1-S18 | reina_2006_wheel_slippage_sinkage_detection.pdf | IEEE/ASME T-Mech 11(2):185–195, 2006 | A | F | General | Yes | Publisher layout; citation correct. |
| L1-S19 | mandow_2007_experimental_kinematics_skid_steer.pdf | IEEE/RSJ IROS 2007, pp. 1222–1227 | A | S | General | Yes | Link is a download-logger URL, but DOI 10.1109/IROS.2007.4399139 gives traceability. |
| L1-S20 | baril_2020_skid_steer_kinematic_models.pdf | CRV 2020 (file is arXiv:2004.05131v1) | A | S | General | Yes, citation fix needed | Preprint with a peer-reviewed version, so A is correct, but the citation must give the venue details: *2020 17th Conf. on Computer and Robot Vision (CRV)*, pp. 198–205, DOI 10.1109/CRV50864.2020.00034. Published-version PDF is open at https://norlab.ulaval.ca/pdf/Baril2020.pdf (optional replacement). |
| L1-S21 | endo_2007_tracked_slip_compensating_odometry.pdf | IEEE/RSJ IROS 2007 (author copy) | A | S | General | Yes | Citation lacks page numbers; DOI given. |
| L1-S22 | seegmiller_2014_enhanced_3d_kinematic_modeling.pdf | RSS X, 2014 | A | S | General | Yes | Official proceedings PDF. |
| L1-S23 | maimone_2007_two_years_visual_odometry_mer.pdf | J. Field Robotics 24(3):169–186, 2007 (author copy) | A | Supporting (not in SCOPE) | General (planetary rover) | Yes | Relevant for wheel-odometry error magnitudes on rovers (subtopic 7d). |
| L1-S24 | merry_2010_encoder_velocity_estimation.pdf | Mechatronics 20:20–26, 2010 (Elsevier) | A | Supporting | General | Yes | Publisher layout confirms. |
| L1-S25 | petrella_2007_speed_measurement_low_resolution_encoder.pdf | ACEMP 2007, pp. 780–787 (IEEE) | A | S | General | Yes | Real paper. |
| L1-S26 | robotlocalization_humble_8696ee5_preparing_sensor_data.rst | robot_localization official docs, pinned commit | B | S | ROS-general | Yes | Original .rst. |
| L1-S27 | robotlocalization_humble_8696ee5_configuring.rst | robot_localization official docs, pinned commit | B | Supporting | ROS-general | Yes | Original .rst. |
| L1-S28 | nav2docs_588d374_setup_odom.md | Nav2 official docs, pinned commit | B | S | ROS-general | Yes | Original markdown source. |
| L1-S29 | nav2docs_588d374_odometry_calibration_bt.md | Nav2 official docs, pinned commit | B | Supporting | ROS-general | Yes | Short (1.5 kB) but real page. |
| L1-S30 | ros2controllers_2_54_0_diff_drive_{odometry,controller}.cpp, _parameters.yaml, _userdoc.rst | ros2_controllers official repo, tag 2.54.0 | B | S | ROS-general | Yes | Four raw files at a pinned tag. |
| L1-S31 | ros2controllers_master_2520ae5_diff_drive_odometry.cpp | ros2_controllers, pinned master commit 2520ae5 | B | Supporting | ROS-general | Yes | Pinned commit (not a moving branch). |
| L1-S32 | clearpath_5c5ec97_{a200_husky,j100_jackal}_control.yaml | Clearpath Robotics clearpath_common, pinned commit | C | Supporting | Product-specific (Husky, Jackal) | Yes | Established open-source configs; C is correct. |
| L1-S33 | rev_2026_revlib_EncoderConfig.java | REV Robotics REVLib 2026.0.5 sources jar | B | — | Product-specific (REV SPARK MAX) | Yes | Manufacturer source at a pinned version. |
| L1-S34 | wpilib_2022_sysid_issue258_rev_hall_latency.md | GitHub issue comment, wpilibsuite/sysid #258 | D | — | Product-specific (REV NEO) | Yes, borderline | Author is a WPILib SysId contributor quoting a REV Support reply, not a maintainer answer in the strict sense; acceptable as D only because no A–C source for the NEO hall-sensor sample period was found, and the comment itself says the figure "may be subject to change". Keep flagged as D. |
| L1-S35 | ctre_2022_phoenix5_sensor_velocity.md | CTR Electronics Phoenix 5 official docs ("stable" URL) | B | — | Product-specific (CTRE Talon) | Yes, with caveat | Legacy (Phoenix 5) docs; currency exception is stated ("legacy framework; comparison only"). Date "c. 2022" is an estimate and the `stable` URL is not version-pinned. |
| L1-S36 | long_2012_banana_distribution_gaussian.pdf | RSS VIII, 2012 | A | Supporting (gap check) | General | Yes | Official proceedings PDF. |
| L1-S37 | nav2amcl_1_1_18_motion_model_differential.cpp | Nav2 official repo, tag 1.1.18 | B | Supporting (Thrun substitute) | ROS-general | Yes | Pinned tag. |
| L1-S38 | nav2docs_588d374_configuring_amcl.md | Nav2 official docs, pinned commit | B | Supporting | ROS-general | Yes | Real page. |
| L1-S39 | sturm_2012_tum_rgbd_benchmark.pdf | IEEE/RSJ IROS 2012, pp. 573–580 | A | Supporting | General | Yes | Author PDF; citation correct. |
| L1-S40 | geiger_2012_kitti_benchmark.pdf | IEEE CVPR 2012, pp. 3354–3361 | A | Supporting | General | Yes | Author PDF; citation correct. |
| L1-S41 | ward_2007_classification_wheel_slip_detection.pdf | IEEE ICRA 2007 (conference-CD copy, session ThC11.2) | A | Supporting | General | Yes, minor | Real 6-page paper (`file` reports "0 page(s)", pdfinfo reports 6 — a `file` quirk, not a defect). IEEE Xplore record is document 4209496; pages 2730–2735 and DOI 10.1109/ROBOT.2007.363878 could not be independently confirmed in this audit — check against Xplore. |
| L1-S42 | kuemmerle_2012_simultaneous_parameter_calibration.pdf | Advanced Robotics 26(17):2021–2041, 2012 (accepted manuscript) | A | Supporting (named in SCOPE 4c) | General | Yes | Author manuscript from Freiburg; citation correct. |

### Level consistency
- Levels are consistent for journal/conference papers (A), official ROS/Nav2/ros2_controllers/REV material (B), Clearpath configs (C) and the GitHub comment (D).
- **Inconsistent: university technical reports.** S03 (Where am I?, A), S05 (Chong & Kleeman tech report, A) and S06 (Kleeman tech report, B). STANDARDS §2 has no tech-report row; B is for official product/project documentation. Recommendation: S05 stays A because its peer-reviewed ICRA 1997 version is cited; S03 and S06 should share one level (A as a standard survey/reference work from a named university lab, or C), and the README should say which rule was applied.
- Preprint rule (§2): S20 is an arXiv preprint with a peer-reviewed CRV 2020 version → A is correct, but the venue details must be added. Other author copies (S01, S08, S14, S16, S23, S42) are copies of published papers, not unpublished preprints → A correct.

## Files

`file` was run on all 47 files in `sources/`.

- All 27 `.pdf` files are `PDF document`; pdfinfo and first-page text confirm real content matching the citation (no error, login or bot-check pages). `file` reports "0 page(s)" for `ward_2007_classification_wheel_slip_detection.pdf` and no page count for `borenstein_1996_where_am_i.pdf` and `censi_2013_…pdf`; pdfinfo shows 6, 282 and 13 pages with real text. **Pass.**
- `.md` files (ctre, 3× nav2docs, wpilib): text; heads show the real page content with source comments; grep for 403/forbidden/captcha/sign-in/"just a moment" found none. `file` labels two of them "exported SGML document" because they begin with an HTML comment — expected, not a defect. **Pass.**
- `.rst` (REP-103, REP-105, 2× robot_localization, ros2_controllers userdoc), `.msg`, `.yaml` (3), `.cpp` (4), `.java` (1): text/source types matching extensions; content is the real file. **Pass.**
- Mapping: every file is referenced in the Sources table (S30 covers 4 files, S32 covers 2), and every row S01–S42 has a file. The 8 "not downloaded" rows have no file, as expected. **Pass.**
- Naming: `ros2_2022_nav_msgs_odometry.msg` year does not match the cited 2020 file change / pinned commit (cosmetic).

## Citations

| ID | Issue | Correct form |
|---|---|---|
| S01 (and SCOPE.md) | Wrong issue number | *IEEE Trans. Robotics and Automation* **12(6)**:869–880, Dec. 1996, DOI 10.1109/70.544770 |
| S20 | Preprint cited without peer-reviewed venue details | *Proc. 17th Conf. on Computer and Robot Vision (CRV)*, pp. 198–205, 2020, DOI 10.1109/CRV50864.2020.00034 (arXiv:2004.05131 as the open copy) |
| S02 | Missing volume/pages/DOI | Add Proc. SPIE vol. 2591 (Mobile Robots X), page range and DOI |
| S21 | Missing pages | Add IROS 2007 page range |
| S41 | Pages/DOI not confirmed | Confirm pp. 2730–2735 and DOI against IEEE Xplore document 4209496 |
| S35 | Date estimated, URL on moving "stable" docs | State "accessed 2026-09-27, undated page" |
| S11 | File-name year | Rename or note (cosmetic) |

All other authors, titles, years, venues, links and pinned versions/commits checked against the file headers and are correct. Code sources are all pinned (tags 2.54.0, 1.1.18, REVLib 2026.0.5; commits 8696ee5, 588d374, 2520ae5, 5c5ec97, 08434490).

## Format

- No paper, book, thesis, standard or datasheet is saved as text: every paper/book/report is a PDF. REPs are the original `.rst` sources (accepted by §3 "documentation stored as source files"). **Nothing to upgrade to PDF.**
- Optional version upgrade: S20 is the arXiv v1 file; the published CRV version is open at https://norlab.ulaval.ca/pdf/Baril2020.pdf.

## Foundational

Every foundational reference in SCOPE.md is accounted for:

| SCOPE foundational reference | Status |
|---|---|
| Borenstein & Feng 1996 T-RA / UMBmark 1995 | Downloaded — S01, S02 (fix S01 issue number) |
| Borenstein, Everett & Feng, *Where am I?* 1996 | Downloaded — S03 |
| Wang 1988 ICRA | Not downloaded, reason given |
| Chong & Kleeman 1997 / Kleeman 1995 | Downloaded — S05, S06 |
| Kelly 2004 IJRR | Downloaded — S04 |
| Antonelli et al. 2005 T-RO | Not downloaded, reason given |
| Martinelli et al. 2007 Auton. Robots | Not downloaded, reason given (ECMR 2003 S12 used) |
| Censi et al. 2013 T-RO | Downloaded — S08 |
| Siegwart et al. 2nd ed. 2011 | Not downloaded, reason given (1st ed. ch. 5 = S07) |
| Thrun et al. 2005 | Not downloaded, reason given (Nav2 AMCL S37/S38 used) |
| Martínez et al. 2005 IJRR | Not downloaded, reason given |
| Reina et al. 2006 T-Mech | Downloaded — S18 |
| REP-105 / REP-103 / nav_msgs/Odometry | Downloaded — S09, S10, S11 |

**Pass.** Supporting items named in SCOPE but neither downloaded nor listed as not downloaded: Jung & Chung 2011 heading-error calibration; Kelly, *Mobile Robotics* (Cambridge UP, 2013); Iagnemma & Dubowsky, *Mobile Robots in Rough Terrain* (Springer, 2004). They are not foundational, so this is not a failure, but the README should list them in "not downloaded" or say they were dropped. Pentzer 2014 and Ward & Iagnemma model-based 2007 are listed with reasons.

## Coverage

| SCOPE subtopic | README section | Cited findings | Notes |
|---|---|---|---|
| 1 Dead reckoning / integration | §1 | 9 | 1(b) integration-error size and 1(c) slopes/3D are Open questions |
| 2 Error taxonomy | §2 | 12 | Covered |
| 3 Error modelling / covariance | §3 | 22 | Covered (incl. 12, 13) |
| 4 Calibration | §4 | 21 | Covered (incl. 15) |
| 5 Encoder / velocity / timing | §5 + product section | 12 (+ product) | 5(c) quantified lag effect and 5(d) timestamping/wrap-around are Open questions — no source |
| 6 Skid-steer / tracked | §6 | 14 | 6(d) forward vs inverse params is an Open question |
| 7 Error magnitudes | §7 | 11 | 7(d) agricultural figures is an Open question |
| 8 Slip detection | §8 | 16 | Covered (incl. 16 via S41) |
| 9 Testing / evaluation | §9 | 13 | Covered (incl. 14 via S39, S40) |
| 10 ROS reporting | §10 | 11 | Covered |
| 11 Reference implementations | §11 | 6 | Covered |
| 12 Probabilistic motion model (AMCL) | §3 (S37, S38) | yes | Covered |
| 13 Non-Gaussian uncertainty | §3 (S36) | yes | Covered |
| 14 Trajectory metrics | §9 (S39, S40) | yes | Covered |
| 15 Parameter change / online calibration | §4 (S42) | yes | Covered |
| 16 Immobilisation detection | §8 (S41) | yes | Covered |

- Every subtopic has cited findings; every finding bullet in §1–§11 carries an `[L1-Sxx]` citation. Uncovered sub-questions (1b, 1c, 5c, 5d, 6d, 7d) are honestly recorded under Open questions.
- **General first:** yes — §1–§11 are general; product-specific material (REV SPARK MAX/NEO) sits in its own section after them.
- **Share:** of 42 source IDs, 4 are product-specific (S32 Clearpath, S33 REVLib, S34 REV/WPILib comment, S35 CTRE) = **10 %**; 38 general = **90 %** (of which 12 are ROS-framework official material: S09–S11, S26–S31, S37, S38, and 26 are vendor-neutral papers/books/reports).
