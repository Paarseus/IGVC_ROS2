# L2 — IMU and heading: source audit

| | |
|---|---|
| **Topic** | L2 — IMU and heading |
| **Date** | 2026-09-27 |
| **Reviewer** | Independent — sources (step 4 of STANDARDS.md §5) |
| **Inputs** | `README.md` (Sources table, Foundational references), `SCOPE.md`, `sources/` (45 files + a stray `.playwright-cli/` folder) |
| **Method** | `file` + `pdfinfo` + `pdftotext` first-page check on every PDF; header/body read of every text file; grep for error/login/bot-check markers; citation details checked against the file's own title page; GitHub API check of every cited commit (all four exist: xsens e145fb5 = 2026-09-25, robot_localization 8696ee5 = 2025-08-29, ardupilot_wiki 5365bb6 = 2026-09-27; REP repo HEAD = 11ca24a); Kalibr wiki file is byte-identical to the live wiki page. |

**Overall result:** 44 table rows (S24–S26 unused, as stated). 39 downloaded and all real content; 5 not downloaded, all foundational, all with a reason. No file fails the type or content check. Minor issues only: some level labels are inconsistent with STANDARDS §2, two Xsens documents are older revisions, a few citations lack page ranges or DOIs, some file names use the access year instead of the publication year, and there is a stray `.playwright-cli/` folder in `sources/`.

## Source quality

Legend: **F** = foundational (listed in SCOPE.md foundational table), **S** = supporting. **Gen** = general (applies across vehicles/products), **Prod** = product-specific.

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| L2-S01 | not downloaded | Artech House (textbook), 2013 | A | F | Gen | Yes (not downloaded) | Standard textbook; no open copy; correctly not cited for findings. |
| L2-S02 | not downloaded | IET (textbook), 2004, DOI given | A | F | Gen | Yes (not downloaded) | As above. |
| L2-S03 | not downloaded | McGraw-Hill (textbook), 2008 | A | F | Gen | Yes (not downloaded) | As above; link is the author's supplement page. |
| L2-S04 | not downloaded | IEEE standard | A | F | Gen | Yes (not downloaded) | Paywalled; definitions used only via L2-S07. |
| L2-S05 | not downloaded | IEEE Trans. Instrum. Meas., 2008, DOI | A | F | Gen | Yes (not downloaded) | SCOPE listed a ResearchGate author copy "check before use"; README says no reachable open copy. Acceptable; thesis L2-S07 substitutes. |
| L2-S06 | woodman_2007_intro_inertial_navigation.pdf | Univ. of Cambridge Computer Lab tech report UCAM-CL-TR-696 | A (labelled "not peer-reviewed") | F | Gen | Yes | Real 37-page report, citation correct. **Level inconsistent:** STANDARDS §2 level A is peer-reviewed/textbook/standard/spec; an un-refereed technical report does not fit A. Its foundational status is fine, but the level should be B/C or the README should state the exception explicitly. |
| L2-S07 | hou_2004_allan_variance_thesis.pdf | Univ. of Calgary MSc thesis, UCGE 20201 | A (examined thesis) | F (stand-in for S05) | Gen | Yes | Real 147-page thesis, citation correct. Theses are not named in the level table; A with "examined thesis" is applied consistently with S15. |
| L2-S08 | ethzasl_kalibr_wiki_imu_noise_model.md | ETH Zurich ASL, official Kalibr wiki, commit 73a2ba7 | B | S | Gen (noise model is tool-independent) | Yes | Official project docs; matches live page byte-for-byte. |
| L2-S09 | gebreegziabher_2006_magnetometer_calibration.pdf | ASCE J. Aerospace Eng. 19(2) 87–102 (author copy) | A | F | Gen | Yes | 45-page author copy; citation correct. Page numbers are author-copy pages, not journal pages (README's citation note covers this). |
| L2-S10 | vasconcelos_2011_geometric_magnetometer_calibration.pdf | IEEE TAES 47(2) 1293–1306 (author preprint) | A | F | Gen | Yes | Citation correct; peer-reviewed version identified. |
| L2-S11 | kok_2017_inertial_position_orientation.pdf | Found. Trends Signal Process. 11(1–2) 1–153, DOI (arXiv 1704.06053v2) | A | F | Gen | Yes | 90-page arXiv v2; published venue cited. |
| L2-S12 | kok_2016_magnetometer_calibration_inertial.pdf | IEEE Sensors J. 16(14) 5679–5689 (arXiv 1601.05257v3) | A | S | Gen | Yes | Venue confirmed on the file's cover page. DOI (10.1109/JSEN.2016.2569160) not given — minor. |
| L2-S13 | crassidis_2007_nonlinear_attitude_survey.pdf | AIAA J. Guidance, Control, and Dynamics 30(1) 12–28 (author copy) | A | S | Gen | Yes | Citation correct. |
| L2-S14 | dissanayake_2001_vehicle_model_constraints.pdf | IEEE Trans. Robotics and Automation 17(5) 731–747 | A | F | Gen | Yes | Published version; citation matches header. |
| L2-S15 | shin_2005_lowcost_ins_thesis.pdf | Univ. of Calgary PhD thesis, UCGE 20219 | A (examined thesis) | S | Gen | Yes | Real 206-page thesis. Most-cited source (35 citations). |
| L2-S16 | wahlstrom_2021_zero_velocity_review.pdf | IEEE Sensors J. 21(2) 1139–1151 (arXiv 2008.09208v1) | A | S | Gen | Yes | Peer-reviewed version identified. Topic is foot-mounted/pedestrian ZUPT; relevance to ground vehicles is partial but it is used for general ZUPT/ZARU principles. |
| L2-S17 | brossard_2020_ai_imu_dead_reckoning.pdf | IEEE Trans. Intelligent Vehicles 5(4) 585–595 (arXiv 1904.06064) | A | S | Gen | Yes | 10-page arXiv copy; peer-reviewed version cited. arXiv version number not given — minor. |
| L2-S18 | huang_2019_vins_concise_review.pdf | IEEE ICRA 2019, 9572–9582 (arXiv 1906.02650v1) | A | S | Gen | Yes | Peer-reviewed version identified. |
| L2-S19 | prikhodko_2013_mems_gyrocompassing.pdf | IEEE/ASME J. Microelectromech. Syst. 22(6) 1257–1266 | A | S | Gen | Yes | Published version; link is UC eScholarship. DOI (10.1109/JMEMS.2013.2282936) not given — minor. |
| L2-S20 | miao_2023_mems_gyrocompass_virtual_maytagging.pdf | Microsystems & Nanoengineering 9:138, DOI | A | S | Gen | Yes | Open-access published version. |
| L2-S21 | ublox_2023_zedf9p_moving_base_appnote.pdf | u-blox application note UBX-19009093 R03, 14-Sep-2023 | B | S | Prod (u-blox) | Yes | Manufacturer app note; date (2023) should be added to the citation. |
| L2-S22 | ublox_2024_zedf9h_datasheet.pdf | u-blox data sheet UBX-21025012 R05, 21-Mar-2024 | A | S | Prod (u-blox) | Yes | Doc number and revision match file. Link is the product page, not the PDF; add the date (2024). |
| L2-S23 | vectornav_2026_primer_{ahrs,gnss_ins,gnss_compass,heading_determination}.md | VectorNav, *Inertial Navigation Primer* (web) | B | S | Gen (vendor-neutral theory) | Yes, with note | Real page text with source URL and access date. Manufacturer educational pages are not documentation of the product in use; B is acceptable but they carry no version/date. Heavily relied on (20 citations). |
| L2-S27 | ros_rep103_units_coordinates.rst | ROS REP-103 (Active), commit 11ca24a | A | F | Gen | Yes | Original rst; authors correct. |
| L2-S28 | ros_rep145_imu_driver_conventions.rst | ROS REP-145 (**Draft**), commit 11ca24a | A (draft REP) | F | Gen | Yes, with note | Author Bovbel correct (SCOPE.md wrongly lists "T. Moore, A. Wallace"). A draft REP is not an adopted standard; "A (draft)" is flagged in the table, which is acceptable but B would be more consistent. |
| L2-S29 | robotlocalization_preparing_sensor_data.rst | robot_localization official docs, commit 8696ee5 | B | S | Prod (ROS package) | Yes | Pinned commit exists. Citation says "branch humble-devel" — the commit is the pin that matters. |
| L2-S30 | robotlocalization_navsat_transform_node.rst | robot_localization official docs, commit 8696ee5 | B | S | Prod (ROS package) | Yes | As above. |
| L2-S31 | xsens_2020_mti600_datasheet.pdf | Xsens datasheet MT1603P rev. 2020.B | A | F | Prod (Xsens) | Yes, with note | Real 44-page datasheet. Link is a **Farnell distributor mirror**; official URL exists: https://www.xsens.com/hubfs/Downloads/Leaflets/MTi%20600-series%20Datasheet.pdf. 2020.B appears to still be the latest standalone datasheet revision; current content lives at mtidocs.movella.com. |
| L2-S32 | xsens_2020_mti_family_reference_manual.pdf | Xsens MT1600P rev. 2020.A | A | F | Prod (Xsens) | Yes, with note | Real 35-page manual. **Currency:** 2020 revision; superseded content is maintained at mtidocs.movella.com — README should note that it was cross-checked against the 2023 user manual (S33) where they overlap. |
| L2-S33 | xsens_2023_mti600_user_manual.pdf | Xsens/Movella user manual, export 2023-10-31 | A | F | Prod (Xsens) | Yes | 117 pages; creation date confirmed by PDF metadata. Link is generic (mtidocs root). |
| L2-S34 | xsens_2019_magnetic_calibration_manual.pdf | Xsens MT0202P rev. O, Nov 2019 | A | F | Prod (Xsens) | Yes | Real 37-page manual. |
| L2-S35 | movella_2022_gnss_ins_supercharge_appnote.pdf | Movella application note MTAN001 rev. A | B | S | Prod (Xsens) | Yes | Manufacturer app note; somewhat promotional in title, but technical content. |
| L2-S36 | xsens_2026_kb_mti_filter_profiles.md | Xsens BASE KB (last published 2026-05-27) | B | S | Prod (Xsens) | Yes | Real page text. |
| L2-S37 | xsens_2026_kb_automotive_best_practices.md | Xsens BASE KB (last published 2022-06-30) | B | S | Prod (Xsens) | Yes | Real content. File name uses access year 2026, not publication year 2022. |
| L2-S38 | xsens_2026_kb_manual_gyro_bias_estimation.md | Xsens BASE KB (last published 2024-06-21) | B | S | Prod (Xsens) | Yes | File name year 2026 vs publication 2024. |
| L2-S39 | xsens_2026_kb_yaw_magnetically_disturbed.md | Xsens BASE KB (last published 2022-09-30) | B | S | Prod (Xsens) | Yes | Short (3 kB) but real content. File name year 2026 vs 2022. |
| L2-S40 | xsens_ros2driver_README.txt; xsens_ros2driver_xsens_mti_node.yaml; xsens_ros2driver_imupublisher.h | Movella official ROS 2 driver, commit e145fb5 (2026-09-25) | B | S | Prod (Xsens) | Yes | Raw files, pinned commit verified via GitHub API. Note: the local `src/xsens_mti` checkout is at c0a733f (older), so the cited commit is newer than what the robot runs. |
| L2-S41 | xsens_2022_kb_reference_frames_resets.md | Xsens BASE KB (last published 2022-01-11) | B | S | Prod (Xsens) | Yes | Real content. |
| L2-S42 | madgwick_2011_imu_marg_gradient_descent.pdf | IEEE ICORR 2011, pp. 179–185 | A | S | Gen | Yes, with note | Real 7-page proceedings paper. Link is a third-party mirror of the conference CD (vigir.missouri.edu); add DOI 10.1109/ICORR.2011.5975346 for traceability. |
| L2-S43 | wu_2012_ins_alignment_global_observability.pdf | IEEE TAES 48(1) 2012 (arXiv 1112.5282, accepted version) | A | S | Gen | Yes | Peer-reviewed venue identified. Page range (78–102) and DOI (10.1109/TAES.2012.6129622) missing — minor. |
| L2-S44 | ali_2005_mer_attitude_estimation.pdf | IEEE SMC 2005 (JPL copy) | A | S | Gen (rover practice) | Yes | Author list matches file. Page range/DOI missing — minor. |
| L2-S45 | ryu_2002_sideslip_gps_estimation.pdf | AVEC 2002 (Stanford copy) | A | S | Gen | Yes | Citation matches file. |
| L2-S46 | noaa_2026_wmm_accuracy_error_model.md | NOAA NCEI official web page (WMM2025) | B | S | Gen | Yes, with note | Real content (blackout zones, error model) but the first ~90 lines are site navigation and an iframe snippet — clean-up desirable, not a content failure. |
| L2-S47 | ardupilot_wiki_magnetic_interference.rst; ardupilot_wiki_compass_setup_advanced.rst | ArduPilot official wiki, commit 5365bb6 | B | S | Prod (ArduPilot autopilot) | Yes, with note | Official project docs, pinned. Relevance: drone-oriented; README flags this. |

### Evidence-level consistency
- Levels are mostly applied consistently: peer-reviewed papers A; manufacturer datasheets/manuals A; manufacturer app notes, KB pages, official project docs and code B; preprints raised to A only where a peer-reviewed version is named (S11, S12, S16, S17, S18, S43 — all correct).
- Inconsistencies: **S06** (non-peer-reviewed technical report labelled A); **S07/S15** (theses labelled A — not in the level table, but applied consistently); **S28** (Draft REP labelled A).
- No level C or D sources are used.

## Files

- `file` run on all 45 files. All 30 `.pdf` files are real PDFs (1.2–1.7) with real content on the first page; no HTML, error, login or bot-check pages. All 15 text files (`.md`, `.rst`, `.txt`, `.h`, `.yaml`) match their extension and contain the expected content.
- Suspicious-looking `file` page counts (e.g. "1 page", "3 pages") were checked with `pdfinfo`: the real counts are Brossard 10, Hou 147, Kok 2016 19, Kok 2017 90, Wahlström 13 — all complete documents.
- Every file is referenced in the Sources table (S23, S40, S47 each group several files); every table row has a file or is marked *not downloaded* (S01–S05).
- **Stray folder:** `sources/.playwright-cli/` (browser-automation logs from a failed MDPI download, HTTP 403) is not a source and should be deleted.
- File-naming: S37, S38, S39 use the access year (2026) instead of the publication year (2022, 2024, 2022) required by STANDARDS §3; S40/S29/S30/S27/S28/S47/S08 omit the year (acceptable for pinned repo files but inconsistent with the naming rule).

## Format

- No paper, book, thesis, standard or datasheet is saved as text where an open PDF exists. All papers, theses, datasheets and manuals are PDFs.
- Text-saved items are web documentation (VectorNav primer, Xsens KB, Kalibr wiki, NOAA page), original `.rst` sources (REPs, robot_localization, ArduPilot) and raw code/config — all correct per STANDARDS §3.
- Optional upgrade: L2-S46 cites the NOAA error-model web page; the peer-reviewed-equivalent primary document is the *US/UK World Magnetic Model for 2025–2030: Technical Report* (Chulliat/Brown et al., NOAA NCEI, 2025, doi:10.25923/prbc-s316), available as a PDF from https://www.ncei.noaa.gov/products/world-magnetic-model. Adding it would give page-citable declination-uncertainty numbers. Not a failure (the page itself is not a text copy of that report).

## Foundational

| SCOPE.md foundational reference | Status in README |
|---|---|
| Groves 2013 | S01, not downloaded (no open copy; ResearchGate bot-check) — reason given |
| Titterton & Weston 2004 | S02, not downloaded (no open copy) — reason given |
| Farrell 2008 | S03, not downloaded (no open copy) — reason given |
| Woodman 2007 | S06, downloaded, cited 13× |
| IEEE Std 952-1997 | S04, not downloaded (paywall) — reason given; used via S07 only |
| El-Sheimy, Hou, Niu 2008 | S05, not downloaded ("no reachable open copy"; SCOPE had flagged a ResearchGate copy) — reason given, S07 substitutes |
| Gebre-Egziabher et al. 2006 | S09, downloaded, cited 10× |
| Vasconcelos et al. 2011 | S10, downloaded, **cited only once** in findings |
| Kok, Hol, Schön 2017 | S11, downloaded, cited 10× |
| Dissanayake et al. 2001 | S14, downloaded, cited 3× |
| REP-145 / REP-103 | S28 / S27, downloaded, cited 4× / 3× |
| Xsens Family Ref. Manual, MTi-600 Datasheet / User Manual | S32, S31, S33 (+ S34), downloaded, cited 13×, 9×, 4× (+12×) |

All foundational references are either downloaded and cited, or marked *not downloaded* with a reason. SCOPE.md misattributes REP-145 to "T. Moore, A. Wallace, et al."; the README (Bovbel) is correct.

## Coverage

| SCOPE subtopic | README section | Cited? |
|---|---|---|
| 1 Heading fundamentals | §1 | Yes (S23, S27–S30, S32, S34, S45, S46) |
| 2 IMU error sources | §2 | Yes (S06, S07, S11, S19) |
| 3 Specs → filter noise | §3 | Yes (S06, S08, S15, S28, S29) |
| 4 AHRS algorithms | §4 | Yes (S11–S13, S15, S23, S32, S42, S44) |
| 5 Initial alignment | §5 | Yes (S15, S19, S23, S32, S43) |
| 6 Yaw observability GNSS/INS | §6 | Yes, partly — S14, S15, S23 general; S31, S36, S37 Xsens. Rhee/Tang/Jiang not obtainable (listed in Open questions) |
| 7 Standstill / slow / reverse | §7 | Yes (S14–S17, S33, S36–S38) |
| 8 Magnetometer | §8 | Yes (S09, S10, S12, S23, S32, S34, S39, S47) |
| 9 GNSS heading | §9 | Yes (S15, S21–S23, S45) |
| 10 Other heading sources | §10 | Yes (S14, S18–S21, S44) |
| 11 Failure modes / testing | §11 | Yes (S06, S08, S09, S12, S15, S23, S32, S34, S38, S43, S44) |
| 12 Xsens MTi-600 + ROS driver | §12 (last) | Yes (S31–S33, S35, S37, S38, S40, S41) |
| 13 Mounting alignment / resets | §1 subsection + §12 | Yes (S28, S29, S41) |
| 14 Bias models / consistency | §3 subsection | Yes (S15) — single source |
| 15 Earth-rate handling | §4 / §5 | Yes (S11, S15, S19, S44) |
| 16 Declination accuracy | §1 | Yes (S46) |
| 17 Heading vs course / time alignment | §1, §9 | Yes (S45) |
| 18 Global vs linearized observability | §5 | Yes (S43) |
| 19 Complementary AHRS filters | §4 | Partly — Madgwick (S42) only; Mahony 2008 not downloaded (bot-check/paywall), in Open questions |
| 20 Current-induced magnetometer interference | §8 | Partly — ArduPilot docs (S47) only, drone-oriented; no peer-reviewed or ground-robot source (Silic & Rogers 2019 listed in SCOPE, not obtained) |

**Ordering:** findings sections 1–11 are general and section 12 (product-specific) is last, as STANDARDS requires. General sections do lean on Xsens documents in places (e.g. §6 uses S31/S36/S37, §4 and §5 use S32), which is acceptable because general sources are cited first in each.

**General vs product-specific share:**
- All 44 table rows: 28 general (64 %), 16 product-specific (36 %) — product-specific = S21, S22 (u-blox), S29, S30 (robot_localization), S31–S41 (Xsens, 11), S47 (ArduPilot).
- The 39 downloaded sources: 23 general (59 %), 16 product-specific (41 %).

**Weakest coverage:** 6(c) GNSS/INS yaw-observability literature for land vehicles, 19 (Mahony / invariant EKF), 20 (current-induced magnetometer error on ground robots), and 14 (single source). All are acknowledged in README *Open questions*.
