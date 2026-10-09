# L5 — Time synchronization: source audit

| | |
|---|---|
| **Topic** | L5 — Time synchronization |
| **Date** | 2026-09-28 |
| **Reviewer** | independent — sources |
| **Scope** | README.md Sources table (L5-S01 … L5-S67), `sources/` (91 files), SCOPE.md foundational list and subtopics; rules from STANDARDS.md §2–5 |

**Result:** 67 source IDs (62 downloaded, 5 marked *not downloaded*). **0 fail** the acceptance checklist. **13 need a correction** (citation details, level consistency, version or file). All 91 files are real content of the right type.

Key: F = foundational, S = supporting; G = general, P = product-specific.

## Source table

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| L5-S01 | mills_1991_ntp_internet_time_sync.pdf | IEEE Trans. Communications (author reprint) | A | F | G | Yes — needs correction | Real 14-page PDF. Add DOI 10.1109/26.103043 to the Sources row (it is in SCOPE but not the table). |
| L5-S02 | ietf_2010_rfc5905_ntpv4.txt | IETF RFC (Standards Track) | A | F | G | Yes | Canonical RFC text; authors/year correct. |
| L5-S03 | mills_1998_hybrid_clock_discipline.pdf | IEEE/ACM Trans. Networking (author copy) | A | F | G | Yes — needs correction | Real PDF. Add DOI 10.1109/90.731187. |
| L5-S04 | lamport_1978_time_clocks_ordering.pdf | Comm. ACM (third-party mirror, worrydream.com) | A | F | G | Yes — needs correction | Scanned, no text layer; hosted on a personal mirror. Add DOI 10.1145/359545.359563 and replace with the author-hosted copy https://lamport.azurewebsites.net/pubs/time-clocks.pdf (checked: 200, application/pdf). |
| L5-S05 | eidson_2005_ieee1588_tutorial.pdf | NIST-hosted tutorial by 1588 committee chair (Agilent) | B | F (proxy for S50) | G | Yes | 94-page slide deck; correct level (not the standard itself). |
| L5-S06 | ietf_2000_rfc2783_pps_api.txt | IETF RFC (Informational) | A | F | G | Yes | Authors correct; RFC is Informational, but it is the de-facto Linux PPS interface spec — A acceptable. |
| L5-S07 | nist_1990_tn1337_clocks_oscillators.pdf | NIST Technical Note | A | F | G | Yes | 357-page scan (no text layer); `file` header count "11 pages" is an artefact, pdfinfo gives 357. |
| L5-S08 | riley_2008_nist_sp1065_frequency_stability.pdf | NIST Special Publication | A | S | G | Yes | Real 136-page PDF. |
| L5-S09 | larsen_1998_delayed_measurements_kf.pdf | IEEE CDC (DTU Orbit copy) | A | F | G | Yes | Citation and DOI correct. |
| L5-S10 | skog_2007_gnss_ins_licentiate_summary.pdf | KTH licentiate thesis (DIVA) | A | F (partial stand-in for S52) | G | Yes | Only the thesis summary; findings may use only what the summary and Paper E abstract say. Level A accepted as an examined academic thesis. |
| L5-S11 | olson_2010_passive_sensor_sync.pdf | IEEE/RSJ IROS | A | F | G | Yes | Correct citation and DOI. |
| L5-S12 | furgale_2013_unified_temporal_spatial_calib.pdf | IEEE/RSJ IROS | A | F | G | Yes | Correct citation and DOI. |
| L5-S13 | ros2_design_clock_and_time.md | ROS 2 design site (official) | B | F | G | Yes | Real page text. Design article has no date/version; acceptable. |
| L5-S14 | jellum_2022_syncline.pdf | IEEE CCTA 2022; arXiv:2209.01136v2 | A | S | G | Yes | Peer-reviewed venue exists, so A is correct. Note the saved v2 is dated 20 Jan 2024 (after the conference); page numbers refer to the arXiv copy. |
| L5-S15 | brouk_2024_kf_asynchronous_epochs.pdf | ION NAVIGATION journal (CC-BY) | A | S | G | Yes | Authors Brouk & DeMars confirmed in file; DOI correct. |
| L5-S16 | mauthner_2006_oosm_buffering_vs_algorithms.pdf | Workshop Fahrerassistenzsysteme (FAS) 2006, author version | A | S | G | Yes — needs correction | File confirms venue and pp. 20–30 (Löwenstein/Hößlinsülz). Peer-review status of this national workshop is not stated; confirm it, otherwise level C. |
| L5-S17 | harrison_2011_ticsync.pdf | IEEE ICRA | A | S | G | Yes | Correct. |
| L5-S18 | qin_2018_online_temporal_calib_vio.pdf | IEEE/RSJ IROS 2018; arXiv copy | A | S | G | Yes | Peer-reviewed version exists (IROS 2018, pp. 3662–3669). |
| L5-S19 | kelly_2021_question_of_time_temporal_calib.pdf | IEEE MFI 2021; arXiv v3 | A | S | G | Yes | Peer-reviewed version exists. |
| L5-S20 | tschopp_2020_versavis.pdf | MDPI Sensors 20(5):1439, 2020; arXiv v1 (Dec 2019) saved | A | S | G | Yes — needs correction | The saved file is the pre-review arXiv v1, although the peer-reviewed article is open access. Add DOI 10.3390/s20051439 and replace with the MDPI PDF (https://www.mdpi.com/1424-8220/20/5/1439/pdf — bot check blocked curl, fetch via browser) and re-check the page numbers cited in findings. |
| L5-S21 | geiger_2013_kitti_dataset.pdf | IJRR (author copy) | A | S | G | Yes | Correct. |
| L5-S22 | bedard_2022_ros2_tracing.pdf | IEEE RA-L 7(3), 2022; arXiv v4 | A | S | G | Yes — needs correction | Add pages (6511–6518) and DOI (10.1109/LRA.2022.3174346); verify both against IEEE Xplore. |
| L5-S23 | astrom_2020_feedback_systems_ch10_loop_analysis.pdf | Princeton UP textbook, author-hosted chapter v3.1.5 | A | S | G | Yes | File confirms Chapter 10 "Frequency Domain Analysis", 2020-07-24. |
| L5-S24 | chrony_2025_faq.md | chrony project (official docs) | B | S | G | Yes | Real page. |
| L5-S25 | chrony_2025_comparison.md | chrony project | B | S | G | Yes | Versions stated. |
| L5-S26 | chrony_2024_chrony_conf_4.6.md | chrony project | B | S | G | Yes | Versioned man page. |
| L5-S27 | linuxptp_v4.4_{ptp4l,phc2sys,ts2phc}.8 | linuxptp project, tag v4.4 | B | S | G | Yes | Pinned tag. The GitHub repo is a mirror of the official linuxptp repo (linuxptp.nwtime.org / SourceForge); optionally cite the official one. |
| L5-S28 | linux_v6.10_networking_timestamping.rst; linux_v6.10_pps.rst | Linux kernel docs, tag v6.10 | B | S | G | Yes | Original `.rst` source, as required. |
| L5-S29 | linux_manpages_6.9_clock_getres.2 | Linux man-pages 6.9 | B | S | G | Yes | Pinned version. |
| L5-S30 | ublox_2011_gps_based_timing_appnote.pdf | u-blox application note | B | S | P | Yes — needs correction | Link is only the u-blox homepage, which does not let anyone find the file; give the document URL. The note covers the older u-blox 6 generation; keep its use to general time-pulse principles (currency caveat). |
| L5-S31 | ublox_2024_zed_f9t_10b_datasheet.pdf | u-blox datasheet R09 | A | S | P | Yes | Manufacturer spec. |
| L5-S32 | geometry2_humble_*.cpp/.hpp (5 files) | ROS 2 geometry2, pinned commit | B | S (named with S13 in SCOPE) | G | Yes | Pinned commit. |
| L5-S33 | ros2doc_humble_*.rst (4 files) | ROS 2 docs, pinned commit | B | S | G | Yes | `file` reports one as "Python script", which is fine for `.rst` with code blocks. |
| L5-S34 | message_filters_humble_* (3 files) | ROS 2 message_filters, pinned commit | B | S | G | Yes | Correct. |
| L5-S35 | common_interfaces_humble_{Header,Image,Imu,LaserScan,TimeReference}.msg | ROS 2 common_interfaces, pinned commit | B | S | G | Yes — needs correction | The File column shortens 4 names ("_Image.msg; _Imu.msg; …"). Write the full file names. |
| L5-S36 | robot_localization_humble_* (3 files) | robot_localization, pinned commit | B | S | G | Yes | Correct. |
| L5-S37 | diagnostics_humble_update_functions.hpp | ros/diagnostics, pinned commit | B | S | G | Yes | Correct. |
| L5-S38 | ethzasl_2024_kalibr_wiki_camera_imu_calibration.md | Kalibr official wiki, pinned wiki commit | B | S | G | Yes | Correct. |
| L5-S39 | ftdi_2006_an232b04_latency.pdf | FTDI application note | B | S | P | Yes | Old (2006) but still FTDI's reference for the latency timer. |
| L5-S40 | xsens_2023_mti600_user_manual.pdf | Xsens/Movella user manual | A | S | P | Yes | Correct. |
| L5-S41 | xsens_2020_mt_low_level_protocol.pdf | Xsens protocol spec MT0101P 2020.A | A | S | P | Yes | Correct. |
| L5-S42 | xsens_ros2driver_e145fb5_* (3 files) | Xsens official ROS 2 driver, pinned commit | B | S | P | Yes | Correct. |
| L5-S43 | velodyne_2019_vlp16_user_manual_revf.pdf | Velodyne manual (Ouster-hosted) | A | S | P | Yes — needs correction | The file's title page reads "63-9243 Rev. F DRAFT" and "Last Updated: 2022-03-07" (© 2022; 2019-07-27 is only a revision-history entry). Fix the year to 2022 and rename the file (`velodyne_2022_…`). Note "DRAFT" and Ouster hosting in the citation. |
| L5-S44 | velodyne_ros2_{driver.cpp,input.cpp,time_conversion.hpp,driver_README.md} | ros-drivers/velodyne, pinned commit | B | S | P | Yes | Pinned; file names lack the commit prefix used elsewhere (cosmetic). |
| L5-S45 | stereolabs_2026_zed_{sensors_time_sync,sensors_api,api_video_module}.md | Stereolabs official docs | B | S | P | Yes — needs correction | No SDK version is given for these live pages. The saved pages come from docs.stereolabs.com/docs/development/zed-sdk/…, not the www.stereolabs.com/docs URLs in the table. Give the real URLs and the SDK version (5.x). |
| L5-S46 | nvidia_2023_driveos606_orin_time_sync.md | NVIDIA DRIVE OS 6.0.6 docs | B | S | P | Yes | Covers DRIVE AGX Orin, not Jetson. The README already says so ("not Jetson"), so it is kept as a cross-domain example only. |
| L5-S47 | nvidia_2026_jetson_r36.4.4_orin_series_features.md | NVIDIA Jetson Linux r36.4.4 docs | B | S | P | Yes | Matches the JetPack 6 / L4T R36 target. |
| L5-S48 | nvidia_2024_jetson_r36.3_generic_timestamp_engine.md | NVIDIA Jetson Linux r36.3 docs | B | S | P | Yes | Short page, but real content (GTE deprecated in favour of HTE). |
| L5-S49 | not downloaded | Springer, Distributed Computing | A | F | G | Yes | No open copy; reason given. |
| L5-S50 | not downloaded | IEEE standard + Springer book | A | F | G | Yes | Paywalled; S05 used as the proxy. |
| L5-S51 | not downloaded | IEEE TAES | A | F | G | Yes | No open copy; described via S15/S16. |
| L5-S52 | not downloaded | IEEE T-ITS | A | F | G | Yes | Only ResearchGate/academia.edu copies, which are behind logins (403); the reason is valid. |
| L5-S53 | li_2014_online_temporal_calib_camera_imu.pdf | IJRR (author copy via Internet Archive) | A | F | G | Yes | 17-page author version; correct. |
| L5-S54 | not downloaded | Springer STAR 79 (ISER 2010) | A | F | G | Yes | Search found no open copy (Springer/ResearchGate only). |
| L5-S55 | vig_2007_quartz_oscillator_tutorial.pdf | Author tutorial (US Army CERDEC), third-party mirror rfseminar.nl | A | F | G | Yes — needs correction | A tutorial is not peer-reviewed, not a standard and not a manufacturer spec. It should be level B, as the Eidson tutorial (S05) is. The file is a third-party mirror; prefer the IEEE UFFC-hosted copy if one can be reached (it returned 403 to curl). |
| L5-S56 | usspaceforce_2022_is_gps_200n.pdf | US Space Force interface spec | A | F | G | Yes | Official specification. |
| L5-S57 | indelman_2012_factor_graph_incremental_smoothing_ins.pdf | IEEE FUSION 2012 (author copy) | A | S | G | Yes | Correct. |
| L5-S58 | autosar_2022_prs_time_sync_protocol.pdf | AUTOSAR FO R22-11 standard | A | S | G | Yes | Doc ID 897 confirmed in file. |
| L5-S59 | franko_2023_8021as_multihop_pi_servo.pdf | IFIP/IEEE CNSM 2023 workshop (AnServApp) | A | S | G | Yes | Peer-reviewed workshop; correct. |
| L5-S60 | kronauer_2021_ros2_latency_analysis.pdf | IEEE MFI 2021; arXiv v3 | A | S | G | Yes | Peer-reviewed version exists. |
| L5-S61 | rmw_humble_types.h | ROS 2 rmw, pinned commit | B | S | G | Yes | Correct. |
| L5-S62 | zed_ros2_wrapper_v5.2.2_* (3 files) | Stereolabs official wrapper, tag v5.2.2 | B | S | P | Yes | Matches the deployed pin. |
| L5-S63 | nav2_humble_{robot_utils,observation_buffer}.cpp | Nav2, pinned Humble commit | B | S | G | Yes | Correct. |
| L5-S64 | nav2docs_588d374_{costmap_2d_index,obstacle_layer}.md | docs.nav2.org (rolling), pinned commit | B | S | G | Yes — needs correction | Currency: the docs describe rolling, while the robot runs Humble. The README finding says "Current Nav2 documentation", which is correct, but the defaults it quotes should be checked against Humble source (S63) or labelled as rolling-only. |
| L5-S65 | bachhuber_2016_glass_to_glass_delay.pdf | IEEE ICIP 2016; arXiv | A | S | G | Yes | Peer-reviewed version exists. |
| L5-S66 | nikolic_2014_synchronized_vi_sensor_fpga.pdf | IEEE ICRA 2014 (author copy via Internet Archive) | A | S | G | Yes | Correct. |
| L5-S67 | zhang_2014_loam.pdf | RSS 2014 (CMU RI copy) | A | S | G | Yes | Correct. |

## Files
- Ran `file` and `pdfinfo` on all 91 files in `sources/`. All 36 PDFs are real PDFs whose first page matches the cited work; none is an HTML, error, login or bot-check page. Two are image-only scans with no text layer (S04 Lamport, S07 TN 1337). Where `file` gives a wrong page count (e.g. TN 1337 "11 pages" vs 357), it comes from PDF header hints; pdfinfo is correct.
- The text, markdown, `.rst`, man-page (`.2`, `.8`), `.msg`, `.cpp/.hpp/.h` and `.yaml` files all match their extensions. `file` labels some `.rst`/`.md` files "C source" or "Python script" because of embedded code blocks; this is expected. Spot-checks of every web-text file show real page content, with no error or login text.
- **Mapping:** every file in `sources/` belongs to a Sources-table row. The only mismatch is L5-S35, where 4 file names are shortened in the table (see needs_correction). Every table row has a file or is marked *not downloaded* (S49, S50, S51, S52, S54), each with a reason.
- **File names:** `velodyne_2019_…` carries the wrong year (the manual is last updated 2022). `velodyne_ros2_*` and `common_interfaces_humble_*` omit the `<year>` part of the naming rule (cosmetic, consistent with other code files in the topic).

## Format
Items saved in a weaker form than an available open PDF:
- **L5-S20 VersaVIS**: arXiv v1 preprint saved, although the peer-reviewed open-access version exists at https://www.mdpi.com/1424-8220/20/5/1439/pdf (DOI 10.3390/s20051439).
- **L5-S04 Lamport**: an image-only scan from a personal mirror; the author-hosted PDF at https://lamport.azurewebsites.net/pubs/time-clocks.pdf is the better copy.
- No paper, book, thesis, standard or datasheet is saved as text. The two RFCs (S02, S06) are saved in their canonical plain-text format, which is correct for IETF RFCs.

## Foundational
Every SCOPE.md foundational reference is either downloaded and cited, or marked *not downloaded* with a reason:

| SCOPE foundational | Status |
|---|---|
| Mills 1991 | S01 downloaded, cited 10× |
| RFC 5905 + Mills 1998 | S02, S03 downloaded |
| Cristian 1989 + Lamport 1978 | S49 not downloaded (no open copy); S04 downloaded |
| IEEE 1588-2019 + Eidson book | S50 not downloaded (paywalled); open proxy S05 downloaded |
| RFC 2783 | S06 downloaded |
| NIST TN 1337 | S07 downloaded |
| Bar-Shalom 2002/2004 | S51 not downloaded (no open copy; described via S15, S16) |
| Larsen 1998 | S09 downloaded |
| Skog & Händel 2011 | S52 not downloaded (login-walled copies only); S10 summary used for abstract |
| Olson 2010 | S11 downloaded |
| Furgale 2013 | S12 downloaded |
| Li & Mourikis 2014 + Kelly & Sukhatme 2014 | S53 downloaded (Internet Archive author copy); S54 not downloaded (no open copy found in this audit either) |
| ROS 2 "Clock and Time" (+ tf2, message_filters docs) | S13 downloaded; tf2/message_filters as S32–S34 |

The README adds two more foundational references, Vig (S55) and IS-GPS-200N (S56), both downloaded. The README's *Foundational references* section matches SCOPE.

## Coverage
- **Subtopics 1–12** (SCOPE) each have a README Findings section (§1–§12) with cited bullets.
- **Gap rows 13–17:** 13, 16 and 17 are covered. 14 (factor graphs, S57 simulation only) and 15 (gPTP numeric requirement; the 802.1AS text is paywalled) are only partly covered, as SCOPE already records.
- **Open items** (already listed in the README's Open questions): Jetson per-module PHC, Xsens output latency, VLP-16 firing-to-host latency, and rolling-shutter modelling.
- **General first:** the README orders the sections general → product-specific. Product material is in §12, placed last. §11 cites DRIVE Orin (S46) as a cross-domain example. Findings in §1–§10 rest on general sources.
- **Share:** 54 of 67 sources (81 %) are general, and 13 (19 %) are product-specific: S30, S31, S39, S40, S41, S42, S43, S44, S45, S46, S47, S48 and S62. Among downloaded sources the split is 49 general and 13 product-specific. ROS 2, Nav2, chrony, linuxptp and Linux-kernel sources count as general because they implement general protocols and middleware.
- **Levels:** 40 A, 27 B, 0 C, 0 D. Levels are applied consistently, except S55 (Vig tutorial at A, while the comparable Eidson tutorial S05 is at B) and S16 (A depends on the FAS workshop being peer-reviewed).
