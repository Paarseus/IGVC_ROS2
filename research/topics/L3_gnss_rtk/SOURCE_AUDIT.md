# L3 — GNSS and RTK: source audit

| | |
|---|---|
| **Topic** | L3 — GNSS and RTK |
| **Date** | 2026-09-28 |
| **Reviewer** | independent — sources |
| **Scope** | README.md Sources table (L3-S01 to L3-S52), `sources/` (64 files), SCOPE.md foundational list and subtopics 1–22, against STANDARDS.md §2–5 |

Method: `file` and `pdfinfo`/`pdftotext` on every file; first page of each paper read to confirm title, authors and venue. Pinned code checked with `git ls-remote` (tags 3.0.0 → 3a3e1c2, v1.4.8 → 5613af2, 3.5.4, 4.2.4, v2.4.3-b34, ros2-1.4.1 all exist). Byte-compared against the pinned upstream files: `navsat_transform.cpp` (first 40 lines), `NavPVT.msg`, and Xsens `ntrip_client.cpp` at e145fb5. All matched.

**Result:** 0 failing, 20 need correction, 32 pass with no change.

## Source table

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| L3-S01 | ublox_2024_zed_f9p_integration_manual.pdf | u-blox (manufacturer), UBX-18010802 R16, 30 Oct 2024 | A | Yes | Product | Yes | Real 129-page manual. Citation is correct. |
| L3-S02 | ublox_2015_gnss_antennas_rf_design.pdf | u-blox application note UBX-15030289 R03 | B | No | Product (general antenna principles) | Yes — **fix** | The file is R03, dated 16-Oct-2019, but its name says 2015. Rename it to `ublox_2019_gnss_antennas_rf_design.pdf`. |
| L3-S03 | teunissen_1995_lambda_fast_gps_surveying.pdf | Int. Symp. GPS Technology Applications, Bucharest 1995 (Curtin copy) | A | Yes (open stand-in for J. Geod. 1995) | General | Yes | OK. |
| L3-S04 | teunissen_2001_integer_bootstrapping.pdf | KIS 2001, Banff | A | Yes (stand-in for Teunissen 1998) | General | Yes | OK. |
| L3-S05 | teunissen_2007_ambiguity_acceptance_tests.pdf | IGNSS Symposium 2007, UNSW | A | Yes | General | Yes | The authors (Teunissen, Verhagen) are confirmed from the PDF. |
| L3-S06 | verhagen_2013_ratio_test_future_gnss.pdf | *GPS Solutions* 17:535–548, Springer | A | Yes | General | Yes | The file is the journal version and its DOI matches. |
| L3-S07 | blewitt_1990_turboedit_cycle_slip.pdf | *Geophys. Res. Lett.* 17(3) | A | No (SCOPE "strong supporting") | General | Yes | OK. |
| L3-S08 | langley_1999_dilution_of_precision.pdf | *GPS World* Innovation column (trade magazine; named UNB author) | C | No | General | Yes — **fix** | The link is only the gpsworld.com homepage, so the item cannot be traced. Give the actual PDF URL (e.g. the UNB-hosted copy). The magazine is editorially reviewed but not peer-reviewed; add a note on why it is level C. |
| L3-S09 | novatel_2010_intro_to_gnss_1st_ed.pdf | NovAtel (C. Jeffrey), 1st ed. 2010 | B | No | General | Yes — **fix** | The link points to the page for the 3rd edition, not to where the 1st-edition PDF was downloaded (SCOPE says "open 1st-ed. PDF mirror"). Give the actual download URL and state that the 3rd edition exists. |
| L3-S10 | dod_2020_gps_sps_performance_standard.pdf | U.S. DoD, SPS PS 5th ed. 2020 | A | Yes | General | Yes | OK. |
| L3-S11 | henning_2014_ngs_single_base_rtk_guidelines_v3.1.pdf | NOAA NGS guidelines v3.1 | A | Yes | General | Yes | OK. See the level-consistency note at S41/S42. |
| L3-S12 | landau_2002_virtual_reference_stations.pdf | *J. Global Positioning Systems* 1(2):137–143 | A | Yes | General | Yes | Content is correct, but the PDF metadata title is wrong ("An Overview of Atmospheric Radio Occultation"). This is harmless. |
| L3-S13 | bkg_ntrip_documentation.pdf | BKG, public NTRIP 1.0 description | B | Yes (open stand-in for RTCM 10410) | General (protocol) | Yes — **fix** | The file name has no year (the convention needs `<org>_<year>_…`). Add the document date to both the file name and the citation. |
| L3-S14 | weber_2007_bkg_ntrip_client_bnc.pdf | BKG, BNC Version 1 manual | B | No | Product (BNC) | Yes — **fix** | **Currency:** this manual is for BNC v1, which v2.13.7 (L3-S40) supersedes. README findings l.155–156 and Key numbers l.269 still quote v1 behaviour (20 s outage rule, 256 s back-off) as if it were current. Cite L3-S40 for current behaviour and label S14 as historical. The file name says 2007 but the citation says c. 2008; make them agree. |
| L3-S15 | rtklib_v2.4.3-b34_stream.c | RTKLIB official repo, tag v2.4.3-b34 (verified) | B | Yes (with S16/S17) | Product (RTKLIB) | Yes | OK. |
| L3-S16 | rtklib_2013_manual_2.4.2.pdf | RTKLIB official repo, doc/ at v2.4.3-b34 | B | Yes | Product (RTKLIB) | Yes | OK. |
| L3-S17 | takasu_2009_rtklib_lowcost_rtk.pdf | Int. Symp. GPS/GNSS 2009, Jeju | A | Yes | General | Yes — **fix** | The link is only the rtklib.com homepage. Give the direct PDF URL and the page range. |
| L3-S18 | microstrain_ntrip_client_ros2-1.4.1_* (5 files) | LORD-MicroStrain repo, tag ros2-1.4.1 (verified) | C | No | Product | Yes | OK. |
| L3-S19 | xsens_ntrip_e145fb5_* (4 files) | xsenssupport official repo, commit e145fb5 (file byte-matches) | B | No | Product | Yes | OK. |
| L3-S20 | xsens_ros2driver_e145fb5_* (2 files) | xsenssupport official repo, commit e145fb5 | B | No | Product | Yes | OK. |
| L3-S21 | reid_2019_rtk_30000km_highways.pdf | ION GNSS+ 2019 (arXiv:1906.08180 copy) | A | No (SCOPE "strong supporting") | General | Yes — **fix** | Add the ION GNSS+ 2019 proceedings page range (and ION DOI if one exists). The citation currently gives only the arXiv ID. |
| L3-S22 | humphreys_2020_deep_urban_rtk.pdf | *IEEE ITS Magazine* 12(3):109–122, 2020 (arXiv v2 copy) | A | No | General | Yes | The peer-reviewed venue is correctly given. |
| L3-S23 | pesyna_2014_smartphone_antenna_cm_positioning.pdf | ION GNSS+ 2014 (author copy) | A | No | General | Yes — **fix** | The link is a lab homepage, not the PDF URL. Add the exact PDF URL and the proceedings page range. |
| L3-S24 | odolinski_2017_lowcost_gps_bds_rtk.pdf | *GPS Solutions* 21:1315–1330 | A | No | General | Yes | OK (journal PDF). |
| L3-S25 | macgougan_2001_gnss_signal_degradation_overview.pdf | KIS 2001, Univ. of Calgary | A | No | General | Yes — **fix** | The Link column holds no URL ("University of Calgary, Dept. of Geomatics Engineering"), so traceability is incomplete. Add the source URL and page range. |
| L3-S26 | schmid_2007_absolute_phase_center_model.pdf | *J. Geodesy* 81:781–798 (IGS copy) | A | Yes | General | Yes | OK. |
| L3-S27 | shin_2005_lowcost_ins_thesis.pdf | PhD thesis, Univ. of Calgary (UCGE 20219) | A | No (stand-in for Groves on lever arm) | General | Yes | OK. Level A for an examined PhD thesis is accepted. |
| L3-S28 | chauchat_2024_invariant_lever_arm.pdf | arXiv:2409.07050 → **IEEE CDC 2024 (63rd Conf. on Decision and Control, Milan, Dec 2024)** | C → **A** | No | General | Yes — **fix** | A peer-reviewed version exists (IEEE CDC 2024, per the Mines Paris CAOR publication list). Cite CDC 2024 (add DOI/pages), keep the arXiv PDF as the open copy, and raise the level to A (STANDARDS §2, Preprints). Remove "venue not confirmed". |
| L3-S29 | robotlocalization_3.5.4_* (3 files) | robot_localization official repo, tag 3.5.4 (verified, cpp matches) | B | Yes | Product (ROS package) | Yes | OK. |
| L3-S30 | ros2_common_interfaces_4.2.4_*.msg | ROS 2 official repo, tag 4.2.4 | B | No | General (ROS standard message) | Yes | OK. |
| L3-S31 | ros_rep105_coordinate_frames.rst | ROS REP-105 (Meeussen), original .rst | A | Yes | General | Yes | OK. REP-103, named with it in SCOPE, was not downloaded (see Foundational). |
| L3-S32 | xsens_2023_mti600_user_manual.pdf | Movella/Xsens, MTi 600-series User Manual (31 Oct 2023) | A | No | Product | Yes — **fix** | The link is the mtidocs homepage. Give the page URL or document ID. |
| L3-S33 | xsens_2020_mti600_datasheet.pdf | Xsens, MT1603P rev. 2020.B | A | No | Product | Yes — **fix** | **Currency:** the June 2020 revision predates the 2023 user manual (S32) and current MTi-680G firmware. Check for a newer revision, or say that 2020.B was used on purpose. Give a specific link. |
| L3-S34 | xsens_2020_mti_family_reference_manual.pdf | Xsens, MT1600P rev. 2020.A | A | No | Product | Yes — **fix** | Same currency and link issue as S33. S34 is cited for RTCM input (README §6), so it matters which revision is current. |
| L3-S35 | xsens_mti600_hardware_integration_manual.pdf | Xsens, MT1601P rev. 2020.A | A | No | Product | Yes — **fix** | The file name has no year; rename to `xsens_2020_mti600_hardware_integration_manual.pdf`. Same currency and link issue as S33. |
| L3-S36 | xsens_mti600_dk_user_manual.pdf | Xsens, MT1602P rev. C, June 2020 | A | No | Product | Yes — **fix** | The file name has no year; rename to `xsens_2020_mti600_dk_user_manual.pdf`. Give a specific link. |
| L3-S37 | movella_2022_gnss_ins_supercharge_appnote.pdf | Movella application note MTAN001 rev. A | B | No | Product | Yes — **fix** | Real 31-page app note (the title reads like marketing, but the content is technical). The link is only movella.com; give a direct URL. The README Summary (l.14) and §4 (l.55) use S37 for a general RTK principle (multipath is not cancelled). Cite a general source (S09/S11/S25) first. |
| L3-S38 | weber_2005_ntrip_ion_gnss.pdf | ION GNSS 2005, pp. 2243–2247 (BKG copy) | A | Yes (stand-in for Weber 2005 IAG) | General | Yes | Authors confirmed from the PDF (Weber, Dettmering, Gebhard, Kalafus). |
| L3-S39 | rtcm_2009_ntrip_v2_press_release.pdf | RTCM (standards body) press release, 2009 | B | No | General | Yes | A thin, 2-page official summary. Use it only for the fact that the NTRIP 2.0 changes exist. |
| L3-S40 | bkg_bnc_2.13.7_help.md | BKG, BNC 2.13.7 source zip (sha256 given) | B | No | Product (BNC) | Yes | Saved as text from the official HTML help, which is correct for web documentation. |
| L3-S41 | igs_real_time_broadcaster_station_guidelines.pdf | IGS RTWG/IC, v1.0 Oct 2021 | B | No | General | Yes — **fix** | The file name has no year (`igs_2021_…`). **Level consistency:** these are official agency practice guidelines, like the NGS guidelines (S11), which are graded A. Grade S11, S41 and S42 the same way (A for all three is recommended), or give the reason for the difference. |
| L3-S42 | euref_epn_station_guidelines.pdf | EPN Central Bureau (C. Bruyninx), version 1 Sep 2025 | B | No | General | Yes — **fix** | The file name has no year (`euref_2025_…`). The URL printed in the PDF (`guidelines_station_operationalcentre.pdf`) differs from the table link (`guidelines_EPN_stations.pdf`); confirm the link. Fix the level as for S41. |
| L3-S43 | kumarrobotics_ublox_3.0.0_* (2 files) | KumarRobotics repo, tag 3.0.0 = 3a3e1c2 (verified, NavPVT.msg matches) | C | No | Product | Yes | OK. |
| L3-S44 | septentrio_gnss_driver_v1.4.8_message_handler.cpp | Septentrio official repo, tag v1.4.8 = 5613af2 (verified) | B | No | Product | Yes | OK. |
| L3-S45 | zhu_2018_gnss_integrity_urban_review.pdf | *IEEE T-ITS* 19(9):2762–2778 (HAL copy) | A | Yes (added in the gap check) | General | Yes | OK. |
| L3-S46 | reid_2019_localization_requirements_av.pdf | *SAE Int. J. CAV* 2(3):173–190 (arXiv copy) | A | No | General | Yes | The peer-reviewed venue is correctly given. |
| L3-S47 | odolinski_2019_lowcost_rtk_ionospheric_disturbance.pdf | *J. Geodesy* 93:701–722 (OA) | A | No | General | Yes | OK. |
| L3-S48 | janos_2021_lowcost_f9p_network_rtk_demanding.pdf | *Sensors* 21(16):5552, MDPI with DOI | A | No | General | Yes | OK. |
| L3-S49 | choy_2017_ppp_misconceptions.pdf | *GPS Solutions* 21(1):13–22 (open FIG Article of the Month 2016) | A | No | General | Yes | The file is the FIG 2016 version and the README says so. The name uses the journal year, which is acceptable. |
| L3-S50 | borko_2018_gnss_ins_virtual_lever_arm.pdf | *Sensors* 18(7):2228, MDPI with DOI | A | No | General | Yes | OK. |
| L3-S51 | tallysman_2020_ground_planes.pdf | Tallysman (manufacturer) note, distributor-hosted copy | B | No | Product (general ground-plane physics) | Yes | Passes as a manufacturer technical note. It is labelled a "brochure" but the content is technical. If possible, prefer a copy hosted by Tallysman or Calian. |
| L3-S52 | remondi_1985_cm_surveys_in_seconds.pdf | NOAA Tech. Memo NOS NGS-43, 1985 | A | Yes | General | Yes | A scan without a text layer; the content is real. |

## Files

- `file` was run on all 64 files in `sources/`. Every PDF is a real PDF, and every `.cpp`/`.c`/`.py`/`.msg`/`.yaml`/`.rst`/`.md` is text of the matching kind. `xsens_ntrip_e145fb5_ntrip_client_node.cpp` is reported as "C source", which is the usual libmagic result for C++ and is fine.
- None of the files is an error page, a login wall or a bot-check page. The first page of every PDF was read and matches its citation.
- Every file appears in the Sources table, and every row points to at least one existing file. No row is marked "not downloaded" (items that were not downloaded appear only in the Foundational table).
- File-name convention problems (STANDARDS §3):
  - year missing: `bkg_ntrip_documentation.pdf`, `euref_epn_station_guidelines.pdf`, `igs_real_time_broadcaster_station_guidelines.pdf`, `xsens_mti600_hardware_integration_manual.pdf`, `xsens_mti600_dk_user_manual.pdf`, `ros_rep105_coordinate_frames.rst` (the last is minor for a living REP)
  - wrong year: `ublox_2015_…` should be 2019; `weber_2007_…` does not match "c. 2008".

## Format

- No paper, book, thesis, standard or datasheet was saved as text when an open PDF exists. The only text sources are web or HTML documentation (BNC help, REP `.rst`, robot_localization `.rst`) and code, which is the correct format for each.
- PDF upgrades needed: 0.
- L3-S49 uses the open FIG 2016 PDF, not the paywalled *GPS Solutions* version. This is acceptable and is disclosed.

## Foundational

| SCOPE foundational reference | Status |
|---|---|
| Springer Handbook of GNSS (2017) | Not downloaded; reason given (no open copy) |
| Misra & Enge | Not downloaded; reason given |
| Kaplan & Hegarty | Not downloaded; reason given |
| Groves 2013 | Not downloaded; reason given (L3-S27 used instead) |
| Counselman & Gourevitch 1981 | Not downloaded; reason given |
| Remondi 1985 *Bull. Géod.* / *NAVIGATION* 32(4) | *Bull. Géod.* not downloaded (given). The *NAVIGATION* paper is covered by the NOAA memo version (L3-S52) |
| Hatch 1990 | Not downloaded; reason given |
| Teunissen 1995 *J. Geodesy* (LAMBDA) | Not downloaded; reason given; open conference paper L3-S03 used |
| Teunissen 1998 *J. Geodesy* (success probability) | **Not listed explicitly.** The README uses Teunissen 2001 (S04) instead but never marks the 1998 paper as not downloaded. Add a "not downloaded" line |
| Verhagen & Teunissen 2013 ratio test | Downloaded (L3-S06) |
| RTCM 10403.x and 10410.1 | Not downloaded; reason given (sold by RTCM) |
| Weber et al. 2005 IAG Symposia 128 | Not downloaded; reason given; ION GNSS 2005 version L3-S38 used |
| Landau et al. 2002 VRS | Downloaded (L3-S12) |
| Henning NGS single-base guidelines | v3.1 downloaded (L3-S11). The companion NGS *Guidelines for Real Time GNSS Networks* named in SCOPE is **not downloaded and not marked**; add a line |
| Schmid et al. 2007 | Downloaded (L3-S26) |
| Takasu & Yasuda 2009 + RTKLIB | Downloaded (L3-S15, S16, S17) |
| GPS SPS Performance Standard 2020 | Downloaded (L3-S10) |
| REP-105 + REP-103 + navsat_transform | REP-105 and navsat_transform downloaded (L3-S31, S29). **REP-103 is not downloaded and not marked**; add it, or state that it is not needed |
| u-blox ZED-F9P Integration Manual | Downloaded (L3-S01) |

## Coverage

- **Subtopics:** all 22 SCOPE subtopics have cited findings. Subtopics 1–12 have their own README sections. Subtopics 13–22, added in the gap check, are covered inside those sections:
  - 13: S49
  - 14: S47
  - 15: S11, S01
  - 16: S38–S42
  - 17: S43, S44
  - 18: S45, S46
  - 19: S48
  - 20: S50, S28
  - 21: S51
  - 22: S16, S09, S40
- **Known thin spots**, already listed under Open questions and not failures:
  - quantified on-vehicle EMI C/N0 loss
  - lever-arm error budget
  - normative RTCM/NTRIP 2.0 details
- **General first:** not always. §1, §2, §5, §8 and §9 open with general sources. §3 (S01), §4 (S02), §6 (S01), §7 (S01) and §11 (S01) open with u-blox product material, and the Summary cites the Movella app note (S37) for a general RTK principle. Reorder these sections so general sources come first: S11/S21/S24/S47 for §3, S23/S25/S26 for §4, S13/S38 for §6, S16/S21 for §7, S21/S22 for §11. This is a README fix, not a source fix.
- **Share:** 32 of 52 sources (62 %) are general. 20 of 52 (38 %) are product-specific: S01, S02, S14, S15, S16, S18, S19, S20, S29, S32–S37, S40, S43, S44, S51.

Sources for the S28 venue check: [Mines Paris CAOR publications](https://www.caor.minesparis.psl.eu/publications/), [arXiv 2409.07050](https://arxiv.org/abs/2409.07050).
