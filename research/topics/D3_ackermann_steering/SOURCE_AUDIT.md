# D3 — Ackermann / car-like steering: source audit

**Topic:** D3 — Ackermann / car-like steering
**Date:** 2026-10-06
**Reviewer:** independent — sources (step 4 of STANDARDS.md §5, sources arm; README.md not edited)

Scope of this audit: every source in the Sources table, every file in `sources/`, citation accuracy, file format, foundational-reference completeness, and subtopic coverage, checked against `STANDARDS.md` §2–5. Methodology: ran `file` on all 44 files in `sources/`; read the first page (and, where relevant, index/back-matter or specific cited pages) of every PDF with `pdftotext`/`pdfinfo`; read every `.md`/`.rst`/`.msg`/`.hpp` file directly; cross-checked every citation's authors/venue against the downloaded file's own title page; cross-checked `SCOPE.md`'s Foundational references and Subtopics tables against `README.md`.

## 1. Source quality table

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| D3-S01 | reeds_shepp_1990_optimal_paths_car.pdf | *Pacific Journal of Mathematics* | A | Yes | General | Yes | Peer-reviewed math journal; title/authors verified on p.1 against citation. |
| D3-S02 | macenski_2023_regulated_pure_pursuit.pdf | *Autonomous Robots* (Springer), via arXiv preprint | A | Yes | General | Yes | Peer-reviewed venue correctly cited per the preprint rule (DOI given, venue named). |
| D3-S03 | coulter_1992_pure_pursuit.pdf | CMU-RI technical report | A | Yes | General | Yes | Verified p.1 (authors, report number). |
| D3-S04 | snider_2009_automatic_steering_survey.pdf | CMU-RI technical report | A | Yes | General | Yes | Verified. |
| D3-S05 | shamah_1999_skid_vs_explicit_steering.pdf | CMU-RI technical report | A | Yes | General | Yes | Verified. |
| D3-S06 | sousa_petry_moreira_odometry_calibration_survey.pdf | IEEE ICARSC 2020 | A | Yes | General | Yes | Peer-reviewed IEEE conference. |
| D3-S07a | ackermann_msgs_AckermannDrive.msg | ros-drivers/ackermann_msgs, tag v2.0.2 | B | Yes (as package) | General | Yes | Official project source, pinned tag. |
| D3-S07b | ackermann_msgs_AckermannDriveStamped.msg | same | B | Yes | General | Yes | Same. |
| D3-S07c | ackermann_msgs_README.rst | same | B | Yes | General | Yes | Same. |
| D3-S08 | hoffmann_2007_stanley_control_acc.pdf | Proc. American Control Conference 2007 | A | Yes | General | Yes | Verified p.1 (Stanford authors, Stanley controller). |
| D3-S09 | ros2_controllers_steering_controllers_library_userdoc.rst | ros2_controllers, pinned commit | B | No | General | Yes | Official project doc source file, pinned commit. |
| D3-S10 | ros2_controllers_bicycle_steering_controller_userdoc.rst | same | B | No | General | Yes | Same. |
| D3-S11 | ros2_controllers_tricycle_steering_controller_userdoc.rst | same | B | No | General | Yes | Same. |
| D3-S12 | ros2_controllers_ackermann_steering_controller_userdoc.rst | same | B | No | General | Yes | Same. |
| D3-S13 | nav2docs_configuring_regulated_pp.md | Nav2 docs, pinned commit | B | No | General | Yes (see Files §2) | Real official Nav2 doc content; `file` flags it as HTML because of an embedded `<style>`/video-`<iframe>` block — see Files section, not an error/login page. |
| D3-S14 | nav2docs_configuring_smac_hybrid.md | Nav2 docs, pinned commit | B | No | General | Yes | Official doc, pinned commit. |
| D3-S15 | nav2docs_configuring_mppic.md | Nav2 docs, pinned commit | B | No | General | Yes | Same. |
| D3-S16 | nav2_mppi_controller_motion_models.hpp | navigation2, pinned commit | B | No | General | Yes | Code pinned to commit, not a moving branch. |
| D3-S17 | jpl_rover_maneuvering_autonomous_manipulation.pdf | JPL document 00-0139 | A | No | General | Yes | Verified p.1 (Nesnas/Maimone/Das, JPL). Minor: filename uses org acronym "jpl" rather than first-author surname, unlike every other individually-authored source in this topic (see needs_correction). |
| D3-S18 | holand_2025_passive_skid_steer_rover_thesis.pdf | CMU-RI-TR-25-87 (MSR thesis) | **C** | No | General | Yes | Content verified. **Level inconsistent** — same CMU-RI-TR report series/institutional rigor as D3-S03/S04/S05 (graded A); see needs_correction. |
| D3-S19 | deshpande_cmu_msr_thesis.pdf | CMU-RI-TR-23-39 (MSR thesis) | **C** | No | General | Yes | Same inconsistency as D3-S18. |
| D3-S20 | okelly_2020_f1tenth_evaluation_env.pdf | NeurIPS 2019 Comp./Demo Track, PMLR 123 | A | No | Product | Yes | PMLR-indexed proceedings; reasonable A. |
| D3-S21 | srinivasa_2019_mushr_racecar.pdf | arXiv preprint, no peer-reviewed venue found | C | No | Product | Yes (see needs_correction) | Correct application of the preprint→C default rule. File content is v3 (PDF `CreationDate` Dec 2023; in-text "arXiv:1908.08031v3"), matching the citation's "2023," but the **filename says 2019** — mismatch with the cited/downloaded version. |
| D3-S22 | camara_2023_openpodcar.pdf | *Journal of Open Hardware* 2023, via arXiv | A | No | Product | Yes | Peer-reviewed venue with DOI; author "Camara" matches filename. |
| D3-S23 | camara_2026_openpodcar2_ros2.pdf | arXiv preprint, not yet confirmed peer-reviewed | C | No | Product | Yes (see needs_correction) | Correct C per preprint rule. **Filename wrong**: file is named "camara_..." but the actual paper's first author (verified p.1) is **Rakshit Soni** (Soni, Waltham, Ibrahim, Crampton, Fox) — "Camara" is the first author of the *sibling* paper D3-S22, apparently copied by mistake. |
| D3-S24 | agilex_hunter20_user_manual.pdf | AgileX Hunter 2.0 User Manual, distributed by MyBotShop | A | No | Product | Yes (see needs_correction) | 62-page manual with a genuine numeric "Technical Specifications" section; A is defensible. Attribution imprecise — see needs_correction. |
| D3-S25 | pacmod_lexus_brochure.pdf | AutonomouStuff product brochure | A (as graded) | No | Product | **No — FAILS** | Self-described "product brochure"; promotional copy ("About AutonomouStuff" ×2, "Key Features," safety-marketing framing), no quantitative datasheet. STANDARDS.md §2: "marketing pages are not used." See Failing list. |
| D3-S26 | igvc_2026_rules.pdf | IGVC official rules, Oakland University | A | No | General | Yes | Verified exact wording ("turning radius not less than five feet"). |
| D3-S27 | bobjones_eran_2010_design_report.pdf | IGVC design report | C | No | General | Yes | Matches STANDARDS' own named C example. |
| D3-S28 | oakland_botzilla_2011_design_report.pdf | IGVC design report | C | No | General | Yes | Same. |
| D3-S29 | utaustin_enterpras_2011_design_report.pdf | IGVC design report | C | No | General | Yes | Same (zip-deflate-encoded PDF, not an error — confirmed real content). |
| D3-S30 | zhang_2009_agricultural_platooning_rtk_steering.pdf | IEEE ICVES 2009 | A | No | General | Yes | Verified p.1. |
| D3-S31 | paden_2016_survey_motion_planning_control.pdf | IEEE Trans. Intelligent Vehicles, via arXiv | A | No | General | Yes | Verified p.1; preprint rule correctly applied (DOI + venue given). |
| D3-S32 | carron_2023_chronos_crs_miniature_car.pdf | ICRA 2023, via arXiv | A | No | General | Yes | Research platform (not a commercial product); preprint rule correctly applied. |
| D3-S33 | lavalle_2006_planning_algorithms.pdf | *Planning Algorithms*, Cambridge Univ. Press | A | Yes (added, gap check) | General | Yes | Verified title page; §13.1.2.1 "A simple car" located at internal book pp. 723–724 (PDF is a 2-up render, 512 physical pages vs. ~1030 book pages per the index's own page numbers reaching ~920) — the cited "pp. 725-726" is consistent with this, not an error. |
| D3-S34 | padua_2020_ackermann_race_car_performance.pdf | *Vehicle System Dynamics* (Taylor & Francis), accepted manuscript | A | Yes (added, gap check) | General | Yes (see needs_correction) | Content verified (Veneri & Massaro, Univ. of Padova); "Received 00 Month 20XX / accepted 00 Month 20XX" is Taylor & Francis's standard accepted-manuscript self-archiving template — normal, not a red flag. **Filename** uses the university's city ("padua") instead of first-author surname "Veneri" — inconsistent with this project's own naming convention. |
| D3-S35 | iaeng_2023_fsae_ackermann_steering_optimization.pdf | *IAENG Int'l J. Applied Mathematics* 53(4) | **B** | No | General | Yes (see needs_correction) | Content/authors verified (Cai, Zhao, Sun, Sun); real review timeline found ("received March 23, 2023, revised October 19, 2023"). **Level category mismatch**: B is defined for official manufacturer/project documentation, not a journal article — this should be A or C, not B. **Filename** uses the venue acronym "iaeng" instead of first-author surname "Cai." |
| D3-S36 | polack_2018_kinematic_bicycle_consistency_acc.pdf | 2018 American Control Conference, via arXiv | A | No | General | Yes | Recognized peer-reviewed venue; preprint rule correctly applied. |
| D3-S37 | aertssen_2026_single_track_model_position_accuracy.pdf | arXiv preprint; acceptance status unconfirmed | **B** | No | General | Yes (see needs_correction) | Authors/affiliation verified (TU Eindhoven / DAF Trucks). **Level category mismatch + internal inconsistency**: this topic grades every other unconfirmed-peer-review preprint (D3-S21, D3-S23, D3-S38) as C per STANDARDS' explicit preprint rule, but this one is B. The document's own header reads "Received on M, D, YYYY / Accepted on M, D, YYYY" — literal unfilled template fields, not actual dates — so the Sources-table description "accepted paper" is not substantiated by the file itself (contrast with D3-S34, whose similar-looking placeholder is the standard filled T&F template, not unfilled field names). |
| D3-S38 | caponio_2024_qcar_scaled_vehicle_modelling.pdf | arXiv preprint, "not yet confirmed peer-reviewed" | C | No | Product | Yes | Correct application of preprint rule. |
| D3-S39 | quanser_qcar2_product_page.md | Quanser QCar 2 product page | A | No | Product | Yes | Page has a genuine "Specifications" section with hard numeric data (the part actually cited); marketing chrome elsewhere on the page is not what is cited. |
| D3-S40 | agilex_limo_specifications_trossen.md | Trossen Robotics distributor docs | C | No | Product | Yes | Verified weight/dimensions match; verified the per-mode turning-radius/payload numbers the README says it could **not** confirm are indeed absent from this file — the README's own caveat checks out. |
| D3-S41 | agilex_limo_steering_modes_trossen.md | Trossen Robotics distributor docs | C | No | Product | Yes | Verified latch/30°-rotation mechanism description matches Finding §8 closely. |
| D3-S42 | igvc_2020_rules.pdf | IGVC official rules, Oakland University, 2020 | A | No | General | Yes | Verified identical wording to the 2026 rules. |

**Totals:** 44 files / 44 table rows (42 distinct `D3-S` IDs, with D3-S07 split into three files). 43 of 44 pass the acceptance checklist; 1 (D3-S25) fails outright. Levels: A = 24, B (as graded) = 13, C = 13 — of which 4 rows (D3-S18, D3-S19, D3-S35, D3-S37) have a recommended level change (see Needs correction).

## 2. Files (`file` on every file in `sources/`)

All 44 files were checked with `file`. Findings:

- **43 of 44 match their extension and are real content** (PDF documents with sane page counts opened and read; `.md`/`.rst`/`.msg`/`.hpp` are ASCII/UTF-8 text). Several PDFs that `file` could not report a page count for (`camara_2023`, `camara_2026`, `caponio_2024`, `holand_2025`, `padua_2020`) were independently confirmed with `pdfinfo` to have real page counts (23, 67, 16, 97, 20 pages respectively) and real `pdftotext` content — not corrupted, not blank.
- **No error, login, paywall or bot-check pages** were found in any file. First-page (or title-page) text was read for every PDF foundational reference and for a representative sample of the rest, and all matched their claimed authors/venue.
- **One type/extension mismatch**: `nav2docs_configuring_regulated_pp.md` (D3-S13) is reported by `file` as `HTML document, ASCII text` rather than markdown/text. Inspection shows this is because the genuine, 354-line official Nav2 documentation page embeds a ~40-line raw `<style>` + YouTube `<iframe>` comparison-video widget near the top; the remaining ~300 lines (including the actual cited parameter list) are ordinary markdown/YAML. This is **not** an error/login/bot-check page — it is real official content — but it does not meet STANDARDS §3's "clean markdown" bar for web documentation as cleanly as the other Nav2/`ros2_controllers` doc files in this topic. Recommend stripping the HTML widget block (needs_correction, not failing).
- Every file in `sources/` has a matching row in the Sources table, and every Sources-table row has a file (verified by extracting all `sources/...` paths from the table and diffing against `ls sources/`: zero orphans either direction).

## 3. Citations

Spot-verified against the downloaded file's own title/first page for every foundational reference and a broad sample of supporting sources (23 of 44 files read directly for this purpose): authors, title, venue and year all matched the Sources-table citation in every case checked, with the two filename-only exceptions below (the in-table *citation text* was correct in both cases — only the *file name* was wrong):

- **D3-S23**: citation text correctly names Soni et al., but the file is named `camara_2026_...` (first author of the sibling paper D3-S22) — apparent copy-paste of the wrong filename.
- Preprint peer-review checks: D3-S02, D3-S22, D3-S31, D3-S32, D3-S36 all cite a confirmed peer-reviewed venue (with DOI) for an arXiv preprint and are graded A, correctly following the preprint rule. D3-S21, D3-S23, D3-S38 have no confirmed peer-reviewed venue and are correctly graded C. D3-S37 is the one exception that breaks this otherwise-consistent pattern (graded B; see table and Needs correction).
- Code/doc citations (D3-S07, D3-S09–S16) are all pinned to a specific tag or commit hash, never a bare branch name, per STANDARDS §3.

## 4. Format

Checked every paper/book/thesis/standard/datasheet in `sources/` for whether it was saved as text/markdown when an open PDF exists. **None found** — every academic paper, thesis, competition-rules document, design report and manual in this topic is saved as PDF. The `.md`/`.rst`/`.msg` files in `sources/` are all web documentation, product pages, or source-controlled doc/code files (Nav2 docs, `ros2_controllers` userdoc, `ackermann_msgs`, Quanser/Trossen product pages) — exactly the category STANDARDS §3 says has "no PDF" and should be saved as the page's own markdown/text or original source file. No upgrades needed.

## 5. Foundational

All 12 foundational references listed in `SCOPE.md`'s original Foundational references table are accounted for in `README.md`'s Foundational references table:

| SCOPE.md foundational reference | Status in README.md |
|---|---|
| Lankensperger/Ackermann 1816–1818 | Not downloaded, reason given (no primary patent document found meeting the checklist) |
| Dubins 1957 | Not downloaded, reason given (JSTOR/AMS paywalled) |
| Reeds & Shepp 1990 | Downloaded, D3-S01 |
| Coulter 1992 | Downloaded, D3-S03 |
| Macenski et al. 2023 | Downloaded, D3-S02 |
| Snider 2009 | Downloaded, D3-S04 |
| Hoffmann et al. 2007 | Downloaded, D3-S08 (open copy found during research step, at ai.stanford.edu, after the mapping step had flagged it paywalled — expected progress, not a discrepancy) |
| Rajamani, *Vehicle Dynamics and Control* | Not downloaded, reason given (Springer paywalled) |
| Gillespie, *Fundamentals of Vehicle Dynamics* | Not downloaded, reason given (SAE paywalled) |
| Shamah 1999 | Downloaded, D3-S05 |
| Sousa, Petry & Moreira | Downloaded, D3-S06 |
| `ackermann_msgs` | Downloaded, D3-S07a/b/c |

Two further foundational-caliber references were added during the 2026-10-06 gap check and are both downloaded and cited (LaValle, D3-S33; Veneri & Massaro, D3-S34); four more items the gap check searched for (ISO 8855, SAE J695, Kong et al. 2015, Polack et al. 2017-original) remain correctly marked "not downloaded" with a reason, in both `SCOPE.md`'s gap-check log and `README.md`'s Foundational references table. **No foundational reference is missing or silently dropped.**

## 6. Coverage

All 12 subtopics in `SCOPE.md` have cited findings in `README.md`: Findings §1–§12 map one-to-one onto Subtopics 1–12, and every subtopic's (a)/(b)/(c) sub-questions have at least one corresponding, cited bullet (cross-checked topic by topic).

**General-first ordering:** the topic is structured general-to-specific at the section level — general kinematics/geometry/control theory and ROS 2/Nav2 software support (Findings §1–§7) precede the off-the-shelf-platform survey (§8–§9), which precedes the IGVC-specific application (§10) and other-domain generalizations (§11–§12). Within sections, general principles are stated before single-example numbers (e.g. §1 states the general Ackermann-condition equation before the one-FSAE-car numeric example).

**General vs. product-specific source split:** classifying each of the 42 distinct source IDs as "general" (theory, standards, ROS/Nav2 software, surveys, competition rules, one-off research/thesis robots) or "product-specific" (named commercial/off-the-shelf platforms — F1TENTH, MuSHR, OpenPodcar/2, AgileX Hunter/LIMO, Quanser QCar/QCar2, PACMod):

- Product-specific: D3-S20, S21, S22, S23, S24, S25, S38, S39, S40, S41 = **10 of 42 (≈24%)**
- General: the remaining **32 of 42 (≈76%)**

This matches STANDARDS §1's "general first, specific second" intent at the sourcing level, not just the section-ordering level — most of the topic's evidence base is general/cross-vehicle, with product-specific material correctly scoped to one supporting section (§8–§9) rather than dominating the topic.

## Failing (must be removed)

- **D3-S25** (`pacmod_lexus_brochure.pdf`) — self-described "product brochure." Content is promotional vendor copy (two "About AutonomouStuff" boxes, a "Key Features" marketing list, safety-messaging framing) rather than a technical datasheet; the only factual content is a bullet list of controlled functions and feedback signals, with no quantitative specs (response times, voltages, tolerances, etc.). STANDARDS.md §2 states plainly: "marketing pages are not used." Recommend removing this source and, if the control/feedback-signal facts in Finding §9 are still wanted, re-sourcing them from AutonomouStuff's PACMod technical documentation or ROS driver repository/README (an official-project-code-type source), if one can be found.

## Needs correction (passes, but needs a fix)

1. **D3-S18, D3-S19** (Holand 2025 / Deshpande 2023, both CMU-RI-TR theses) — graded C, but D3-S03/S04/S05 (Coulter/Snider/Shamah) are the *same* CMU-RI-TR technical-report series from the *same* institute, graded A. An MSR thesis undergoes committee review, at least as rigorous as an internally-reviewed technical report. Recommend regrading D3-S18/S19 to A for internal consistency (or, if the reviewer prefers the stricter reading, downgrading D3-S03/S04/S05 to C instead — but these are long-standing foundational references for this field, so raising S18/S19 is the lower-disruption fix).
2. **D3-S35** (Cai et al., IAENG) — graded B, but B is defined for official manufacturer/project documentation, not a journal article. Recommend A (if the reviewer judges IAENG's review process, confirmed real via "received/revised" dates, as adequate peer review) or C (safer default, since IAENG is not among STANDARDS' named Level-A venues). Also rename the file from `iaeng_2023_...` to `cai_2023_...` (first-author surname, per this project's own naming convention).
3. **D3-S37** (Aertssen et al.) — graded B; should be C per STANDARDS' explicit preprint-default rule, consistent with how D3-S21/S23/S38 (structurally identical unconfirmed-peer-review preprints in this same topic) were graded. Also correct the Sources-table description: "accepted paper" is not supported by the file itself, which shows unfilled template placeholders ("Received on M, D, YYYY / Accepted on M, D, YYYY") rather than real dates.
4. **D3-S21** (Srinivasa et al., MuSHR) — filename says `srinivasa_2019_...` but the actually-downloaded/cited file is v3 (PDF `CreationDate` Dec 2023; citation already says "arXiv:1908.08031v3, 2023"). Rename to `srinivasa_2023_...` or otherwise resolve the year ambiguity.
5. **D3-S23** (Soni et al., OpenPodcar2) — filename says `camara_2026_...` but the paper's actual first author (verified on p.1) is Rakshit Soni — rename to `soni_2026_openpodcar2_ros2.pdf`.
6. **D3-S34** (Veneri & Massaro) — filename says `padua_2020_...` (the authors' university city) instead of the first-author surname. Rename to `veneri_2020_ackermann_race_car_performance.pdf`.
7. **D3-S24** (AgileX Hunter 2.0 manual) — citation attributes authorship to "AgileX Robotics," but the downloaded 62-page PDF is branded "MYBOTSHOP ROBOTICS" throughout, lists only MyBotShop contact details, and explicitly distinguishes "manufacturer (AgileX Robotics) or distributor (MYBOTSHOP)" as two separate parties — i.e. this specific document was compiled/published by the distributor, not verbatim-authored by AgileX. Recommend correcting the citation to credit MyBotShop as the document's publisher (e.g. "MyBotShop, *AgileX Hunter 2.0 User Manual* (AgileX Hunter 2.0 product documentation, published by distributor MyBotShop)"). The Level A grade itself is still defensible (genuine numeric specifications section).
8. **D3-S13** (`nav2docs_configuring_regulated_pp.md`) — strip the embedded `<style>`/video-`<iframe>` HTML block so the saved file is clean markdown, consistent with the sibling Nav2-doc files in this topic, and so `file` no longer reports it as an HTML document.
9. **D3-S17** (JPL rover-maneuvering report) — filename uses the org acronym `jpl_...` rather than the first author's surname (`nesnas_...`), unlike every other individually-authored paper in this topic (Coulter, Snider, Shamah, Zhang, Paden, etc. are all author-named). Low priority / optional, for naming consistency only.

## Summary

44 files / 44 Sources-table rows audited, all present and correctly cross-referenced in both directions (no orphaned files, no filed-but-missing rows). 43 of 44 pass the source-acceptance checklist; one (D3-S25, a vendor sales brochure) fails outright on STANDARDS.md's explicit "marketing pages are not used" rule and should be removed. Ten sources need a correction — four evidence-level fixes (D3-S18, D3-S19, D3-S35, D3-S37, all concrete category mismatches or internal inconsistencies against this same topic's own precedent), four file-naming fixes (D3-S21, D3-S23, D3-S34, and optionally D3-S17), one attribution-precision fix (D3-S24), and one format/cleanliness fix (D3-S13, real content mis-typed by `file` due to an embedded HTML widget, not an error page). All 12 `SCOPE.md` foundational references are present (downloaded-and-cited or correctly marked not-downloaded-with-reason); all 12 subtopics have cited findings; the topic's source base is appropriately general-first (≈76% general vs. ≈24% product-specific). No papers/theses/standards/datasheets were found saved as text where an open PDF exists.
