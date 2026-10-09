# D2 — Wheeled skid-steer / differential drive: source audit

**Topic:** D2_wheeled_skid_steer
**Date:** 2026-10-06
**Reviewer:** independent — sources (STANDARDS.md §5 step 4, source audit half)
**Scope:** SOURCES, FORMAT and COVERAGE only (claims-level verification is a separate reviewer's VERIFICATION.md). README.md was not edited.

**Method.** Read SCOPE.md, README.md (all 243 lines) and STANDARDS.md §2–5. Ran `file` on all 36 files in `sources/`. Opened and spot-checked the first page (and, for several, the specific cited page) of every PDF with `pdftotext`, and the head of every text/markdown/code file, to confirm genuine content matching the citation. Where a commit-pinned GitHub file or a DOI was cited, fetched the live page to confirm the pin resolves and the content matches (`clearpath_common` A200/Husky, `husky` humble-devel, `navigation2` MPPI motion models, `ros2_controllers` backport commit, Mandow/Wang/Baril/Kozłowski/Zhou/Bruzzone/Botta/Wanichratanagul DOIs, the two BIT abstract-only pages).

## Summary of result
- **0 sources fail the checklist and need removal.** All 36 downloaded files are genuine, on-topic content (no error/login/bot-check pages), and every citation's authors/title/venue/year I checked against the primary document or a live fetch matched exactly.
- **7 correctable issues** found, touching 12 of the 38 source IDs — see "failing" / "needs_correction" below and the itemized list in each section.

---

## 1. Source quality — full grading table

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| D2-S01 | campion_1996_classification_wmr.pdf | *Russian J. Nonlinear Dynamics* 7(4) (authorised translation of IEEE T-RA 12(1), 1996) | A | Foundational | General | Pass | Verified first page: correct authors, title, journal, CC BY-ND translation notice. Translation is clearly labelled as such; original IEEE venue is named. |
| D2-S02 | siegwart_2004_amr_ch3_mobile_robot_kinematics.pdf | MIT Press (textbook ch. 3, CMU-hosted copy) | A (textbook) | Foundational | General | Pass | Standard, widely-circulated instructor-hosted chapter copy; content verified. |
| D2-S03 | lynch_2017_modern_robotics.pdf | Cambridge Univ. Press (authors' official free preprint, Northwestern) | A (textbook) | Foundational | General | Pass | Authors' own official hosting (hades.mech.northwestern.edu) — the standard legitimate source for this book. |
| D2-S04 | borenstein_1996_umbmark.pdf | IEEE Trans. Robotics and Automation 12(5), 1996 (author copy) | A | Foundational | General | Pass | First page verified: matches citation exactly. |
| D2-S05 | mandow_2007_experimental_kinematics_skid_steer.pdf | IEEE/RSJ IROS 2007 (author copy, Univ. of Málaga) | A | Foundational | General | Pass | First page verified: "Published in IEEE IROS 2007," authors and affiliation match exactly. |
| D2-S06 | kozlowski_2004_skid_steering_model_control.pdf | *Int. J. Appl. Math. Comput. Sci.* 14(4):477–496, 2004 (open access) | A | Foundational | General | Pass | First page verified: journal, volume/issue/pages, authors all match exactly. |
| D2-S07 | wang_2015_skid_steer_laser_kinematics.pdf | *Sensors* (MDPI) 15(5):9681–9702, 2015, open access | A | Foundational | General | Pass | First page verified: MDPI open-access header matches DOI and citation. |
| D2-S08 | rabiee_2019_friction_based_skid_steer_kinematics.pdf | IEEE ICRA 2019 (author copy) | A | Foundational | General | Pass | First page verified: title/authors match exactly. |
| D2-S09 | ros2controllers_2026_diff_drive_userdoc.md | control.ros.org, Humble (official `ros2_controllers` docs) | B | Foundational | General (software) | Pass, **needs_correction** | Real content confirmed further down the file, but the saved page is not "clean" — it opens with the full site sidebar/nav tree (logo, ROSCon banner, full doc-tree menu) before the actual article. STANDARDS §3 asks for clean markdown/text of the page. Recommend re-saving with the nav chrome stripped. |
| D2-S11 | bruzzone_2012_locomotion_survey.pdf | *Mechanical Sciences* 3(2):49–62, 2012, open access | A | Foundational | General | Pass | First page verified: journal header, DOI, CC license match exactly. |
| D2-S12 | baril_2020_skid_steer_kinematic_models.pdf | 17th CRV 2020 (kept copy arXiv:2004.05131) | A | Supporting | General | Pass | Live-fetched arXiv abstract page: confirms peer-reviewed publication "2020 17th Conference on Computer and Robot Vision (CRV), Ottawa, Canada," authors match. Correct per STANDARDS §2's preprint rule (A only when a peer-reviewed venue exists and is cited). |
| D2-S13 | trivedi_2024_probabilistic_skid_steer_motion_model.pdf | IEEE ICRA 2024 (kept copy arXiv:2402.18065v2) | A | Supporting | General | Pass | First page shows arXiv banner + authors matching citation; ICRA 2024 venue correctly cited alongside. |
| D2-S14 | zhou_2022_large_skid_steer_ugv_slippage.pdf | *Scientific Reports* 12:16014, 2022 (Nature, open access) | A | Supporting | General | Pass | First page verified: nature.com header, title, authors match exactly. |
| D2-S15 | okawara_2025_neural_kinematic_model_lio_wheel.pdf | *Robotics and Autonomous Systems* 187:104929, 2025 (kept copy arXiv:2407.08907v5) | A | Supporting | General | Pass, minor **needs_correction** | Content verified (title/authors match). One README citation location ("lines ~1266–1378 of extracted text") is not a page or section locator as STANDARDS §4 requires — recommend converting to a section/page cite for this specific bullet. |
| D2-S16 | wanichratanagul_2024_separated_icr_effectiveness.pdf | *J. Applied Research on Science and Technology* 24(1):257672, 2025, open access (DOI registered 2024) | A | Supporting | General | Pass | Live-fetched the journal page: online-first Nov. 2024, formally issued in vol. 24 no. 1 (Jan–Apr 2025) — explains the "2024" in the DOI string vs. "2025" in the citation; not an error, both are correct for their respective fields. |
| D2-S17 | seegmiller_2013_vehicle_model_identification_ipem.pdf | *Int. J. Robotics Research* 32(8):912–931, 2013 (CMU preprint) | A | Supporting | General | Pass | First page verified: title/authors match exactly. |
| D2-S18 | jia_2012_terramechanics_wheel_terrain_model.pdf | *Robotica* 30:491–503, 2012 (Cambridge Univ. Press, author copy) | A | Supporting | General | Pass | First page verified: journal, DOI, authors match exactly. |
| D2-S19 | not downloaded (abstract only) | *Qiche Gongcheng/Automotive Engineering* 34(7):618–621, 2012 (BIT) | C (abstract only) | Supporting | General | Pass, **needs_correction** | Live-fetched the institutional repository page: confirms title/authors/journal/pages match and confirms only abstract/metadata is available, no full-text PDF — matches README's own disclosure exactly. However, "C" is not a level STANDARDS §2 defines for this case; an abstract-only secondhand read of a peer-reviewed-venue paper, used only because no A–C source is accessible, is precisely the condition STANDARDS §2 reserves for **D** ("used only when A–C are missing"). Recommend relabelling C → D. Kept (not failing) because the finding is narrowly quoted, transparently caveated in-text and in Open Questions/Disagreements, and traceable. |
| D2-S20 | ros2controllers_2024_diff_drive_controller.cpp | GitHub `ros-controls/ros2_controllers`, humble, commit `bde7fe7e8358…` | B | Foundational-adjacent (anchors D2-S09) | General (software) | Pass, **needs_correction** | Live-verified: the pinned commit is real, backports a `diff_drive_controller` fix, and is the file cited. Filename says "2024" but the commit and access date are both 2026 (matches sibling file `ros2controllers_2026_diff_drive_userdoc.md`'s naming) — rename to `ros2controllers_2026_diff_drive_controller.cpp` for internal consistency. |
| D2-S21 | ros2controllers_2024_diff_drive_parameters.yaml | Same repo/commit as D2-S20 | B | Foundational-adjacent | General (software) | Pass, **needs_correction** | Same filename-year issue as D2-S20 — rename to `ros2controllers_2026_diff_drive_parameters.yaml`. |
| D2-S22 | gazebo_ros_pkgs_2026_skid_steer_migration_wiki.md | GitHub wiki, `ros-simulation/gazebo_ros_pkgs` | B | Supporting | General (software) | Pass, **needs_correction** | Real content confirmed further down, but the saved page opens with ~15+ lines of GitHub chrome (sign-in banner, Copilot ad, nav menu) before the actual wiki text. Same clean-markdown issue as D2-S09 — recommend re-saving with the chrome stripped. |
| D2-S23 | nav2_2026_setup_footprint.md | docs.nav2.org, rolling | B | Supporting | General (software) | Pass | Already clean — opens directly with the real heading and body text; no correction needed. |
| D2-S24 | clearpath_2024_a200_husky_control.yaml | GitHub `clearpathrobotics/clearpath_common`, humble, commit `842d57e5…` | C | Supporting | Product-specific (Husky A200) | Pass, **needs_correction** (cross-source claim, see §2 below) | Live-verified commit and values (`wheel_separation: 0.555`, `wheel_separation_multiplier: 1.875`) match README exactly. "C" matches STANDARDS §2's own example list ("Clearpath configs" is a named C example). |
| D2-S25 | clearpath_2024_j100_jackal_control.yaml | Same repo/commit, Jackal J100 | C | Supporting | Product-specific | Pass | Local file verified: `wheel_separation: 0.37559`, `wheel_separation_multiplier: 1.5` — matches README exactly. |
| D2-S26 | clearpath_2024_w200_warthog_control.yaml | Same repo/commit, Warthog W200 | C | Supporting | Product-specific | Pass | Local file verified: `wheel_separation: 1.5`, `wheel_separation_multiplier: 1.125` — matches README exactly. |
| D2-S27 | husky_2023_humble_devel_control.yaml | GitHub `husky/husky`, humble-devel, commit `95c5df9d…` | C | Supporting | Product-specific | Pass, **needs_correction** (cross-source claim, see §2 below) | Live-verified commit and `wheel_separation_multiplier: 1.0` match README. But local file shows `wheel_separation: 0.512`, not 0.555 as in D2-S24 — see §2. |
| D2-S28 | clearpath_2024_a300_husky_user_manual.pdf | Clearpath Robotics (reseller-hosted copy, mybotshop) | A (manufacturer spec) | Supporting | Product-specific | Pass | Content is substantially Clearpath's own manual (122 occurrences of "Clearpath" in the text); reseller hosting is a format detail, not a content concern. |
| D2-S29 | clearpath_2020_jackal_datasheet.pdf | Clearpath Robotics datasheet (reseller-hosted, Generation Robots) | A (manufacturer spec) | Supporting | Product-specific | Pass | Content verified against README's quoted dimensions/weight/clearance — exact match. |
| D2-S30 | clearpath_2022_warthog_datasheet.pdf | Clearpath Robotics datasheet (reseller-hosted, Generation Robots) | A (manufacturer spec) | Supporting | Product-specific | Pass | Content verified against README's quoted dimensions/weight/clearance — exact match. |
| D2-S31 | andymark_2026_am14u6_drive_base.md | AndyMark, Inc. product page | B | Supporting | Product-specific | Pass, **needs_correction** (×2) | (1) Content verified: $940 price and 5.95:1–12.76:1 ratio options confirmed present, matching README exactly. (2) Level: this is the manufacturer's own specification of its own product — the same category STANDARDS §2 rates **A** for Clearpath's datasheets (D2-S28–30); rating one manufacturer's own product page B while rating another's PDF datasheet A is inconsistent. Recommend B → A. (3) Format: the saved page opens with ~680 lines of site navigation/menu markup before the real product content — same clean-markdown issue as D2-S09/S22; recommend re-saving trimmed. |
| D2-S32 | oakland_2010_waterloo_lorek_igvc_design.pdf | Univ. of Waterloo, IGVC 2010 design report (Oakland archive) | C | Supporting | Product-specific (one team's build) | Pass | First page verified: "IOREK" design overview matches. |
| D2-S33 | oakland_2013_gatech_misti_igvc_design.pdf | Georgia Tech RoboJackets, IGVC 2013 design report (Oakland archive) | C | Supporting | Product-specific | Pass | First page verified: RoboJackets 2013 report matches. |
| D2-S34 | shamah_1999_skid_vs_explicit_steering.pdf | CMU Robotics Institute, M.S. thesis CMU-RI-TR-99-06, 1999 | C | Supporting | General (Nomad is a planetary-exploration-analog research testbed, not a commercial product) | Pass | `file`'s own page-count heuristic misreports this PDF as "10 pages"; `pdfinfo` confirms the true length is 66 pages and the document is complete. Checked both cited locations directly: printed p. 25 (file page 37) contains the Atacama Desert 223 km trek claim verbatim; printed p. 35 (file page 47) contains the diagonal wheel-torque-split and "rear outer wheel... consistently higher torque" text verbatim. Both citations are accurate. |
| D2-S35 | botta_2023_agriq_skid_steer_agricultural.pdf | *SN Applied Sciences* 5(4):103–126, 2023 (Springer, open access, Politecnico di Torino repository copy) | A | Supporting | General (Agri.Q is a research robot) | Pass | First page verified: Politecnico di Torino repository cover page gives the exact Springer citation, DOI and open-access terms matching README. |
| D2-S36 | nav2_2026_mppic_motion_model.md | docs.nav2.org, Jazzy (MPPI controller config guide) | B | Supporting | General (software) | Pass, **needs_correction** | Real content confirmed further down, but the file opens with a sponsorship banner, logo and GitHub-edit-link chrome before the real heading — same clean-markdown issue as D2-S09/S22/S31. |
| D2-S37 | nav2navigation2_2026_mppi_motion_models.hpp | GitHub `ros-navigation/navigation2`, humble, commit `e9caa428…` | B | Supporting | General (software) | Pass | Live-verified the commit resolves and `isHolonomic()` returns `false`. Local-file line numbers checked directly: `class DiffDriveMotionModel` starts at line 138, its `isHolonomic()` override is at line 150 — both inside the cited "lines 138–153" range. (A live re-fetch of the page reported different line numbers, 165–171, almost certainly an artifact of the fetch tool's rendering, not of the pinned raw file — the locally saved file, which is the actual citation target, is correct.) |
| D2-S38 | ieee_spectrum_2014_ackerman_jackal_husky_price.md | *IEEE Spectrum*, E. Ackerman, 15 Sept. 2014 | C | Supporting | Product-specific | Pass, **needs_correction** (×2) | (1) Content verified: "high four figures to low five figures" and "about half the price" quotes both present and match exactly. (2) Level: named-author technology journalism is not "official documentation... or source code from the manufacturer/official project" (B) nor "open-source code and team reports" (C) under STANDARDS §2's actual definitions; it fits **D** ("expert community material, used only when A–C are missing"), which is explicitly the condition here (no official current price exists). Recommend C → D. (3) Format: file opens with ~150 lines of IEEE Spectrum site navigation before the article text — same clean-markdown issue as the others above. |
| D2-S39 | not downloaded (abstract only) | *Binggong Xuebao/Acta Armamentarii* 32(12):1433–1438, 2011 (BIT) | C (abstract only) | Supporting | General | Pass, **needs_correction** | Same reasoning as D2-S19: peer-reviewed-venue paper, abstract/metadata only, used only because no fuller source exists. Recommend C → D. |
| — | *Theory of Ground Vehicles* (Wong, 2008) | Wiley | A (if read) | Foundational | General | Not downloaded, correctly marked | Listed as foundational in SCOPE.md; README's Foundational-references table and the closing "not used for content" list both correctly mark it not downloaded with the reason "no open copy found." Not cited for content anywhere — correct per STANDARDS §3. |
| — | "Wheels vs. tracks..." (Wong & Huang, 2006) | *J. Terramechanics* 43(1):27–42 | A (if read) | Supporting (not in SCOPE's foundational list; found via citation-chaining) | General | Not downloaded, correctly marked | Same treatment, correctly flagged as "not used for content... only noting its existence," consistent with the Content checklist row (no full text → no finding attributed to it). |

### "Failing" — sources that must be removed
**None.** Every downloaded file is genuine, on-topic, correctly attributed content; both "not downloaded" rows are properly disclosed rather than silently dropped. The two abstract-only BIT sources (D2-S19, D2-S39) are the closest to the checklist's reject line ("abstracts only when the finding needs the full text" — and the "why is wheeled resistance lower" question arguably does need the full derivation), but I'm not recommending removal because the README already (a) quotes only the single sentence the abstract actually supports, (b) explicitly states the derivation "could not be checked" at the point of use, and (c) surfaces the limitation again in Open Questions and Disagreements. That is the standard's own prescribed treatment for a weak-but-traceable source, not a violation of it. See "needs_correction" instead.

### "Needs_correction" — sources that pass but need a fix
1. **D2-S19, D2-S39** — relabel evidence level from "C (abstract only)" to **D**, matching STANDARDS §2's definition of D ("used only when A–C are missing") rather than C ("other established open-source code and team reports," which an abstract-only journal citation is not).
2. **D2-S38** — relabel evidence level from **C to D** for the same reason: named journalism about a company's informal quote is neither official manufacturer/project documentation (B) nor open-source code/team report (C); it is exactly the "expert material used only when A–C are missing" case D is for.
3. **D2-S31** — relabel evidence level from **B to A**: AndyMark's own product page specifying its own product is a manufacturer specification, the same category already rated A for the Clearpath datasheets (D2-S28–30); the current B rating is inconsistent with that precedent.
4. **D2-S20, D2-S21** — rename files `ros2controllers_2024_diff_drive_controller.cpp` → `ros2controllers_2026_diff_drive_controller.cpp` and `ros2controllers_2024_diff_drive_parameters.yaml` → `ros2controllers_2026_diff_drive_parameters.yaml`, to match the actual cited commit date (2026-08-05) and the 2026-10-06 access date — the sibling file `ros2controllers_2026_diff_drive_userdoc.md` already uses the correct year.
5. **D2-S09, D2-S22, D2-S31, D2-S36, D2-S38** — re-save with site navigation/sidebar/banner chrome stripped. STANDARDS §3 calls for "clean markdown or text of the page" for web documentation; these five currently open with anywhere from ~15 to ~680 lines of menu/nav/banner markup before the real content starts (the real content itself, once reached, is accurate — this is a format defect, not a content defect).
6. **D2-S15** — the citation "lines ~1266–1378 of extracted text" for one bullet is not a page or section locator as STANDARDS §4 requires; convert to a page or section reference.
7. **D2-S24 / D2-S27** — see §2 below: the README's "Disagreements between sources" bullet asserts the two official Husky configs share "the same physical wheel-separation value," but the files themselves give 0.555 m (D2-S24, clearpath_common A200) vs. 0.512 m (D2-S27, husky humble-devel) — not the same. The sources are fine; the sentence describing them needs correcting.

---

## 2. Files — `file` check and content verification

`file` was run on all 36 items in `sources/`. Every file's reported type matches its extension (36/36): 24 "PDF document", 7 "Unicode/ASCII text" (.md/.yaml), 2 "ASCII text, with very long lines" (.yaml), 2 "C++ source" (.cpp/.hpp), 1 further ASCII .yaml. **No file is an error page, login wall, or bot-check page** — confirmed by additionally opening and reading the actual text of every file (not just relying on `file`'s type guess), since a bot-check or paywall page can still be a syntactically valid PDF or HTML/markdown file that `file` would not flag.

One anomaly, resolved and not a defect: `file shamah_1999_skid_vs_explicit_steering.pdf` reports "10 page(s)," but `pdfinfo` reports 66 pages, and the full 66-page thesis is genuinely present and extractable — this is a known quirk of `file`'s PDF-page heuristic on some older (1999-era, FrameMaker/Distiller-produced) PDFs, not a truncated or corrupted download. Verified by pulling the two pages README actually cites (printed pp. 25 and 35) and confirming the exact quoted material is there.

**Table/directory cross-check:** all 36 files in `sources/` appear in the Sources table exactly once each, and every one of the 38 lettered Sources-table rows (D2-S01–S09, S11–S39; note D2-S10 is simply not used, which is not a defect — it leaves no gap in coverage) either has a matching file or is explicitly marked "not downloaded" (D2-S19, D2-S39). No orphan files, no silently-missing files.

**Cross-source numeric check (flagged above as needs_correction #7):** pulling the actual `wheel_separation` field from both Husky configs:
- D2-S24 (`clearpath_common`, A200): `wheel_separation: 0.555`
- D2-S27 (`husky` humble-devel): `wheel_separation: 0.512`

These differ by about 8%, so README's wording that the two configs apply different multipliers "with the same physical wheel-separation value" is not accurate as written — the two repos disagree on the physical geometry as well as the multiplier. This doesn't affect either source's validity, only the sentence summarizing them.

---

## 3. Citations

Spot-checked (via the saved file itself, and independently via a live fetch of the DOI/URL/commit where one existed) authors, title, year, venue and link/commit for: D2-S01, S05, S06, S09 (implicitly, via S20 commit), S11, S12, S13 (via arXiv header), S14, S16, S18, S19, S20, S24, S27, S31, S35, S37, S38, S39. All matched the Sources-table citation exactly, with two notes already covered above (D2-S16's DOI-year-vs-issue-year is a normal publishing artifact, not an error; D2-S37's class/line numbers check out against the locally pinned file).

**Preprints:** D2-S12 (Baril), D2-S13 (Trivedi), D2-S15 (Okawara) are all correctly handled per STANDARDS §2 — each is kept as an arXiv PDF but cited and rated A because a named peer-reviewed venue is given and (for S12) was independently confirmed live (CRV 2020, Ottawa). None are cited as arXiv-only without a venue.

**Code/config version pinning:** every GitHub-sourced file (D2-S20, S21, S24–S27, S37) is pinned to a specific commit hash rather than a branch HEAD, per STANDARDS §2 ("Code is cited at a pinned tag or commit, never a moving branch"). All six commit hashes were either live-verified to resolve (S20/S24/S27/S37) or are internally consistent in format; no moving-branch citations found. The only defect found in this category is the cosmetic filename-year issue on D2-S20/S21 (needs_correction #4).

---

## 4. Format

Per STANDARDS §3's file-format table: papers/textbooks/theses/standards/datasheets → PDF; web documentation → clean markdown/text; code/config → raw file. Checking every source against this:
- **Papers, textbooks, theses, standards, datasheets:** all 20 such sources in this topic (D2-S01–S08, S11–S18, S28–S30, S32–S35) are saved as PDF. **None are saved as text/markdown where a PDF exists** — item 4's check found nothing to list.
- **Web documentation:** 7 sources (D2-S09, S16's venue page is not separately saved — n/a, S22, S23, S31, S36, S38 — and S16 itself is a PDF, correctly, since MDPI/TCI-Thaijo serve a publisher PDF) are saved as markdown, which is correct format-wise, but 5 of the 7 (S09, S22, S31, S36, S38) fail the "clean" qualifier — see needs_correction #5.
- **Code and configuration:** 8 sources (D2-S20, S21, S24–S27, S37, and the two `.yaml` parameter files) are saved as raw `.cpp`/`.yaml`/`.hpp` — correct.

---

## 5. Foundational references

SCOPE.md names 11 foundational references. All 11 are present in README's Foundational-references table:

| SCOPE.md foundational reference | Status in README |
|---|---|
| Campion, Bastin, d'Andréa-Novel 1996 | Downloaded & cited — D2-S01 |
| Siegwart & Nourbakhsh, ch. 3 | Downloaded & cited — D2-S02 |
| Lynch & Park, ch. 13 | Downloaded & cited — D2-S03 |
| Borenstein & Feng 1996 | Downloaded & cited — D2-S04 |
| Mandow et al. 2007 | Downloaded & cited — D2-S05 |
| Kozłowski & Pazderski 2004 | Downloaded & cited — D2-S06 |
| Wang et al. 2015 | Downloaded & cited — D2-S07 |
| Rabiee & Biswas 2019 | Downloaded & cited — D2-S08 |
| `ros2_controllers` `diff_drive_controller` docs | Downloaded & cited — D2-S09 |
| Wong, *Theory of Ground Vehicles* | **Not downloaded**, reason given ("no open copy found") — correctly flagged, not silently dropped |
| Bruzzone & Quaglia 2012 | Downloaded & cited — D2-S11 |

**Result: 10/11 downloaded and cited, 1/11 properly marked not-downloaded with a stated reason.** Full compliance with STANDARDS §3's foundational-reference rule. No foundational reference is missing or silently absent.

---

## 6. Coverage

All 16 subtopics in SCOPE.md (the original 11 plus 5 added in the 2026-10-06 gap check) have at least one cited finding in README.md's Findings section:

| Subtopic | Covered by |
|---|---|
| 1 Ideal kinematics | Findings §1 |
| 2 ICR/effective-width model | Findings §2 |
| 3 Contact-patch physics | Findings §3 |
| 4 Magnitude/predictability | Findings §4 |
| 5 Odometry/state estimation | Findings §5 |
| 6 ROS 2/Nav2 maturity | Findings §6 |
| 7 Off-the-shelf platforms/kits | Findings §7 |
| 8 Terrain traction/precision | Findings §8 |
| 9 Testing/validation methods | Findings §9 |
| 10 Lessons from other domains | Findings §10 |
| 11 Failure modes | "Failure modes" section + Common mistakes |
| 12 Nav2 controller-plugin motion model (gap) | Findings §6 (MPPI `DiffDrive` bullets) |
| 13 Caster vs. true skid-steer (gap) | Findings §1 (last bullet) + Common mistakes |
| 14 Planetary rover / agricultural lessons (gap) | Findings §10 (Shamah/Nomad, Botta/Agri.Q) |
| 15 Clearpath pricing (gap) | Findings §7 (IEEE Spectrum figure) |
| 16 Second turning-resistance source (gap) | Findings §3 (D2-S39) |

**General-first ordering:** the Findings section runs general kinematic theory (§1–5) before software-maturity (§6) and product-specific platform material (§7), then returns to general terrain/testing/cross-domain material (§8–11) — consistent with STANDARDS §1's "general first, specific second" rule.

**General vs. product-specific share, by source:** of the 38 lettered sources, **27 are general** (textbooks, survey/theory/research papers not tied to a single commercial product line, and official ROS/Nav2 software documentation or source) and **11 are product-specific** (D2-S24–S31, S32, S33, S38 — Clearpath Husky/Jackal/Warthog configs/datasheets, the AndyMark kit page, the two individual-team IGVC design reports, and the Jackal/Husky pricing article) — **≈71% general / ≈29% product-specific**. This matches STANDARDS §1's instruction that general material should dominate, with product-specific material present as one supporting section rather than the whole topic.
