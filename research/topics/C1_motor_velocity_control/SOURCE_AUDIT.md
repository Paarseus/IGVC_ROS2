# C1 — Motor velocity control: source audit

| | |
|---|---|
| **Topic** | C1 — Motor velocity control (`README.md`, Sources table C1-S01 to C1-S51; `SCOPE.md`; `sources/`, 69 files) |
| **Date** | 2026-09-28 |
| **Reviewer** | Independent — sources (step 4, source audit; claims are checked separately in `VERIFICATION.md`) |
| **Rules** | `research/STANDARDS.md` sections 2–5 |
| **Method** | `file` and `pdfinfo` on every file; first page / first 300 bytes of every file read to confirm real content; every file matched against the Sources table; citation details checked against the file itself (title page, running header, journal header) and, for doubtful items, a web search; levels compared across the table for consistency with the section 2 level definitions. This audit replaces any earlier one. |

**Result:** 51 sources. **0 fail** the checklist. **14 pass but need a correction** (level, pinning, format or citation details).

## Source quality

Foundational = listed as foundational in `SCOPE.md` / README *Foundational references*. General = principle that carries across vehicles and drives; Product = documentation or code of one vendor/product.

| ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |
|---|---|---|---|---|---|---|---|
| C1-S01 | rev_2026_closed_loop_overview.md | REV Robotics official docs | B | Supporting | Product | Yes | Official GitBook markdown export, real content. |
| C1-S02 | rev_2026_closed_loop_units.md | REV Robotics official docs | B | Supporting | Product | Yes | Real content (markdown with embedded HTML table; `file` says "HTML document" because of that). |
| C1-S03 | rev_2026_feedforward_control.md | REV Robotics official docs | B | Supporting | Product | Yes | As S02. |
| C1-S04 | rev_2026_pid_tuning_getting_started.md | REV Robotics official docs | B | Supporting | Product | Yes | Real content. |
| C1-S05 | rev_2026_velocity_control_mode.md | REV Robotics official docs | B | Supporting | Product | Yes | Real content (short page, 2 kB, complete). |
| C1-S06 | rev_2026_maxmotion_velocity.md | REV Robotics official docs | B | Supporting | Product | Yes | Real content. |
| C1-S07 | rev_2026_closed_loop_getting_started.md | REV Robotics official docs | B | Supporting | Product | Yes | Real content. |
| C1-S08 | rev_2026_sparkmax_parameters.md | REV Robotics official docs | B | Foundational (product, per SCOPE) | Product | Yes | Real content; currency caveat ("partly legacy") is already stated in the citation. |
| C1-S09 | rev_2026_firmware_revlib_release_notes.md | REV GitHub Releases (official) | B | Supporting | Product | Yes | Verbatim release bodies with tag names and dates; traceable per tag. |
| C1-S10 | rev_2026_revlib_*.java (5), rev_2025_revlib_*.java (2) | REV Maven sources jars 2026.0.5 / 2025.0.3 (official) | B | Foundational (product, per SCOPE) | Product | Yes | Pinned by release version; level B matches STANDARDS ("REVLib … source"). |
| C1-S11 | rev_2025_/rev_2026_revlib_example_*_Robot.java (3) | REVrobotics/REVLib-Examples (official) | C | Supporting | Product | Yes, with fix | 2026 files fetched from `main` (moving branch); level should be B (manufacturer's official repository). |
| C1-S12 | rev_2026_neo_v11.md | REV Robotics motor specification | A | Foundational | Product | Yes | Manufacturer specification = A per STANDARDS. |
| C1-S13 | rev_2026_sparkmax_make_it_spin.md | REV Robotics official docs | B | Supporting | Product | Yes | Real content. |
| C1-S14 | rev_2026_neo_locked_rotor_testing.md | REV Robotics official docs | B | Supporting | Product | Yes | Real content; NEO (full-size) curves are images only (noted in VERIFICATION). |
| C1-S15 | wpilib_2026_intro_feedforward.rst | WPILib frc-docs (official) | B | Supporting | Product (FRC) | Yes, with fix | rst source fetched from `main`; pin commit. |
| C1-S16 | wpilib_2026_tuning_flywheel.rst | WPILib frc-docs (official) | B | Supporting | Product (FRC) | Yes, with fix | As S15 (`file` "HTML" = raw-HTML directive for the embedded simulator). |
| C1-S17 | wpilib_2026_common_control_issues.rst | WPILib frc-docs (official) | B | Supporting | Product (FRC) | Yes, with fix | As S15. |
| C1-S18 | wpilib_2026_sysid_*.rst (6) | WPILib frc-docs (official) | B | Supporting | Product (FRC) | Yes, with fix | As S15. |
| C1-S19 | wpilib_2026_feedforward_classes.rst | WPILib frc-docs (official) | B | Supporting | Product (FRC) | Yes, with fix | As S15 (`file` misreads it as "Nim source"; it is rst). |
| C1-S20 | wpilib_2026_sysid_*.hpp/.cpp, wpilib_2026_sysidroutine_example_Drive.java | wpilibsuite/allwpilib @ decbe89 (official) | C | Supporting | Product (FRC) | Yes, with fix | Pinned commit, good. Level should be B (source of the official project), consistent with S10. |
| C1-S21 | wpilib_2022_sysid_issue258_rev_hall_latency.md | GitHub issue comment (SysId contributor quoting REV Support) | D | Supporting | Product | Yes | Second-hand vendor statement; D is correct and allowed because no A–C source gives the NEO filter window. |
| C1-S22 | ctre_2026_phoenix6_*.md (2) | CTR Electronics official docs | B | Supporting | Product | Yes | Real content. |
| C1-S23 | ctre_2022_phoenix5_sensor_velocity.md | CTR Electronics official docs (Phoenix 5) | B | Supporting | Product | Yes, with fix | Phoenix 5 is superseded by Phoenix 6 for Talon FX; citation should state it is legacy (still current for Talon SRX) to satisfy the currency check. |
| C1-S24 | astrom_murray_2008_feedback_systems.pdf | Princeton University Press (author e-edition v2.11b) | A | Foundational | General | Yes | Full 408-page authorised electronic edition. |
| C1-S25 | merry_2010_encoder_velocity_estimation.pdf | *Mechatronics* (Elsevier) | A | Supporting (proxy for Brown 1992) | General | Yes | Published version, 7 pages, citation correct. |
| C1-S26 | vangeffen_2009_friction_models_compensation.pdf | TU/e traineeship report DCT 2009.118 | C | Supporting | General | Yes, with fix | Named author, TU/e group, report number confirmable; but the link is a third-party mirror (Central Oregon CC course server), no TU/e copy found. Citation should say "third-party mirror". Student report — C is right. |
| C1-S27 | veness_2026_controls_engineering_in_frc.pdf | Self-published book (WPILib maintainer), CC BY-SA | C | Supporting | General (FRC-oriented) | Yes | Not peer-reviewed; C is appropriate; build date pinned. |
| C1-S28 | astrom_2002_ch6_pid_control.pdf | Åström, *Control System Design* lecture notes (unpublished), Caltech CDS 101 host | A | Foundational (proxy) | General | Yes, with fix | Real content by the leading authority, but it is an unpublished manuscript chapter, not an academic-publisher textbook; level A overstates it (compare S26 = C). Grade B/C, or keep A only with an explicit justification. |
| C1-S29 | skogestad_2003_simc_pid_tuning.pdf | *J. Process Control* (Elsevier), author copy | A | Foundational | General | Yes | Citation correct (13:291–309). |
| C1-S30 | galeani_2009_antiwindup_tutorial.pdf | Proc. ECC 2009 (EUCA) | A | Supporting | General | Yes | 18 pages = pp. 306–323; journal version correctly given. |
| C1-S31 | bona_2005_friction_compensation_robotics.pdf | Proc. 44th IEEE CDC–ECC 2005 | A | Foundational | General | Yes | 8 pages = pp. 4360–4367. |
| C1-S32 | cardonasoto_2026_dc_motor_step_ramp_identification.md | *Sensors* (MDPI, DOI) | A | Supporting | General | Yes, with fix | Saved as markdown from PMC XML with equations lost; an open MDPI PDF exists — save as PDF. |
| C1-S33 | siwek_2023_diff_drive_dynamic_identification.md | *Materials* (MDPI, DOI) | A | Supporting | General | Yes, with fix | Same format problem as S32; citation uses "et al." — give the full author list from the article. |
| C1-S34 | toupet_2020_curiosity_wheel_speed_control.pdf | *J. Field Robotics* (Wiley), JPL copy | A | Supporting | General (planetary rover) | Yes | DOI and pages match file header. |
| C1-S35 | seegmiller_2013_vehicle_model_identification.pdf | *IJRR* (SAGE), author preprint | A | Supporting | General (UGV) | Yes | Peer-reviewed version cited; preprint pagination noted. |
| C1-S36 | akrami_2024_low_resolution_hall_sensor_review.pdf | *Energies* (MDPI, DOI) | A | Supporting | General | Yes | Published PDF (20 pages). |
| C1-S37 | moreno_2023_agricultural_robot_speed_control.pdf | *Vehicles* (MDPI, DOI) | A | Supporting | General (agricultural robot) | Yes | Published PDF. |
| C1-S38 | ziegler_1942_optimum_settings.pdf | *Trans. ASME* | A | Foundational | General | Yes | Image-only scan (no text layer; 6 A4-landscape two-page spreads); annotations noted. The URL typo `puublications_others` is the real path. |
| C1-S39 | astrom_1984_relay_autotuning.pdf | *Automatica* (Elsevier), Lund repository | A | Foundational | General | Yes | Repository cover page confirms DOI. |
| C1-S40 | canudasdewit_1995_lugre_friction_model.pdf | *IEEE TAC*, Lund repository (version of record) | A | Foundational | General | Yes | Correct. |
| C1-S41 | peng_1996_antiwindup_bumpless_transfer_extract.pdf + _decoded.txt | *IEEE Control Systems Magazine* 16(4) | A | Supporting (proxy for Fertik & Ross / Hanus) | General | Yes | Abstract + introduction only, clearly labelled; acceptable only because findings are limited to that text. Decoded .txt is a derived aid, disclosed. |
| C1-S42 | wittenmark_2002_computer_control_overview.pdf | IFAC Professional Brief, Lund repository | A | Foundational (proxy for Franklin et al.) | General | Yes | 95 pages, real content. |
| C1-S43 | ljung_2010_perspectives_system_identification.pdf | *Annual Reviews in Control* 34(1):1–12 (tech report LiTH-ISY-R-2989) | A | Foundational (proxy for Ljung 1999) | General | Yes | Report states acceptance in the journal; correct. |
| C1-S44 | forssell_1999_closed_loop_identification.pdf | *Automatica* 35(7) (tech report LiTH-ISY-R-1959) | A | Foundational | General | Yes | Correct; report is the submitted preprint (dated 1997), noted. |
| C1-S45 | sariyildiz_2020_disturbance_observer_overview.pdf | *IEEE TIE* 67(3), arXiv author version | A | Foundational | General | Yes | Peer-reviewed venue given, as required for a preprint. |
| C1-S46 | kawaharazuka_2020_motor_core_temperature.pdf | *IEEE RA-L* 5(3), arXiv author version | A | Supporting | General (humanoid actuators) | Yes | Peer-reviewed venue given. Adding the RA-L DOI would help traceability. |
| C1-S47 | maxon_2026_i2t_winding_protection.md | maxon Support knowledge base (manufacturer) | B | Supporting | Product (other vendor), used for a general principle | Yes | Official manufacturer doc, dated. |
| C1-S48 | freescale_2005_an2988_bus_ripple_cancellation.pdf | Freescale/NXP application note AN2988 Rev 1.2 | B | Supporting | Product (other vendor), used for a general principle | Yes | Relevance is indirect (mains-fed AC induction drive); README already states this. File name describes the topic, not the title — acceptable. |
| C1-S49 | kayacan_2018_tracked_field_robot_traction.pdf | *J. Field Robotics* 35 (Wiley), arXiv author version | A | Supporting | General (tracked field robot) | Yes, with fix | Add issue and DOI: 35(7):1050–1062, DOI 10.1002/rob.21794 (also an accepted manuscript on OSTI, osti.gov/biblio/1466744). |
| C1-S50 | galati_2019_tracked_skid_steer_terrain_awareness.pdf | *Frontiers in Robotics and AI* | A | Supporting | General (tracked vehicle) | Yes | Published PDF, DOI correct; corrigendum noted. |
| C1-S51 | yu_2011_skid_steer_dynamic_power_modeling.pdf | InTech edited book chapter (open access) | A | Supporting | General (skid-steer UGV) | Yes, with fix | InTech edited-volume chapter is not an academic-publisher textbook or clearly peer-reviewed; A overstates it — grade C (or B). The peer-reviewed companion is Yu, Chuy, Collins, Hollis, IEEE T-RO 26(2), 2010 (paywalled). |

### Level consistency across the topic
- Official vendor/project **code** is graded three ways: S10 = B, S11 = C, S20 = C. STANDARDS lists "REVLib, ros2_controllers … source" under B, so S11 and S20 should be B.
- Non-peer-reviewed university documents are graded A (S28, S51) and C (S26). S28 and S51 should come down to match.
- All other levels follow STANDARDS (peer-reviewed / publisher PDFs = A; official docs = B; self-published book = C; issue comment = D).

## Files
- `file` run on all 69 files in `sources/`. All PDFs are real PDFs with the expected content (title pages checked). No error, login or bot-check page remains (the earlier Cloudflare page `olsson_1998_friction_models_compensation.pdf` is gone).
- Type labels that look wrong but are not: GitBook markdown exports with embedded HTML tables (`rev_2026_closed_loop_units.md`, `rev_2026_feedforward_control.md`, `rev_2026_neo_v11.md`, `rev_2026_sparkmax_parameters.md`) show as "HTML document"; files starting with an HTML comment header show as "exported SGML"; `wpilib_2026_tuning_flywheel.rst` has raw-HTML directives; `wpilib_2026_feedforward_classes.rst` is misread as "Nim source"; `rev_202x_revlib_SparkParameters.java` start with a comment, so show as "ASCII text". All were opened and are the expected format.
- `ziegler_1942_optimum_settings.pdf` has no text layer (image scan); it is real content but can only be checked visually.
- Every file is in the Sources table, and every Sources table row has a file. No row is "not downloaded"; the undownloaded foundational works are listed only in *Foundational references*, which is correct because none of their content was used.

## Format
Papers, books or datasheets saved as text where an open PDF exists:

| ID | Saved as | Open PDF |
|---|---|---|
| C1-S32 | markdown from Europe PMC XML (equations lost) | https://www.mdpi.com/1424-8220/26/1/78/pdf |
| C1-S33 | markdown from Europe PMC XML (equations lost) | https://www.mdpi.com/1996-1944/16/2/683/pdf |

C1-S41 is a PDF (abstract + introduction only); the extra `.txt` is a decoded copy of it, not a replacement. All web documentation, rst sources and code files are in the formats STANDARDS asks for. Pinning: frc-docs rst files (S15–S19) and the 2026 REVLib-Examples files (S11) were fetched from `main`; record the commit hash.

## Foundational
| SCOPE foundational reference | Status | OK? |
|---|---|---|
| Åström & Murray 2008 | C1-S24, downloaded, cited | Yes |
| Åström & Hägglund books 1995 / 2006 | Not downloaded; README marks only the 2006 book (in the S28 row). The 1995 book is not marked | Fix: add 1995 to the not-downloaded list |
| Åström & Hägglund 1984 | C1-S39 | Yes |
| Ziegler & Nichols 1942 | C1-S38 | Yes |
| Skogestad 2003 | C1-S29 | Yes |
| Kessler 1958; Preitl & Precup 1999 | Kessler marked not downloaded with reason; Preitl & Precup 1999 not mentioned in README | Fix: mention Preitl & Precup |
| Leonhard 2001 | Not downloaded, reason given | Yes |
| Armstrong-Hélouvry et al. 1994 | Not downloaded, reason given | Yes |
| Canudas de Wit et al. 1995 / Olsson et al. 1998 | C1-S40 downloaded; Olsson marked not downloaded with reason | Yes |
| Fertik & Ross 1967; Hanus et al. 1987 | Not downloaded, reason given; proxy C1-S41 | Yes |
| Brown, Schneider & Mulligan 1992 | Not downloaded, reason given; proxy C1-S25 | Yes |
| Franklin, Powell & Workman 1998 | Not downloaded, reason given; proxy C1-S42 | Yes |
| Ljung 1999 textbook | Not explicitly marked "not downloaded" in README (only implied in the S43 row) | Fix: add an explicit not-downloaded row with reason |
| REV NEO spec, SPARK MAX parameters, REVLib source (product) | C1-S12, C1-S08, C1-S10 | Yes |

## Coverage
| # | SCOPE subtopic | Cited findings? | General sources used |
|---|---|---|---|
| 1 | Loop architecture | Yes | S24, S28, S27 |
| 2 | Motor and drive physics | **Thin** — only the FRC feedforward derivation (S15, S27) and first-order models (S37); no drive textbook; current/torque inner loop only via S22 | none specific |
| 3 | Feedforward design | Yes | S31, S26, S27 (+ product S03, S15, S19) |
| 4 | System identification | Yes | S32, S33, S35, S37, S43 |
| 5 | Tuning methods | Yes, except symmetrical/modulus optimum | S28, S29, S38, S39 |
| 6 | Windup / anti-windup | Yes | S28, S30, S41 |
| 7 | Delay, sampling, velocity estimation | Yes | S24, S25, S28, S29, S36, S42 |
| 8 | Actuator limits | Partly — 8a (S46, S47), 8b (S48 only, AC drive), 8c from general rate-saturation theory; 8d brake/coast only from product source S08 | S46, S30, S28 |
| 9 | Friction, low-speed | Yes | S25, S26, S31, S32, S40 |
| 10 | Other ground vehicles | Yes (wheeled; tracked on wet soil) | S34, S35, S37, S49, S50 |
| 11 | Testing and metrics | Partly — *How it is tested* table; no general source on measuring speed-loop bandwidth | S29, S32, S38, S39, S43 |
| 12 | Product specifics | Yes | product sources S01–S23 |
| 13 | Disturbance observers | Yes | S45 |
| 14 | Closed-loop identification, validation | Yes | S43, S44 |
| 15 | Terrain load, per-side saturation | Partly — no grass data | S49, S50, S51 |

**General first:** yes. The general sections (loop structure → wheel/track control on other vehicles) come before the single product-specific section, which is last, as STANDARDS §1 requires.

**Share of sources:** 26 of 51 general (51%: S24–S46, S49–S51); 25 of 51 product-specific (49%: S01–S23 REV/WPILib/CTRE, plus S47 maxon and S48 Freescale, which are other-vendor documents used for general principles). By citation count the product-specific share is higher (REV/WPILib sources are cited most heavily, e.g. S18 31×, S10 20×).

**Subtopics without adequate cited coverage:** 2 (motor/drive physics), 5a symmetrical/modulus optimum, 8b/8d (bus-voltage compensation and brake/coast from general sources), 11 (speed-loop bandwidth measurement), 15 (grass). All are already recorded as Open questions in the README.
