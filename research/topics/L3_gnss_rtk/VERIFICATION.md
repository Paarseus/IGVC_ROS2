# L3 — GNSS and RTK: claim verification

| | |
|---|---|
| **Topic** | L3 — GNSS and RTK (`research/topics/L3_gnss_rtk/README.md`) |
| **Date** | 2026-09-28 |
| **Reviewer** | independent — claims |
| **Method** | Every cited item opened at the cited location: PDFs via `pdftotext -layout`, split on form feeds, so page = PDF page; code/text files read at the cited line numbers; the scanned Remondi PDF (L3-S52) read as an image. Sources themselves are not graded here (see `SOURCE_AUDIT.md`). |

## Counts

| Section | Items | Verified | Partly supported | Not supported |
|---|---|---|---|---|
| Summary | 6 | 5 | 1 | 0 |
| Foundational references (downloaded rows) | 14 | 13 | 1 | 0 |
| Findings §1–§12 | 168 | 156 | 12 | 0 |
| Recommended practice | 14 | 14 | 0 | 0 |
| Key numbers | 23 | 20 | 3 | 0 |
| How it is tested | 9 | 9 | 0 | 0 |
| Common mistakes | 11 | 10 | 1 | 0 |
| Disagreements | 7 | 7 | 0 | 0 |
| Open questions (cited) | 4 | 4 | 0 | 0 |
| **Total** | **256** | **238** | **18** | **0** |

## Items that need correction (Partly supported)

| # | Short | Correction |
|---|---|---|
| S1 | Summary: differencing / 2.2 km example | "over short baselines" is not in L3-S37 p. 25. Also, the 2.2 km / 7-satellite setup is stated for Fig. 1, and the Fig. 2 scatter experiment only implies it. Cite L3-S11 pp. 28, 32 for the baseline dependence and phrase the 2.2 km link as the same example. |
| F14 | L3-S01 "receiver family inside the MTi-680G" | No source says the MTi-680G's internal receiver is a ZED-F9P. Sources only say "u-blox" and that the MTi-680 (external) can use a ZED-F9P. Mark it as an inference. |
| 2.3 | 2.2 km float/fixed scatter | Same as S1: 2.2 km, dual-frequency and 7 satellites belong to Fig. 1 (formal precision). The 100-run, 5 s scatter experiment (Fig. 2) does not restate the baseline. Label the link as an inference. |
| 2.8 | TurboEdit wide-lane/ionospheric | Wide-lane and ionospheric combinations are described on p. 2, and the abstract on p. 1 does not name them. Cite pp. 1–2. |
| 3.9 | Motion shortens TAR | The source studies **gentle wavelength-scale random antenna motion (2–5 cm/s)** of a hand-held phone, not vehicle motion. Replace "vehicle motion". |
| 3.14 | MTi-680G RTK convergence | The datasheet footnote "Using GPS + GLONASS + Galileo + BeiDou" is attached to cold-start acquisition (24 s), not to RTK convergence. RTK "< 10 s" carries only the footnote "< 30 s for GPS only". The multi-constellation condition for < 10 s is an inference. |
| 4.12 | Antenna calibration 11 % / 15 % | The source measured the 11 % / 15 % reduction "in open-sky conditions", not in the urban-vehicle system. Add that condition. |
| 4.13 | Radomes, a reason for absolute PCV | The source says radome effects of several cm were "more or less ignored up to November 2006" and that the IGS began to consider them **at the same time as** it adopted absolute corrections. Nothing says radomes were a reason for the move. Drop the causal clause. |
| 9.7 | P_F 2.4 % "because errors not independent Gaussian" | p. 8 gives only the numbers. The explanation (multipath and foliage make empirical P_F exceed the target set under Gaussian assumptions) is on p. 4, with a thick-tailed-error remark on p. 6. Cite pp. 4, 6, 8. |
| 9.20 | Availability = PL below AL | L3-S46 p. 3 literally says availability is "how often our protection levels are larger than our alert limits" (apparently a typo). Fig. 5 on p. 4 ("AL < PL resulting in no availability") supports the README's reading. Cite p. 4, Fig. 5, and note the p. 3 wording. |
| 10.8 | navsat assumes antenna at origin when TF missing | This holds only for the initial Cartesian pose (lines 551–560), which logs only if `frame_id` is non-empty, and for the empty-`frame_id` warning (611–615). On the per-fix path `getRobotOriginWorldPose` (564–603), a failed lookup logs "Will not remove offset of navsat device from robot's origin" and leaves the output pose at identity (line 568). It does not substitute the antenna pose. Reword. |
| 10.9 | UTM or local ENU | The cited lines 633–640 call `LLtoUTM` only. The local-ENU option is at lines 102, 337, 438–439 and 848–851. Fix the line citation. |
| 11.5 | Pesyna Monte-Carlo TAR | The Monte-Carlo batch method is described on p. 7, which is not cited (pp. 5–6, 9 are cited). Add p. 7. |
| 11.6 | Odolinski zero vs short baseline | The zero-baseline comparison is on p. 6, and pp. 1 and 4 cover only LS-VCE and the stochastic model. Add p. 6. |
| K3 | MTi-680G RTK horizontal 0.01 m + 1 ppm; "1 km baseline" | The 1 km baseline / patch-antenna footnote is on p. 15 (and L3-S33 p. 14) and attaches to that table, not to the p. 13 CEP table. Cite p. 15 for the condition. |
| K4 | MTi-680G convergence, "multi-constellation" | Same as 3.14: the condition is not attached to the convergence value in the source. |
| K19 | Station-to-caster latency < 2 s (IGS and EUREF) | The EUREF latency statement is on p. 16, not p. 17, and reads "A latency of two seconds or less from station to data centre is acceptable". Fix the page and wording. |
| C4 | Common mistake: empty frame_id / missing TF → origin assumed | Same as 10.8. |

No item was found Not supported.

## Uncited factual statements
- Open question 7 ("No open standard or REP defines how ROS drivers should map RTK fixed/float into `NavSatStatus`; drivers disagree") has no citation. It is consistent with L3-S30, L3-S43 and L3-S44, but a negative claim about all standards cannot be verified from the sources.
- The seven "not downloaded" rows of the Foundational references table make bibliographic claims (for example "No open copy") with no source. They cannot be checked here.
- The other open questions (1, 4, 8) state gaps in the sources read and are not factual claims about the field.

## Full claim table

| # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|
| S1 | Differencing removes clock/orbit/atmos. "over short baselines" but not multipath; float m-level → fixed ~1 cm in 2.2 km LAMBDA example | L3-S03 p. 3; L3-S37 p. 25 | Partly supported | S37 p25 "can compensate for errors due to the satellite clock, the satellite orbit, the ionosphere and troposphere. However, it does not compensate for multipath"; S03 p3 float "spread amounts to several metres", fixed "about 1 cm" | See above |
| S2 | TTF depends on baseline, multipath, visibility; "1 cm + 1 ppm"; NGS ≤ 20 km, latency ≤ 2 s | L3-S01 p. 18; L3-S11 pp. 28, 42, 44 | Verified | S01 p18 "affected by the baseline length as well as by multipath and satellite visibility at both rover and base"; S11 p28 "(≤ 20 km)", p42 "1 cm + 1 ppm", p44 "no greater than 2 seconds" | — |
| S3 | Antenna often limiting; smartphone −11 dB; multipath main barrier; ≥ 44 dBHz | L3-S23 pp. 1, 5; L3-S02 p. 5 | Verified | S23 p5 "on average 11 dB"; p1 "antenna to be the primary impediment"; S02 p5 "at least 44 dBHz"; "Even the best receiver cannot bring back what has been lost due to a poor antenna" | — |
| S4 | Urban P_F 2.4 % vs 0.1 % target; 7.7 % at 15 km; ratio test not correctness → fixed-failure-rate | L3-S22 pp. 8–9; L3-S05 pp. 3, 12; L3-S06 p. 11 | Verified | S22 p8 "PF = 2.4%… factor of 24 larger than P̄F = 0.1%"; Table I 15 km PF 7.7; S05 p12 "should be used with a fixed failure rate" | — |
| S5 | Highway RTK fixed 49.9 %; 95th-pct RTK outages > 1 min vs < 7 s standalone | L3-S21 pp. 14–16 | Verified | p15 Table 7 "49.9%"; p16 "in the 95% of cases, outages for RTK fixed and float can be longer than a minute, compared to … <7 seconds" | — |
| S6 | navsat removes offset via base_link→frame_id TF; accepts status ≠ NO_FIX; copies covariance; NavSatStatus has no RTK | L3-S29 lines 517–561, 606–652; L3-S30 lines 7–10 | Verified | cpp 525–527 lookup base_link→gps_frame_id_; 619–622 `status != STATUS_NO_FIX`; 644–648 copies `position_covariance`; msg 7–10 only NO_FIX/FIX/SBAS/GBAS | — |
| F1 | L3-S52 Remondi 1985 NOAA memo, origin of kinematic carrier phase | L3-S52 | Verified | PDF p5: title, "NGS… NOAA", "centimeter-level relative surveys can be performed in seconds" | — |
| F2 | L3-S03 open LAMBDA description (float, decorrelation, search, fixed) | L3-S03 | Verified | pp1–2 two-step LAMBDA and float/fixed definitions | — |
| F3 | L3-S04/05/06 success rate, partial AR, fixed-failure-rate | L3-S04, S05, S06 | Verified | S04 p8 partial AR; S05 p1 fixed failure rate; S06 p1 "model-driven ratio test with fixed failure rate" | — |
| F4 | L3-S13 open NTRIP 1.0 description from the developing agency | L3-S13 | Verified | S13 p3 protocol description; S39 p2 "Ntrip project was initiated by… BKG" | — |
| F5 | L3-S38/S39 NTRIP paper + RTCM NTRIP 2.0 press release | L3-S38, S39 | Verified | S38 title/authors p1; S39 p1 "RTCM Standard 10410.1" | — |
| F6 | L3-S45 survey of integrity monitoring (RAIM, ARAIM, PL) | L3-S45 | Verified | p6 RAIM/ARAIM; title "review of literature" | — |
| F7 | L3-S12 network RTK via VRS | L3-S12 | Verified | p1–2 VRS concept | — |
| F8 | L3-S11 NGS single-base guidelines: baselines, PDOP, multipath, latency, classes | L3-S11 | Verified | pp28, 37, 44, 50–51 | — |
| F9 | L3-S26 basis of IGS absolute PCV | L3-S26 | Verified | p3 "consistent absolute antenna correction file for the IGS" | — |
| F10 | L3-S17/S15/S16 RTKLIB paper, source, manual | L3-S17, S15, S16 | Verified | files present and matching | — |
| F11 | L3-S10 official SPS accuracy spec | L3-S10 | Verified | p68 Table 3.8-3 | — |
| F12 | L3-S31/S29 REP-105; navsat offset removal | L3-S31, S29 | Verified | REP-105 lines 56–82; navsat 517–603 | — |
| F13 | L3-S01 ZED-F9P Integration Manual R16 | L3-S01 | Verified | footer "UBX-18010802 - R16" | — |
| F14 | L3-S01 is "the receiver family inside the MTi-680G" | L3-S01 (+ Xsens docs) | Partly supported | Xsens docs name only "u-blox" / "internal L1/L2 RTK enabled GNSS receiver" (S32 p11); ZED-F9P mentioned only for the external-receiver MTi-680 | Label as inference |
| 1.1 | Code: few metres; carrier L1 1575.42 MHz ~19 cm; "orders of magnitude more precise" | L3-S09 pp. 52–53 | Verified | p52 "to within a few metres… orders of magnitude more precise"; p53 "L1 at 1575.42 MHz… about 19 centimetres" | — |
| 1.2 | NovAtel error budget ±2/2.5/5/0.5/0.3/1 m | L3-S09 p. 33 Table 1 | Verified | p33 Table 1 values exactly | — |
| 1.3 | Pos error ≈ DOP × range error; DOP > 6 unacceptable for DGPS/RTK | L3-S09 p. 54 | Verified | p54 "Inaccuracy of Position = DOP x Inaccuracy of Range"; "A DOP above 6 results in generally unacceptable accuracies for DGPS and RTK" | — |
| 1.4 | PDOP²=HDOP²+VDOP², GDOP²=PDOP²+TDOP²; more SVs smaller DOP; VDOP>HDOP; latitude, 55° | L3-S08 pp. 3–4 | Verified | p3 "PDOP2 = HDOP2 + VDOP2, and GDOP2 = PDOP2 + TDOP2"; p4 "disparity… larger for higher latitudes… orbits is about 55 degrees" | — |
| 1.5 | SPS ≤ 8 m H / ≤ 13 m V global avg; ≤ 15/33 m worst site (95 %) | L3-S10 p. 68 Table 3.8-3 | Verified | p68 Table 3.8-3 | — |
| 1.6 | SPS excludes iono/tropo residuals, receiver noise/interference, multipath, antenna | L3-S10 p. 42 §2.4.5 | Verified | p42 "Excluded Errors… Residual receiver ionospheric… Receiver noise (including… interference power)… Multipath… User antenna effects" | — |
| 1.7 | Differencing removes clock/hardware; residual orbit/atmos. grows; ~1 ppm | L3-S11 pp. 22, 32 | Verified | p22 "Differencing reduces or eliminates satellite clock errors, receiver clock errors, satellite hardware…"; p32 "specify a 1 part per million (ppm)… (i.e. 1 mm/km)" | — |
| 1.8 | RTK compensates clock/orbit/iono/tropo but not multipath | L3-S37 p. 25 | Verified | p25 quote as S1 | — |
| 1.9 | Output in correction-source frame; datum transform; frame switches on drop to SBAS (few cm of WGS84) | L3-S01 pp. 17, 37, 115 | Verified | p17 "coordinates in the correction source reference frame"; p37 "reference frame… will switch… aligned within a few cm of WGS84"; p115 "(custom) datum transformation is required" | — |
| 1.10 | PPP: one receiver, no nearby station/baseline; convergence "tens of minutes to several hours" | L3-S49 pp. 2–3 | Verified | p3 "need for a nearby reference station and the associated baseline length constrain… It can range from tens of minutes to several hours" | — |
| 1.11 | Static PPP 35/50/60 min for 20/10/5 cm; kinematic longer | L3-S49 pp. 4–5 Table 1 | Verified | p5 Table 1; p4 "much longer for kinematic processing" | — |
| 1.12 | With GIMs, PPP initial fix "still requires a considerable time (more than 10 minutes)" | L3-S49 p. 8 | Verified | p8 exact quote, context "after a receiver cold start… Global Ionospheric Maps" | — |
| 2.1 | Remondi 1985: two TI-4100 surveys; cm in seconds; no a-priori rover coords; static 10–100 km took 1–3 h | L3-S52 PDF p. 5 | Verified | PDF p5 abstract "Two kinematic surveys… using TI-4100s"; intro "1 to 3 hours… base lines 10 to 100 km" | — |
| 2.2 | DD phase integer ambiguous; float → ILS → fixed | L3-S03 pp. 1–2 | Verified | p1 "ambiguous by an unknown integer number of cycles"; p2 three steps | — |
| 2.3 | 2.2 km, 7 SVs, two epochs 5 s apart: float several m, fixed ~1 cm | L3-S03 p. 3 | Partly supported | p3 Fig. 1 "2.2 km baseline… seven satellites"; separate experiment "5 seconds, using only two epochs… several metres… about 1 cm" | Baseline of Fig. 2 experiment not stated; label link |
| 2.4 | Integer decorrelation flattens/lowers conditional variance spectrum | L3-S03 pp. 1, 5 | Verified | p1 "spectrum of sequential conditional ambiguity variances is flattened and lowered" | — |
| 2.5 | Rounding/bootstrapping/ILS one class; ILS highest SR; bootstrapped SR sharp lower bound on decorrelated ambiguities | L3-S04 pp. 1, 5, 8 | Verified | p1 class; p8 Theorem 4 and "very sharp lower bound"; "bootstrapped success rate is very easy to compute" | — |
| 2.6 | Partial AR: add most precise ambiguities until SR unacceptably small | L3-S04 p. 8 | Verified | p8 "continues until… success rate becomes unacceptably small" | — |
| 2.7 | Cycle slip = integer-cycle jump, generally from tracking break | L3-S09 p. 81 | Verified | p81 glossary "jumps by an arbitrary number of integer cycles… break in the signal tracking" | — |
| 2.8 | TurboEdit: undifferenced DF, wide-lane + ionospheric combos; > 2500 slips; 1 % intervention | L3-S07 p. 1 | Partly supported | p1 "undifferenced, dual frequency… over 2500 cycle slips… 1% of the station-satellite passes"; wide-lane/ionospheric combos only on p2 | Cite pp. 1–2 |
| 2.9 | Wide lane 0.862 m, ~6× noise; narrow lane 0.107 m; iono-free for long baselines | L3-S11 p. 22 | Verified | p22 exact values; L3 "most accurate solution on extended baseline lengths" | — |
| 2.10 | Re-search faster if ≥ 2 SVs continuously tracked; complete loss restarts | L3-S11 p. 23 | Verified | p23 "two or more satellites have been continuously tracked… complete loss of lock starts… over again" | — |
| 2.11 | RTKLIB Continuous/Instantaneous/Fix-and-Hold; F&H tight constraint; introduced for moving receivers | L3-S16 pp. 40, 169 | Verified | p40 "ambiguities are tightly constrained to the resolved values"; p169 "introduced in RTKLIB ver. 2.4.0 in order to improve the fixing ratio especially in the kinematic mode" | — |
| 2.12 | Single-epoch insensitive to slips; carrying states forward "perilous", wrong fixes persist several seconds | L3-S22 p. 6 | Verified | p6 "perilous… This cycle, which can persist for several seconds"; "insensitive to cycle slips… common in the urban environment" | — |
| 3.1 | F9P float→fixed convergence affected by baseline, multipath, visibility | L3-S01 p. 18 | Verified | p18 exact quote | — |
| 3.2 | Single base ≤ 20 km; atmosphere can prevent init or wrong fix; solar max inability to initialize | L3-S11 p. 28 | Verified | p28 "(≤ 20 km)… prevent initialization or, worse, … incorrect ambiguity resolution… inability to initialize" | — |
| 3.3 | Mask 10–15° (10° rec.); 3–5× noise low; > 15° loses data, raises PDOP | L3-S11 pp. 32–33 | Verified | p33 "10°- 15°… A 10° mask is recommended"; "(3-5 times more…)"; p32 "higher than 15˚… higher than desired PDOP" | — |
| 3.4 | Network 16 km ≈ single-base < 10 km; single-base 16/32 km worst TTI | L3-S12 pp. 6–7 | Verified | p6 "network-corrected 16km… comparable to… single-baseline RTK on lines less than 10km"; p7 "single-base RTK results for the 16 and 32km lines exhibit the worst" | — |
| 3.5 | Baseline < 4 → 15 km: P_S 84.8 → 78.9 %, P_F 2.4 → 7.7 % | L3-S22 pp. 8–9 Table I | Verified | Table I rows 1 and 10; p9 "no greater than 4 km" | — |
| 3.6 | Sans Galileo P_V 87.2 → 77.4; sans SBAS 78.6 | L3-S22 p. 9 Table I | Verified | Table I rows 13, 11 | — |
| 3.7 | Screening C/N0 ≥ 37.5, sθ ≥ 0.5, elev ≥ 15° | L3-S22 p. 8 | Verified | p8 exact thresholds | — |
| 3.8 | Age 0 → 1000 ms: P_V 87.2 → 87.0; P_F 2.4 → 5.0 | L3-S22 p. 9 Table I | Verified | Table I rows 1, 9 | — |
| 3.9 | "Vehicle motion" shortened TAR for smartphone antenna (400 → 215 s); lengthened for survey-grade | L3-S23 p. 9 | Partly supported | p9 "wavelength-scale random motion of 2-5 centimeters per second… drops from 400 to 215 sec"; "increased TAR for the survey-grade" | Replace "vehicle motion" with small random antenna motion |
| 3.10 | Static multipath decorrelation hundreds of s (100–200 s) | L3-S23 pp. 1, 9 | Verified | p1 "decorrelation times of hundreds of seconds"; p9 "usual 100-200 second correlation time for a static antenna" | — |
| 3.11 | LEA-4T + RTKLIB, 6 km, 5–8 SVs: 56.9 % fix; RMS 3.0/4.9/7.6 cm; mis-fixes | L3-S17 p. 6 | Verified | p6 Table 6; "5 to 8"; "6 km"; "miss-fixed solutions" (p6–7) | — |
| 3.12 | Low-cost L1 GPS + B1 BDS competitive with survey-grade DF GPS | L3-S24 p. 1 | Verified | p1 abstract "can achieve competitive ambiguity resolution and positioning performance to survey-grade dual-frequency GPS" | — |
| 3.13 | Foliage attenuation dB/m, depends on tree; masking → fading, multipath, blockage | L3-S25 p. 6 | Verified | p6 "often characterized as attenuation in dB/m… depends on the nature of the tree and the height"; "fading of the direct signal, multipath, and… blockage" | — |
| 3.14 | MTi-680G RTK convergence < 10 s with GPS+GLO+GAL+BDS, < 30 s GPS only | L3-S32 p. 15 | Partly supported | p15 "Convergence time RTK < 10 [4]"; [4] "< 30 s for GPS only"; [3] "Using GPS + GLONASS + Galileo + BeiDou" attaches to cold start (p14) | Multi-constellation condition is inference |
| 3.15 | 8.9 km single-epoch ILS SR 100/99.1/68.2 % (SF) vs 100/99.4/82.5 % (DF) for Kp 0+/3o/5o; Kp scale | L3-S47 pp. 2, 15–16, Fig. 13 | Verified | p15–16 values; p2 "range between 0o, 0+… up to 9o… strong storms at 7 (G3)" | — |
| 3.16 | Instantaneous RTK not possible when iono grows; ~2 min TTFF, ~5 min at Kp 7− | L3-S47 pp. 1, 19 | Verified | p1 abstract; p19 "both models need, on average, around 2 min in TTFF (and about 5 min when a Kp-index of 7- is reached" | — |
| 3.17 | Calm (Kp 1+) 21.8 km instantaneous AR 100 % both | L3-S47 p. 19 | Verified | p19 "similar instantaneous ambiguity resolution performance with ILS SR of 100%" | — |
| 3.18 | 7 stations open sky → urban canyon; F9P + patch cm "only in conditions with sufficient horizon exposure"; geodetic antenna better | L3-S48 pp. 1, 5, 16 | Verified | p5 station descriptions (branches, trees and hill, urban canyon); p16 conclusion quote | — |
| 4.1 | ≥ 44 dBHz for spec; 44–50 dBHz high SVs; 47 dBHz off-the-shelf | L3-S02 p. 5 | Verified | p5 exact | — |
| 4.2 | Open sky up to 50 dBHz; 45 investigate; < 40 poor | L3-S02 p. 32 | Verified | p32 §7.4 | — |
| 4.3 | Patch parallel to horizon; full sky; windows/roofs; far from radiators | L3-S02 p. 6 | Verified | p6 §2.3 | — |
| 4.4 | Patch GP 50×50–70×70 mm²; back lobe; larger reduces | L3-S02 p. 17 | Verified | p17 exact | — |
| 4.5 | Chip antenna 80×40 mm 43.4 dBHz; 24×15 mm 34.7 dBHz | L3-S02 p. 10 | Verified | p10 figure captions | — |
| 4.6 | Active if cable > ~10 cm; no need > 26 dB; > 35 dB short cable may overload | L3-S02 p. 7 | Verified | p7 exact | — |
| 4.7 | F9P active antenna 17–50 dB incl. losses; NF < 4 dB; PCV < 10 mm; ground plane; not under dash/mirror | L3-S01 pp. 90–91 | Verified | p90 "requires an active antenna"; p91 Table 39 + note | — |
| 4.8 | Xsens ≥ 150 mm GP; "must be properly fixed" | L3-S37 p. 25 | Verified | p25 exact | — |
| 4.9 | Smartphone −11 dB (~8 %); patch −0.6 dB; DD residuals 3.4/5.5/11.4 mm | L3-S23 pp. 5–6 | Verified | p5 "11 dB… approximately 8%"; "0.6 dB"; p6 "3.4, 5.5, and 11.4 millimeters" | — |
| 4.10 | Antenna multipath, not chipset, primary impediment | L3-S23 pp. 1, 9–10 | Verified | p1 "not in the commodity GNSS chipset… but in the antenna"; p9–10 conclusions | — |
| 4.11 | B1 code STD 74 cm patch vs 33 cm Zephyr; multipath suppression | L3-S24 p. 4 | Verified | p4 "74 cm for the patch… Zephyr… 33 cm… potentially better multipath suppression" (abstract p1 states it firmly) | — |
| 4.12 | Calibration cut L1/L2 residual STD 11 %/15 % "in an urban-vehicle system"; without it P_F 2.4 → 4.2 % | L3-S22 pp. 8–9 | Partly supported | p8 "reducing… by 11% and 15%, respectively, in open-sky conditions"; Table I row 15 PF 4.2 | Add "in open-sky conditions" |
| 4.13 | Radomes shift PC by several cm, "one reason IGS moved to absolute" | L3-S26 p. 3 | Partly supported | p3 "impact of radomes… can amount to several cm… more or less ignored up to November 2006"; p2 "At the same time as absolute… the IGS began to consider the effect" of radomes | Drop the causal clause |
| 4.14 | −128 dBm below −111 dBm floor; digital equipment broadband to several GHz | L3-S01 pp. 98–99 | Verified | p98 exact; p99 "PCs, digital cameras, LCD screens… up to several GHz" | — |
| 4.15 | In-band mitigations; out-of-band GNSS BPF | L3-S01 p. 99 | Verified | p99 §4.6.2 / §4.6.3 lists | — |
| 4.16 | Jamming reporting; UBX-MON-SPAN low-res spectrum analyzer | L3-S01 pp. 69, 83 | Verified | p69 §3.14.2; p83 "low-resolution RF spectrum analyzer… during customer integration" | — |
| 4.17 | NGS multipath sources to avoid | L3-S11 p. 37 | Verified | p37 "tree canopy, structures within 30 m that are over the height of the antenna, nearby vehicles, nearby metal objects, abutting large water bodies, and nearby signs" | — |
| 4.18 | Tallysman horizontal patch, GP 100–120 mm, circular, unbroken; larger reduces gain | L3-S51 pp. 1–2 | Verified | p1 "optimally mounted in a horizontal plane"; p2 "between 100mm to 120mm… circular ground plane… optimal for the axial ratio… gain… reduced due to surface wave diffraction" | — |
| 4.19 | Cable under GP; separate isolated GPs | L3-S51 p. 2 | Verified | p2 "route the antenna cable underneath"; "separate, isolated ground planes" | — |
| 4.20 | Geodetic antenna better: smaller PCV, better multipath reduction | L3-S48 p. 16 | Verified | p16 "with smaller PCVs (phase center variations) and more efficient multipath reduction" | — |
| 5.1 | Network RTK models regional errors; longer distance, reliability, init time | L3-S12 p. 1 | Verified | p1 "increase the distance… increases the reliability… reduces the RTK initialization time" | — |
| 5.2 | VRS: rover sends GGA; corrections as if from station few m away | L3-S12 p. 2 | Verified | p2 "NMEA position string called GGA"; "situated only a few meters from where any rover is situated" | — |
| 5.3 | VRS two-way; FKP broadcast one-way | L3-S12 pp. 1–2 | Verified | p1 "requires bi-directional communication"; p2 FKP area corrections | — |
| 5.4 | NovAtel: VRS reduces number of base stations | L3-S09 p. 53 | Verified | p53 "overall reduction in the number of RTK base stations required" | — |
| 5.5 | 1 cm + 1 ppm H, 2 cm + 1 ppm V (1σ), manufacturers/ISO | L3-S11 p. 42 | Verified | p42 "many… manufacturers state… (at the 68 percent or one sigma level)… ISO/PRF 17123-8" | — |
| 5.6 | RT1 ≤ 10 km, PDOP ≤ 2.0, ≥ 7 SVs → 0.01–0.02 m; RT4 any fixed baseline, PDOP ≤ 6, ≥ 5 → 0.1–0.2 m (95 %) | L3-S11 pp. 50–51 | Verified | p50–51 class text | — |
| 5.7 | OSR (RTCM) vs SSR PPP-RTK (SPARTN/CLAS) | L3-S01 p. 7 | Verified | p7 exact | — |
| 5.8 | F9P warns baseline > 100 km (50 km HPG 1.12) | L3-S01 p. 20 | Verified | p20 + footnote 9 | — |
| 5.9 | Rover never more accurate than base | L3-S11 pp. 20, 40 | Verified | p40 "can never by RT practice be more accurate than that of the base" | — |
| 5.10 | Base coordinate accuracy affects rover; warning if off by > ~50 m up to 25 km | L3-S01 p. 21 | Verified | p21 exact | — |
| 5.11 | Datum readjustments at network level, broadcast | L3-S11 p. 56 | Verified | p56 item 4 | — |
| 6.1 | F9P RTCM 3.4 input incl. MSM4/5/7, 1005/1006, 1033, 1230 | L3-S01 pp. 17–18 | Verified | Table 6 | — |
| 6.2 | Needs MSM4/MSM7 + 1005/1006; station ID must match; GLONASS float without 1230/1033 | L3-S01 pp. 18–19 | Verified | p18–19 exact | — |
| 6.3 | MTi-680G RTCM v3 "with a standard 1Hz frequency" | L3-S34 p. 32 | Verified | p32 exact | — |
| 6.4 | NTRIP HTTP/1.1-based; Server/Caster/Client; TCP/IP, mobile | L3-S13 p. 3 | Verified | p3 exact | — |
| 6.5 | Caster adapted to 50–500 bytes/s per stream | L3-S13 p. 7 | Verified | p7 "from 50 up to 500 Bytes/sec per" stream | — |
| 6.6 | Unavailable mountpoint → source-table | L3-S13 p. 10 | Verified | p10 "Requesting unavailable mountpoints… caster replying with an up-to-date source-table" | — |
| 6.7 | Basic (base64) or Digest auth | L3-S13 pp. 10–11 | Verified | p10–11 §5.2–5.3 | — |
| 6.8 | nmea=1 → caster must receive ≥ 1 GGA; more at any time (VRS / best stream) | L3-S13 pp. 11, 14 | Verified | p11 exact; p14 field 12 `<nmea>` | — |
| 6.9 | 2009 test NTRIP base input 1.1–1.4 kbps | L3-S17 p. 5 | Verified | p5 Table 5 "Base Input 1.1 - 1.4 kbps NTRIP" | — |
| 6.10 | MTi-680G RTCM port outputs GGA at 1 Hz, "often required by NTRIP providers" | L3-S32 p. 20 | Verified | p20 exact | — |
| 6.11 | u-blox base messages 1005, 1074/84/94/1124, 1230; same rate; 1005/6 less often; no MSM4/7 mix | L3-S01 pp. 21–22 | Verified | p21–22 lists and guidance | — |
| 6.12 | EUREF obs 1 Hz; metadata "at least 60 seconds or higher"; MSM7 rec., MSM5 acceptable | L3-S42 p. 17 | Verified | p17 §3.3.5, 3.3.7, 3.3.8 | — |
| 6.13 | IGS: 1 Hz, latency < 2 s; MSM7; MSM4/5 acceptable | L3-S41 p. 10 | Verified | p10 §2.1(2), §2.2(1) | — |
| 6.14 | NTRIP 2.0 (10410.1, 2009) changes list | L3-S39 p. 2; L3-S40 §3.1.2 | Verified | S39 p2 bullet list; S40 §3.1.2 same list | — |
| 6.15 | NTRIP 2 backward compatible; TCP/TLS (443)/RTSP/UDP; "0.5 sec or less"; proxy limits; use '2' behind proxy | L3-S40 §§2.20.1.1.5, 3.1.2 | Verified | §2.20.1.1.5 exact wording | — |
| 6.16 | GGA may be blocked by proxy/firewall/virus scanner without NTRIP 2; "some VRS systems need GGA sentences at regular intervals" | L3-S40 §§2.18.1, 2.10.11 | Verified | §2.18.1 last sentence; §2.10.11 exact | — |
| 6.17 | 2005: ~5 kbit/s RTK, ~0.5 kbit/s DGNSS; HTTP use "slightly differs"; proxies "should be avoided" | L3-S38 p. 3 | Verified | p3 exact | — |
| 7.1 | F9P stops using RTCM > 60 s (CFG-NAVSPG-CONSTR_DGNSSTO) | L3-S01 p. 19 | Verified | p19 exact | — |
| 7.2 | RTKLIB max age default 30 s | L3-S16 p. 41 | Verified | p41 "pos2-maxage… 30" | — |
| 7.3 | "Stale" 10–15 s; 96.0 % < 2 s, 97.3 % < 10 s; rest minutes old | L3-S21 p. 15 Table 7 | Verified | p15 exact | — |
| 7.4 | Display up to 5 s old (2–3 s typ.); ≤ 2 s rec.; intermittent links degrade | L3-S11 pp. 44, 47 | Verified | p44, p47 exact | — |
| 7.5 | Lost TCP detected by sockets → auto reconnect | L3-S13 p. 5 | Verified | p5 exact | — |
| 7.6 | BNC: > 20 s no data; 1, 2, 4 … 256 s | L3-S14 p. 11 | Verified | p11 exact | — |
| 7.7 | BNC buffers decoder outputs; reports to script | L3-S14 pp. 11–12 | Verified | p11–12 exact | — |
| 7.8 | RTKLIB toinact 10000 ms, ticonnect 10000 ms; source-table → "no mountp. reconnect" | L3-S15 lines 271–272, 1432–1436, 1626–1638 | Verified | 271–272; 1432–1436 timeout; 1633 "no mountp. reconnect..." + 1638 `discontcp` | — |
| 7.9 | MicroStrain: 4 s RTCM, 5 zero reads, 5 GGA fails; 5 s × 10 attempts then exception | L3-S18 client 28, 58–60, 182–197, 220–229; base 10–11, 48–62 | Verified | exact constants at cited lines; base 59 `raise Exception` | — |
| 7.10 | SOURCETABLE → invalid mountpoint; 401 → credentials; CRC-24 per frame | L3-S18 client 13–23, 116–128; parser 78–94 | Verified | 116–121 warnings; parser 80–82 checksum compare (24-bit, lines 114–118) | — |
| 7.11 | Xsens: Ntrip/2.0 header; any "200"; keep-alive; reconnect_delay 5 s unlimited; reconnect on error/close; no RTCM-inactivity timer | L3-S19 cpp 218, 265, 304–316, 328–362, 413–421; yaml | Verified | 218 keep_alive; 265 header; 304 `find("200")`; 353–361 Disconnect; yaml `reconnect_delay: 5.0`, `max_reconnect_attempts: 0 # infinite`; no read-inactivity timer in file | — |
| 7.12 | Forwards only $GPGGA/$GNGGA; 4 Hz → 1 Hz; send failures non-fatal | L3-S19 cpp 365–410; yaml | Verified | 369–370 filter; 382 decimation; 399, 408 `HandleError(…, false)` | — |
| 7.13 | Latencies < 2 s typical (Germany/Europe); delayed data → lack of accuracy | L3-S38 pp. 3–4 | Verified | p4 "Latencies less than two seconds are typical… Germany and in Europe"; p3 "Considerably delayed, missing or irregularly arriving correction data entail a lack of accuracy" | — |
| 7.14 | EUREF: restore flow "as quickly as possible, preferable using an automated procedure" | L3-S42 p. 17 | Verified | p17 §3.3.6 | — |
| 7.15 | BNC 2.13.7 same outage rule; failure threshold default 15 min | L3-S40 §§2.11, 2.11.2 | Verified | §2.11, §2.11.2 exact | — |
| 8.1 | Fixed cm, float dm, code m; only 55 % < 10 cm H | L3-S21 p. 9 | Verified | p9 exact | — |
| 8.2 | Modes 49.9 / 14.1 / 0.3 / 33.0 / 2.7 % | L3-S21 p. 15 Table 7 | Verified | Table 7 | — |
| 8.3 | Fragility inverse to accuracy; median 11 s fixed vs 2 s float; 95th > 1 min vs < 7 s | L3-S21 p. 16 | Verified | p16 exact | — |
| 8.4 | 50 % avail. with continuity-loss 0.54 / 4 s vs 0.045 at 98 % | L3-S21 p. 19 | Verified | p19 exact | — |
| 8.5 | MTi-680G RTK 0.01 m + 1 ppm CEP (< 0.05 dynamic), V 0.1 m + 1 ppm, PVT 1.5 m; 1 km baseline, patch, excl. PCO | L3-S32 pp. 13, 15; L3-S33 p. 14 | Verified | p13 table; p15 PVT 1.5 and footnote [5]. Note: p15 and S33 p14 list RTK vertical 0.01 m, which conflicts with p13 (0.1 m + 1 ppm) | Optionally note the source's internal inconsistency |
| 8.6 | GGA 4 fixed / 5 float; NAV-PVT carrSoln 1/2; GGA age + base ID | L3-S01 p. 19 | Verified | p19 exact | — |
| 8.7 | RTKLIB Q 1 fix, 2 float, 4 DGPS, 5 single | L3-S16 p. 105 | Verified | p105 header line | — |
| 8.8 | NavSatStatus only NO_FIX/FIX/SBAS/GBAS | L3-S30 NavSatStatus lines 7–10 | Verified | lines 7–10 | — |
| 8.9 | KumarRobotics: FIX for valid 2D/3D incl. float; GBAS only if carrier fixed; diagonal cov from h/v acc² | L3-S43 hpp 92–111; NavPVT 55–61 | Verified | 93–97; 105–111 | — |
| 8.10 | Septentrio: DGPS/RTK fixed/float/MB/PPP → GBAS; SBAS → SBAS; GPSFix RTK_FIX/RTK_FLOAT (ROS 2) | L3-S44 1381–1437, 1613–1631 | Verified | 1400–1432; 1615–1627 `#ifdef ROS2` | — |
| 8.11 | position_covariance m², ENU; approximate from DOP | L3-S30 NavSatFix 28–45 | Verified | lines 28–45 | — |
| 8.12 | RMS = precision not accuracy; "false precision" under multipath | L3-S11 pp. 37, 39 | Verified | p39 "statistical measure of precision (not accuracy)"; p37 "false precision" | — |
| 8.13 | map may jump; odom continuous | L3-S31 "odom", "map" | Verified | lines 56–58, 73–78 | — |
| 9.1 | Ratio test not correctness; integer shift invariant | L3-S05 p. 3 | Verified | p3 exact | — |
| 9.2 | Critical values 3 (popular), 1.5, 2, 5–10; no rigorous basis; failure rate varies | L3-S05 pp. 3–5 | Verified | p3–4 list; p5 "failure rate will change from epoch to epoch" | — |
| 9.3 | Fixed c: strong → false alarms, weak → failures; FF ratio test + look-up tables | L3-S06 pp. 1, 4, 11 | Verified | p4 exact; p1 "As its replacement… fixed failure rate… look-up tables" | — |
| 9.4 | FF approach in integer aperture theory; guarantees Pf, shortens TTFF | L3-S05 pp. 1, 7 | Verified | p1 abstract; p7 IA estimation | — |
| 9.5 | RTKLIB ratio default 3.0; "only supports a fixed threshold value" | L3-S16 pp. 40, 168 | Verified | p40 "arthres… 3.0"; p168 exact | — |
| 9.6 | Unavoidable P_S / P_F trade-off | L3-S22 p. 4 | Verified | p4 "unavoidable tradeoff… widening of the integer aperture region" | — |
| 9.7 | P_F 2.4 % (24× 0.1 %) "because real errors are not independent Gaussian"; no bit prediction → 25 % | L3-S22 p. 8 | Partly supported | p8 numbers and "factor of 24"; reason stated on p4 ("multipath… foliage… cause the empirical PF to significantly exceed… Gaussian error assumptions") | Cite p. 4 (and p. 6) for the cause |
| 9.8 | Most manufacturers 99.9 %; long fix may be wrong; no indication beyond RMS | L3-S11 pp. 23, 47 | Verified | p22–23 "99.9 percent"; p47 exact | — |
| 9.9 | Multipath wrong fixes, especially vertical; > 2 dm | L3-S11 p. 37 | Verified | p37 exact | — |
| 9.10 | RTK_CAR conservative mode (HPG 1.50+) "near absolute certainty" | L3-S01 p. 19 | Verified | p19 §3.1.5.4.1 | — |
| 9.11 | PL bounds error at TMIR; exceed AL → change mode or unavailable; use only if valid | L3-S01 pp. 43–45 | Verified | p44 TMIR; p45 "stopping, reversing… slowing down"; Table 25 "shall not be used" | — |
| 9.12 | Lawn mower: slow near boundaries; stop/back up if no valid PL | L3-S01 p. 46 | Verified | p46 "may reduce its speed… If the protection level value fails to compute, the device can stop… or go back" | — |
| 9.13 | χ²-type innovation test; N-choose-1 exclusion; depth 8; reset or marginalize | L3-S22 pp. 6–8 | Verified | p6 "χ2 -type test… excluded one at a time"; p8 "depth of 8 signals… reset or integers marginalized" | — |
| 9.14 | Continuity loss ~10⁻¹ vs 10⁻⁶ aviation | L3-S21 p. 16 | Verified | p16 exact | — |
| 9.15 | F9P + patch fixed yet > 13 m off; confidence "should be alerted"; geodetic kept accuracy or went float | L3-S48 pp. 14, 16 | Verified | p14 "marked as the fixed one while the difference… exceeded 13 m"; p16 conclusions | — |
| 9.16 | Aviation integrity not directly transferable (redundancy, single fault vs multipath/NLOS) | L3-S45 pp. 2–3 | Verified | p2–3 exact | — |
| 9.17 | RAIM ≥ 5 detect / ≥ 6 exclude; zero-mean Gaussian; ARAIM multi-fault, risk allocation | L3-S45 p. 6 | Verified | p6 exact | — |
| 9.18 | KF-innovation FDE better for dynamic platforms but "prone to high false alarm" | L3-S45 p. 13 | Verified | p13 exact | — |
| 9.19 | Freeway 0.57 (0.20) / 1.40 (0.48) / 1.30 (0.43) m; local 0.29 (0.10) m; 10⁻⁸ /h aviation & rail | L3-S46 p. 1 | Verified | p1 abstract exact | — |
| 9.20 | Availability = how often PL below AL; PL > AL → no lane guarantee | L3-S46 pp. 1, 3 | Partly supported | p3 literally "how often our protection levels are larger than our alert limits"; p3 "we cannot guarantee we are within the lane"; p4 Fig. 5 "AL < PL resulting in no availability" | Cite p. 4 Fig. 5; note p. 3 wording |
| 9.21 | navsat accepts any status ≠ NO_FIX with non-NaN coords; copies covariance; no fix-type check | L3-S29 lines 618–651 | Verified | 619–622, 644–648 | — |
| 10.1 | Antenna pos = IMU + C_b^n lever arm; velocity; Earth/transport-rate term negligible | L3-S27 pp. 63–64 Eqs. 2.67–2.68 | Verified | p63 Eq 2.67a; p64 Eq 2.68 "second term… can be neglected in most cases"; loose/tight on pp. 64–65 | — |
| 10.2 | In-motion init: DGPS velocity errors ≤ 1.5 m/s, > 2 m/s extreme, incl. lever arm | L3-S27 p. 171 | Verified | p171 exact | — |
| 10.3 | Xsens lever arm essential; X-Y-Z m, sensor frame; cm accuracy; right → −Y | L3-S37 p. 25; L3-S32 p. 25 | Verified | S37 p25 exact; S32 p25 "essential to the sensor fusion algorithm" | — |
| 10.4 | Chauchat invariant EKF; 7°/s circles then straight 5 m/s; EKF mostly failed | L3-S28 pp. 2, 6 | Verified | p6 "angular velocity of 7◦/s… 5m/s… lever arm to be fully observable"; "EKF mostly fail" (Table I 98 % vs 34 %/14 %) | — |
| 10.5 | Lever arm hard to measure; degrades; 18-state model | L3-S50 p. 2 | Verified | p2 exact | — |
| 10.6 | Lever arm unobservable first 80 s; horizontal converges on yaw rotation | L3-S50 p. 8 | Verified | p8 "not observable for the first 80 s. From second 80, when the system starts to rotate around Z, the horizontal elements… convergent" | — |
| 10.7 | frame_id "usually the location of the antenna", relative to vehicle | L3-S30 NavSatFix 9–12 | Verified | lines 9–12 | — |
| 10.8 | navsat rotates base_link→frame_id offset and removes it; if TF missing / frame_id empty logs error and assumes antenna at origin | L3-S29 lines 517–561, 564–603, 609–615 | Partly supported | 546–550 and 585–589 remove offset; 552–560 origin assumed (log only if frame_id non-empty); 591–602 per-fix path logs "Will not remove offset…" and leaves pose identity (568) | Reword behaviour of per-fix path |
| 10.9 | Lat/lon → UTM (or local ENU); datum lat, lon, heading (rad) | L3-S29 cpp 148–170, 633–640; rst 42–52 | Partly supported | cpp 157–171 datum lat/lon/yaw; 634 `LLtoUTM`; rst 52 "heading in radians"; local ENU only at lines 102, 337, 438–439, 848–851 | Fix line citation for local ENU |
| 10.10 | Yaw adds declination + yaw_offset + meridian convergence; 0 = east | L3-S29 cpp 284–292; rst line 25 | Verified | cpp 291–292; rst 25 "Your IMU should read 0 for yaw when facing east" | — |
| 11.1 | Golden device; simulator 38–40 dBHz; HDOP < 3.0; indoor unreliable | L3-S01 pp. 84–85 | Verified | p84–85 exact | — |
| 11.2 | Cold start 30–40 s avg of 10–20; ripple < 50 mV p-p | L3-S02 p. 32 | Verified | p32 exact | — |
| 11.3 | Metrics d95, P_V, P_S, P_F; P_V = P_S + P_F; 30 cm | L3-S22 pp. 3, 8 | Verified | p3 metrics; p8 "within 30 cm" | — |
| 11.4 | Tactical INS + dual survey receivers + networked RTK truth; accuracy/availability/continuity/age | L3-S21 pp. 1, 6–7 | Verified | p1 "two survey grade GNSS receivers with a tactical grade IMU… networked RTK"; p6 SmartNet; p7 sections | — |
| 11.5 | C/N0 diffs, DD residuals, Monte-Carlo TAR curves | L3-S23 pp. 5–6, 9 | Partly supported | p5–6 C/N0 and residuals; Monte-Carlo batch method p7 "Each separate batch… treated as a Monte Carlo run" | Add p. 7 |
| 11.6 | LS-VCE; zero- vs short-baseline; realistic stochastic model | L3-S24 pp. 1, 4 | Partly supported | p1 abstract LS-VCE and "Otherwise, the ambiguity resolution… would deteriorate"; zero-baseline comparison on p6 | Add p. 6 |
| 11.7 | ~2 h, 76,971 epochs vs DF geodetic static reference; fix ratio and RMS | L3-S17 p. 6 | Verified | p6 exact | — |
| 11.8 | Redundant obs with different geometry; checks on known control | L3-S11 pp. 37–38, 48 | Verified | p37–38 "three-hour staggered times"; p48 "check shot should ALWAYS be taken on a known point" | — |
| 11.9 | Seven stations vs Leica MS50; alternating series for similar geometry | L3-S48 pp. 1, 5, 17 | Verified | p1 MS50; p17 "performed alternately… satellite configuration was as similar as possible" | — |
| 11.10 | RTKCONV → RINEX, RTKPOST modes; post-processing needs no link; network base data; PPP | L3-S16 pp. 31, 52; L3-S09 pp. 56–57 | Verified | S16 p31, p52; S09 p56–57 exact | — |
| 11.11 | BNC logs message/observation types and latencies; host clock sync needed | L3-S40 §§2.12, 2.12.2 | Verified | §2.12 Fig. 21 caption; §2.12.2 "requires the clock of the host computer to be properly synchronized" | — |
| 12.1 | MTi-680G internal L1/L2 RTK receiver; RTCM3 via RS232 port (38400) or Xbus | L3-S32 pp. 11, 20, 28 | Verified | p11, p20 "default baud rate… 38400", p28 exact | — |
| 12.2 | Driver maps StatusWord bits 27–28 (2 → 4, 1 → 5), else 1, 6, 0; never 2 | L3-S20 ntrip_util.cpp 64–103 | Verified | lines 67–103 | — |
| 12.3 | RTK flag 3 states; no filtered pos/vel before GNSS fix; usually within 30 s | L3-S37 pp. 22–23 | Verified | p22 "generally takes up to 30 seconds"; p23 three states | — |
| 12.4 | `GNSS_LeverArm` [0,0,0] for MTi-8/680(G); `ublox_platform` | L3-S20 yaml 123–145 | Verified | lines 127–145 | — |
| 12.5 | Platform model affects pos/vel and Xsens filter output | L3-S32 p. 28 | Verified | p28 exact | — |
| 12.6 | Required antenna 17–50 dB, NF ≤ 4 dB, PCV ≤ 10 mm, RHCP, 150 mm GP | L3-S35 p. 22 Table 18 | Verified | Table 18 + footnote 10 | — |
| 12.7 | Starter kit Tallysman TW8889 27 dB, NF 2.5 dB, 4 dBic, 2.9 m | L3-S36 p. 28 Table 21 | Verified | Table 21 | — |
| 12.8 | Optional pos/vel smoother | L3-S32 p. 26 | Verified | p26 exact | — |
| R1 | Level, sky view, away from radiators/digital, metal GP (≥ 150 mm) | L3-S02 p. 6; L3-S37 p. 25; L3-S01 p. 99 | Verified | as 4.3, 4.8, 4.14 | — |
| R2 | C/N0 ≥ 44, up to 50; investigate ~45 | L3-S02 pp. 5, 32 | Verified | as 4.1, 4.2 | — |
| R3 | Mask 10–15° | L3-S11 p. 33 | Verified | as 3.3 | — |
| R4 | ≤ 20 km; ≤ 10 km RT1; or network RTK | L3-S11 pp. 28, 50; L3-S12 pp. 6–7 | Verified | as 3.2, 5.6, 3.4 | — |
| R5 | Matching RTCM obs + 1005/1006 (+1230/1033) | L3-S01 pp. 18–19 | Verified | as 6.2 | — |
| R6 | Latency ≤ 2 s; caution with older | L3-S11 p. 44; L3-S21 p. 15 | Verified | as 7.3, 7.4 | — |
| R7 | No-data timer + back-off | L3-S14 p. 11; L3-S18 lines 194–197 | Verified | as 7.6, 7.9 | — |
| R8 | Fixed-failure-rate test | L3-S05 p. 12; L3-S06 p. 11 | Verified | as 9.3 | — |
| R9 | Compare PL with AL; degrade when exceeded/invalid | L3-S01 pp. 44–45 | Verified | as 9.11 | — |
| R10 | Measure lever arm to cm; publish base_link→antenna TF | L3-S37 p. 25; L3-S29 517–561 | Verified | as 10.3, 10.8 (initial-pose path) | — |
| R11 | Same rate (1 Hz), one MSM type, metadata ≥ 60 s | L3-S01 p. 22; L3-S42 p. 17; L3-S41 p. 10 | Verified | as 6.11–6.13 | — |
| R12 | NTRIP 2 over TCP behind proxy; periodic GGA for VRS | L3-S40 §§2.20.1.1.5, 2.10.11, 2.18.1 | Verified | as 6.15, 6.16 | — |
| R13 | Cable beneath GP; separate isolated GPs | L3-S51 p. 2 | Verified | as 4.19 | — |
| R14 | Restore lost stream automatically, quickly | L3-S42 p. 17 | Verified | as 7.14 | — |
| K1 | SPS ≤ 8 m H, ≤ 13 m V | L3-S10 p. 68 | Verified | Table 3.8-3 | — |
| K2 | RTK 1 cm + 1 ppm H, 2 cm + 1 ppm V, 1σ | L3-S11 p. 42 | Verified | p42 | — |
| K3 | MTi-680G 0.01 m + 1 ppm CEP; dynamic < 0.05; 1 km baseline | L3-S32 p. 13 | Partly supported | p13 values; 1 km baseline footnote on p15 | Cite p. 15 for condition |
| K4 | MTi-680G convergence < 10 s (< 30 s GPS only), "multi-constellation" | L3-S32 p. 15; L3-S33 p. 14 | Partly supported | S33 p14 footnote 6 (multi-GNSS) on cold start; footnote 7 on convergence | As 3.14 |
| K5 | F9P correction timeout 60 s | L3-S01 p. 19 | Verified | p19 | — |
| K6 | RTKLIB max age 30 s | L3-S16 p. 41 | Verified | p41 | — |
| K7 | Latency ≤ 2 s NGS | L3-S11 p. 44 | Verified | p44 | — |
| K8 | Single base ≤ 20 km | L3-S11 p. 28 | Verified | p28 | — |
| K9 | ≥ 44 dBHz | L3-S02 p. 5 | Verified | p5 | — |
| K10 | GP 50×50–70×70 mm²; ≥ 150 mm Xsens | L3-S02 p. 17; L3-S37 p. 25 | Verified | p17; p25 | — |
| K11 | NTRIP 50–500 bytes/s | L3-S13 p. 7 | Verified | p7 | — |
| K12 | BNC > 20 s; 1 s doubling to 256 s | L3-S14 p. 11 | Verified | p11 | — |
| K13 | Highway RTK fixed 49.9 % | L3-S21 p. 15 | Verified | Table 7; GPS+GLONASS L1/L2 on p19 | — |
| K14 | Urban P_F 2.4 % (7.7 % at 15 km), target 0.1 % | L3-S22 pp. 8–9 | Verified | p8, Table I | — |
| K15 | ILS SR 100/99.1/68.2 vs 100/99.4/82.5 | L3-S47 pp. 15–16 | Verified | p15–16 | — |
| K16 | TTFF ~2 min (~5 min at Kp 7−) | L3-S47 p. 19 | Verified | p19 | — |
| K17 | Static PPP 35/50/60 min | L3-S49 p. 5 | Verified | Table 1 | — |
| K18 | 1 Hz obs / metadata "at least 60 seconds or higher" (EUREF) | L3-S42 p. 17 | Verified | p17 | — |
| K19 | Station-to-caster latency < 2 s (IGS, EUREF) | L3-S41 p. 10; L3-S42 p. 17 | Partly supported | S41 p10 "less than two seconds"; S42 p16 (not p17) "two seconds or less… is acceptable" | Fix page and wording |
| K20 | ~5 kbit/s RTK (2005) | L3-S38 p. 3 | Verified | p3 | — |
| K21 | Tallysman 100–120 mm | L3-S51 p. 2 | Verified | p2 | — |
| K22 | Lateral 0.57 m freeway, 0.29 m local | L3-S46 p. 1 | Verified | p1 | — |
| K23 | Wrong fixed > 13 m | L3-S48 p. 14 | Verified | p14 | — |
| T1 | C/N0 vs golden device, 38–40 dBHz | L3-S01 pp. 84–85 | Verified | as 11.1 | — |
| T2 | Open-sky ~50 good, < 40 degraded | L3-S02 p. 32 | Verified | as 4.2 | — |
| T3 | Kinematic vs truth; 30 cm | L3-S22 p. 8 | Verified | p8 | — |
| T4 | Long-distance drive metrics | L3-S21 pp. 14–16 | Verified | pp14–16 | — |
| T5 | RT1 PDOP ≤ 2.0, ≥ 7 SVs, RMS ≤ 0.01 m | L3-S11 p. 50 | Verified | p50 | — |
| T6 | TAR, 90 % success time | L3-S23 p. 9 | Verified | p9 | — |
| T7 | Obstruction series vs total station; alternating | L3-S48 pp. 5, 17 | Verified | p5, p17 | — |
| T8 | Iono comparison, ILS SR and TTFF per Kp | L3-S47 pp. 15–19 | Verified | pp15–19 | — |
| T9 | Stream latency logging; host clock | L3-S40 §2.12.2 | Verified | §2.12.2 | — |
| C1 | Fixed ratio threshold as correctness test | L3-S05 p. 3; L3-S06 p. 11 | Verified | as 9.1, 9.3 | — |
| C2 | Trusting RMS under multipath ("false precision") | L3-S11 p. 37 | Verified | p37 | — |
| C3 | Omitting 1230/1033; mismatched station IDs | L3-S01 p. 19 | Verified | p19 | — |
| C4 | Empty frame_id / missing TF → navsat assumes origin | L3-S29 lines 551–558, 609–615 | Partly supported | log strings at 555–556, 614–615 say so; per-fix path (591–602) instead logs "Will not remove offset" and outputs identity pose | As 10.8 |
| C5 | Lever-arm sign (right → −Y) | L3-S37 p. 25 | Verified | p25 | — |
| C6 | Antenna inside vehicle / no GP | L3-S01 p. 91; L3-S02 p. 6 | Verified | p91 note; p6 window/roof note | — |
| C7 | Wrong mountpoint → source-table | L3-S13 p. 10; L3-S18 lines 116–118 | Verified | p10; lines 116–117 | — |
| C8 | GBAS_FIX ≠ RTK fixed across drivers | L3-S43 92–99; L3-S44 1400–1428 | Verified | as 8.9, 8.10 | — |
| C9 | Obs messages "must" share rate; MSM4/7 mix bit | L3-S01 p. 22 | Verified | p22 exact quotes | — |
| C10 | Base-coordinate errors pass to rover | L3-S11 p. 40; L3-S01 p. 21 | Verified | S11 p40; S01 p21 "Any error in the base station position will directly translate into rover position errors" | — |
| C11 | Aviation integrity assumptions on the ground | L3-S45 pp. 2–3 | Verified | as 9.16 | — |
| D1 | Correction age: 60 s / 30 s / 10–15 s / ≤ 2 s | L3-S01 p. 19; S16 p. 41; S21 p. 15; S11 p. 44 | Verified | as 7.1–7.4 | — |
| D2 | Dead-stream detection BNC / RTKLIB / MicroStrain / Xsens | L3-S14 p. 11; S15 271–272; S18; S19 | Verified | as 7.6, 7.8, 7.9, 7.11 | — |
| D3 | GP size u-blox vs Xsens; u-blox specs on 150 mm plane | L3-S02 p. 17; S37 p. 25; S01 p. 91 | Verified | S01 p91 footnote 17 "Measured with a ground plane d=150 mm" | — |
| D4 | 99.9 % claims vs 2.4 % measured vs > 13 m wrong fix | L3-S11 p. 23; S22 p. 8; S48 p. 14 | Verified | as 9.8, 9.7, 9.15 | — |
| D5 | Tallysman 100–120 mm, larger may reduce gain, vs Xsens ≥ 150 mm | L3-S51 p. 2; S37 p. 25 | Verified | as 4.18, 4.8 | — |
| D6 | NavSatStatus RTK encodings differ; Xsens outputs GGA quality | L3-S43, S44, S20, S30 | Verified | as 8.8–8.10, 12.2 | — |
| D7 | Bandwidth 50–500 B/s vs ~5 kbit/s vs 1.1–1.4 kbps | L3-S13 p. 7; S38 p. 3; S17 p. 5 | Verified | as 6.5, 6.17, 6.9 | — |
| Q2 | NTRIP 2.0 known via S39/S40 changes and transport modes | L3-S39, L3-S40 | Verified | as 6.14, 6.15 | — |
| Q3 | EUREF/IGS 1 Hz obs; EUREF metadata ≥ 60 s | L3-S41, L3-S42 | Verified | as 6.12, 6.13 | — |
| Q5 | Lever-arm error degrades, needs rotation to be observable | L3-S50 | Verified | p2, p8 | — |
| Q6 | S48 used static tripod occupations near trees/buildings | L3-S48 | Verified | p4 "tripods were used together with Leica tribrachs"; p5 stations | — |

## Corrections applied (2026-09-28)

Applied by the topic editor from this review and `SOURCE_AUDIT.md`. Every Partly supported item was re-checked against its source before rewriting. No item was Not supported and no source failed, so nothing was removed. README Status set to **Verified**.

### Claims
| # | Change |
|---|---|
| S1 | Split into two bullets. Differencing / short-baseline dependence now cites L3-S11 pp. 22, 28, 32, and "multipath remains unmodelled after differencing and ambiguity resolution" cites L3-S11 p. 66 (general source first); L3-S37 p. 25 kept as the second citation. The 2.2 km link to the Fig. 2 scatter experiment is labelled an inference [L3-S03, p. 3]. |
| F14 | L3-S01 foundational row no longer says the F9P is "inside the MTi-680G"; it quotes L3-S32 pp. 11, 28 ("internal L1/L2 RTK enabled GNSS receiver", u-blox platform settings) and marks the F9P link as an inference. |
| 2.3 | Rewritten: 2.2 km / dual-frequency / 7 satellites attributed to Fig. 1; the 100-run, 5 s experiment to Fig. 2; the link labelled an inference. |
| 2.8 | Citation changed to L3-S07 pp. 1–2. |
| 3.9 | "Vehicle motion" replaced by "gentle wavelength-scale random antenna motion (2–5 cm/s, a hand-moved antenna rather than vehicle driving)". |
| 3.14 | Now states the < 10 s value with only the "< 30 s for GPS only" footnote; notes that the 4-constellation footnote belongs to cold-start acquisition. |
| 4.12 | Added "in open-sky conditions"; wording says the calibration was of the rover antenna's phase-centre variation. |
| 4.13 | Causal clause dropped; now quotes "more or less ignored up to November 2006". |
| 9.7 | Numbers cited to p. 8; the explanation (multipath, foliage, Gaussian assumptions, thick-tailed errors) cited to pp. 4, 6. |
| 9.20 | Cites Fig. 5 on p. 4 and records the contradictory p. 3 wording as an apparent typo. |
| 10.8 | Split into two bullets: offset removal (lines 517–550, 564–589) and missing-transform behaviour per path (initial pose assumes origin, lines 551–560; per-fix path leaves identity pose and does not remove the offset, lines 568, 590–603; empty `frame_id`, lines 609–615). |
| 10.9 | Line citation extended to 102, 337, 438–439, 848–851 for the local-ENU option. |
| 11.5 | Added p. 7 (L3-S23 pp. 5–7, 9). |
| 11.6 | Added p. 6 (L3-S24 pp. 1, 4, 6). |
| K3 | Condition cites L3-S32 pp. 13, 15 and L3-S33 p. 14; condition text states the 1 km baseline / patch-antenna measurement set-up. |
| K4 | Condition changed from "multi-constellation" to "constellations for the 10 s value not stated". |
| K19 | EUREF cited at p. 16 with its wording ("two seconds or less … is acceptable"); row renamed "station-to-data-centre latency". |
| C4 | Reworded to match 10.8 (lines 551–560, 568, 590–603, 609–615). |
| Open question 7 | Softened to "no open standard or REP was found … the drivers read map it differently (see Disagreements)". |
| BNC currency (audit S14) | Findings §7, Recommended practice 7, Key numbers and Disagreements now cite BNC 2.13.7 (L3-S40, section 2.11) for the 20 s / 256 s outage rule and decoder buffering; L3-S14 kept only as a historical second citation. The former separate "BNC 2.13.7 keeps the same rule" bullet was reduced to the Failure-threshold fact (section 2.11.2). |
| General first (audit coverage) | Reordered so general sources open §3 (L3-S11), §4 (L3-S23, S24, S22, S26), §6 (L3-S13, S38), §7 (L3-S21, S11) and §11 (L3-S22, S21 …); product bullets follow. §1 now gives the general multipath statement (L3-S11 p. 66) before the Xsens app-note one. |
| L3-S28 label | "(preprint, level C)" removed from the §10 bullet after the source was raised to level A. |

### Sources
| ID | Change |
|---|---|
| L3-S02 | File renamed `ublox_2015_…` → `ublox_2019_gnss_antennas_rf_design.pdf`; date given as 16 October 2019 (R03). |
| L3-S08 | Link changed to the UNB-hosted PDF (byte-identical); UNB affiliation and the reason for level C added. |
| L3-S09 | Link changed to the open 1st-edition PDF actually used (byte-identical); 3rd edition noted with its URL. |
| L3-S13 | File renamed to `bkg_2004_ntrip_v1_documentation.pdf`; citation states the document is undated, c. 2004 (evidence given). |
| L3-S14 | Citation date aligned with the file name (c. 2007, evidence given); labelled historical, superseded by L3-S40. |
| L3-S17 | Direct PDF link (author's site, byte-identical), venue dates; proceedings page range not found (noted). |
| L3-S21 | Added ION GNSS+ 2019 pp. 2135–2158, doi:10.33012/2019.16914. |
| L3-S23 | Added exact PDF URL and ION GNSS+ 2014 pp. 1568–1577. |
| L3-S25 | Added direct University of Calgary PDF URL (byte-identical) and full KIS 2001 name; page range not found (noted). |
| L3-S28 | Cited as IEEE CDC 2024, Milan, pp. 2005–2011, doi:10.1109/CDC56724.2024.10886559 (Crossref and proceedings TOC); arXiv PDF kept as open copy; level C → A. |
| L3-S31 | File renamed to `ros_2010_rep105_coordinate_frames.rst`; noted as a living document. |
| L3-S32 | Link changed to the manual's official page; noted that the saved PDF is byte-identical to the RS Components copy. |
| L3-S33, S34, S35 | Direct official Xsens URLs (byte-identical); currency note: 2020 revisions are still the ones served on 2026-09-28, with the 2023 user manual cited first for specs. S35 file renamed to `xsens_2020_mti600_hardware_integration_manual.pdf`. |
| L3-S36 | File renamed to `xsens_2020_mti600_dk_user_manual.pdf`; direct official URL (byte-identical). |
| L3-S37 | Direct PDF URL (byte-identical); date 28 June 2022. |
| L3-S41 | File renamed to `igs_2021_real_time_broadcaster_station_guidelines.pdf`; level B → A for consistency with L3-S11. |
| L3-S42 | File renamed to `euref_2025_epn_station_guidelines.pdf`; both the table link and the URL printed in the PDF confirmed to serve the same byte-identical file; level B → A for consistency with L3-S11. |
| Foundational references | Added "not downloaded" rows for Teunissen 1998 (*J. Geodesy* 72), the NGS *Guidelines for Real Time GNSS Networks* (open draft v2.2, not needed for any finding) and REP-103 (open, not needed); the Open questions list of undownloaded works was updated. |

Format: no text-to-PDF replacements were needed (the audit found none). Final mechanical check: `file` on all 64 files in `sources/` matches their extensions; every file maps to a Sources-table row; every cited ID is in the table; IDs L3-S01 to L3-S52 have no gaps or duplicates.
