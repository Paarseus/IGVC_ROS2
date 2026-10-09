# L5 — Time synchronization: claim verification

| | |
|---|---|
| **Topic** | L5 — Time synchronization |
| **Date** | 2026-09-28 |
| **Reviewer** | independent — claims |
| **Scope** | Every cited item in README.md (Summary, Foundational references, Findings 1–12, Recommended practice, Key numbers, How it is tested, Common mistakes, Disagreements, cited Open questions). Sources were not graded (see SOURCE_AUDIT.md). |
| **Method** | PDFs read with `pdftotext -layout` per page (page numbers below are PDF pages unless marked printed); scanned Lamport PDF read from a page image; text/code files read with grep and line numbers. (Lnnn) = README line number. |

## Counts

| Section | Checked | Verified | Partly supported | Not supported |
|---|---|---|---|---|
| Summary | 7 | 6 | 1 | 0 |
| Foundational | 17 | 15 | 2 | 0 |
| F1 Clock models | 24 | 21 | 3 | 0 |
| F2 Effects | 12 | 10 | 2 | 0 |
| F3 Delayed/OOSM | 14 | 13 | 1 | 0 |
| F4 NTP/chrony | 19 | 19 | 0 | 0 |
| F5 PTP | 22 | 20 | 2 | 0 |
| F6 GNSS/PPS | 13 | 13 | 0 | 0 |
| F7 Stamping | 29 | 29 | 0 | 0 |
| F8 Temporal calib | 15 | 14 | 1 | 0 |
| F9 ROS 2 | 34 | 33 | 1 | 0 |
| F10 Verification | 16 | 14 | 2 | 0 |
| F11 Domains | 11 | 11 | 0 | 0 |
| F12 Products | 36 | 35 | 1 | 0 |
| Rec practice | 14 | 14 | 0 | 0 |
| Key numbers | 34 | 33 | 1 | 0 |
| How tested | 15 | 13 | 2 | 0 |
| Common mistakes | 17 | 17 | 0 | 0 |
| Disagreements | 19 | 13 | 6 | 0 |
| Open questions | 7 | 7 | 0 | 0 |
| **Total** | **375** | **350** | **25** | **0** |

## Items needing correction (summary)

- Partly supported — Device clock mapped to host time "from arrival times alone" (Olson; TICSync; VersaVIS); sub-ms or ±0.2 ms over USB (L22): Say "from arrival or round-trip timing"; TICSync and VersaVIS are not arrival-only.
- Partly supported — S49 Cristian: round-trip remote clock reading behind NTP/PTP (L35): Mark as described via S11 p.2; "behind … PTP" is not stated in any source.
- Partly supported — S05/S50 PTP standard; "Eidson chaired the 1588 committee" (L36): Remove the "chaired" claim or cite a source for it.
- Partly supported — ΔT = To + (Δf/f)t + ½Dt² + σx(t) (L52): Change to PDF p.29.
- Partly supported — Without re-syntonization, aging can be the largest contributor for quartz (L53): Change to PDF p.29; source says "many frequency sources (e.g., quartz …)".
- Partly supported — Allan variance formula eq.6, standard time-domain measure (L55): Change to PDF p.24.
- Partly supported — Unmodelled GNSS–IMU timing error → increased covariance and forward-acceleration bias (L88): Change to PDF p.15.
- Partly supported — Adding timing error to estimator → "almost perfect time synchronization"; observable with turns or accelerations (L89): Change to PDF p.15.
- Partly supported — history_length "must be at least as large as the lag" (L114): Use "should".
- Partly supported — ~100 ns achievable; <20 ns "requires better oscillators, faster sampling, boundary clocks and temperature control" (L151): Say "some combination of"; add statistics/servo algorithms.
- Partly supported — ts2phc.nmea_delay "must be set" so NMEA stamps match the right pulse (L159): Add the condition (only needed when NMEA delay can exceed 1 s / pulse width).
- Partly supported — Excite axes, "low jitter timestamps in same clock"; IMU DT bursts 1 ms/6 ms (L236): Cite "4) The output" without "WARNING".
- Partly supported — rmw source/received timestamps; publication_sequence_number "shows how many messages were lost" (L286): Say "how many were sent in between (lost or taken elsewhere), where the rmw supports it".
- Partly supported — T265 bimodal offset "about ±15 ms apart (half a frame period)" (L301): Write "modes about 15 ms apart".
- Partly supported — Kalibr: inspect IMU DTs (L309): Drop "WARNING".
- Partly supported — Missing requested field → host now() (L348): Describe the two-step fallback.
- Partly supported — "Passive clock mapping (TICSync)" ~35 μs, 162 ms RTT (L413): Relabel "Two-way clock mapping (TICSync)".
- Partly supported — IMU timestamp interval plot; 1 ms bursts / 6 ms gaps (L445): Cite "4) The output".
- Partly supported — Source vs received stamps; "sequence numbers expose lost messages" (L450): Say "sequence gaps count messages not received by this take (lost or taken elsewhere)".
- Partly supported — "Sources differ mainly in which NTP implementation and configuration they assume" (L476): Label as an inference.
- Partly supported — Skog "almost perfect time synchronization" (L478): Change to PDF p.15.
- Partly supported — Qin & Shen summarize Li & Mourikis as "successful" (L479): Reword to "describe it as computationally efficient but less accurate than their own".
- Partly supported — "Optimization and batch methods are not criticized on these grounds" (L481): Label as inference and cite S19 p.8.
- Partly supported — Mauthner "find" buffering can compete (L483): Use "suggest" / "hint".
- Partly supported — Mauthner "find" buffering competitive (L491): Use "suggest".

Uncited factual statements found: "Eidson chaired the 1588 committee" (L36) and the inference at L476; both are listed in the table. No other uncited factual statements were found outside the Sources table and the uncited Open questions (L510, L511, L514), which only state that something was not found.

## Claim-by-claim results

| # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|
| 1 | [Summary] Timing error → geometric error: v·μ + d·ω·μ; 15.7 cm example (L14) | S14 p.3 eq.10; S11 p.1 | Verified | S14 eq.(10) "δsync ≈ ∥v∥μ + d·∥ω∥μ"; S11 p.1 "90 deg/s … 10 m … 10 ms … 15.7 cm" | — |
| 2 | [Summary] Arrival stamping weakest; non-RT OS jitter hundreds of ms; FTDI 16 ms; HW timestamping only deterministic way (L15) | S11 p.1; S14 pp.1–2; S39 p.7 | Verified | S11 p.1 "hundreds of milliseconds on a loaded system"; S14 p.2 "deterministic … only solution is to perform it in hardware. This applies to 1PPS, TOV and Triggered"; S39 p.7 16 ms | Optional: add "on a loaded system". |
| 3 | [Summary] Per-method accuracy list: NTP few ms / few hundred μs; chrony sub-μs "might be possible"; PTP ~100 ns; F9T 5 ns 1σ (L16–21) | S01 p.1; S02 §1; S24 FAQ 2.7; S05 p.73; S31 §1.2 | Verified | All five quotes found at cited places (S02 §1 says "precise within a few hundred microseconds") | — |
| 4 | [Summary] Device clock mapped to host time "from arrival times alone" (Olson; TICSync; VersaVIS); sub-ms or ±0.2 ms over USB (L22) | S11 pp.3–5; S17 pp.1,7; S20 p.5 | Partly supported | Olson is arrival-only; TICSync learns the mapping from two-way (round-trip) exchanges (S17 p.7 "mean round trip time of 162ms"); VersaVIS uses request/response exchanges with the MCU (S20 p.3 eq.2) | Say "from arrival or round-trip timing"; TICSync and VersaVIS are not arrival-only. |
| 5 | [Summary] Constant offset estimable: ±0.2 ms Kalibr batch; online in VIO (L24) | S12 p.6; S18 p.7 | Verified | S12 p.6 "within a domain of ±0.2 ms"; S18 p.7 "converges … within a few seconds" | — |
| 6 | [Summary] Late measurement fused at true time by reprocessing, extrapolation or exact OOSM update (L25) | S09 p.2; S15 p.2; S36 smooth_lagged_data | Verified | S09 p.2 extrapolation/recalculation; S15 p.2 in-sequence processing, Bar-Shalom exact update; S36 revert-and-reprocess | — |
| 7 | [Summary] header.stamp = acquisition; tf2 10 s, interpolates, no extrapolation; ROS time = system unless use_sim_time (L26) | S35; S32; S13 | Verified | Image.msg "should be acquisition time"; buffer_core.hpp l.73 10 s; cache.cpp extrapolation errors; S13 "ROS Time" | — |
| 8 | [Foundational] S01 Mills 1991: four-timestamp offset/delay, minimum-delay filter, discipline loop (L31) | S01 | Verified | S01 pp.5, 9–10 (formulas, lowest-delay sample, PLL) | — |
| 9 | [Foundational] S02 RFC 5905 current NTP standard (L32) | S02 | Verified | RFC 5905 "Standards Track", NTPv4 | — |
| 10 | [Foundational] S03 introduced hybrid PLL/FLL and Allan-intercept argument (L33) | S03 | Verified | S03 p.3 "true hybrid PLL/FLL design"; p.8 "Allan intercept is the primary statistic" | — |
| 11 | [Foundational] S04 Lamport: event ordering and physical clock bounds (L34) | S04 | Verified | Abstract (p.558, read from page image): partial ordering, logical clocks, "bound … how far out of synchrony" | — |
| 12 | [Foundational] S49 Cristian: round-trip remote clock reading behind NTP/PTP (L35) | S49 (not downloaded) | Partly supported | Not downloaded; only secondary support: S11 p.2 "Most clock synchronization schemes are variants on Cristian's Algorithm, which relies on round-trip measurements" | Mark as described via S11 p.2; "behind … PTP" is not stated in any source. |
| 13 | [Foundational] S05/S50 PTP standard; "Eidson chaired the 1588 committee" (L36) | S05 | Partly supported | S05 is Eidson's tutorial (p.1); no text says he chaired the committee (p.93 only "appoints chair of working group", generic) | Remove the "chaired" claim or cite a source for it. |
| 14 | [Foundational] S06 RFC 2783 kernel PPS interface (L37) | S06 | Verified | RFC 2783 §1, §3 | — |
| 15 | [Foundational] S07 NIST TN 1337 clock/oscillator characterization (L38) | S07 | Verified | PDF p.15 (TN-4) Allan variance, drift papers | — |
| 16 | [Foundational] S51 Bar-Shalom exact and multistep OOSM updates (L39) | S51 (not downloaded) | Verified | Titles in S51 citation; S15 p.2 describes Bar-Shalom (2002) exact update | — |
| 17 | [Foundational] S09 Larsen: standard comparison + extrapolation method (L40) | S09 | Verified | S09 p.2 abstract "various methods … compared", new extrapolation method | — |
| 18 | [Foundational] S52/S10 Skog & Händel GNSS–IMU time offset (L41) | S10 | Verified | S10 PDF p.15 Paper E summary | — |
| 19 | [Foundational] S11 Olson passive mapping from arrival times (L42) | S11 | Verified | S11 p.1 abstract | — |
| 20 | [Foundational] S12 Furgale continuous-time batch, Kalibr (L43) | S12; S38 | Verified | S12 p.3 B-splines; S38 cites Furgale 2013 as the Kalibr method | — |
| 21 | [Foundational] S53/S54 Li & Mourikis online filter + identifiability; Kelly & Sukhatme curve registration (L44) | S53; S18 | Verified | S53 abstract; S18 p.2 "aligned rotation curves … ICP" | — |
| 22 | [Foundational] S13 ROS 2 time sources and use_sim_time (L45) | S13 | Verified | S13 "Time Abstractions", "ROS Time" | — |
| 23 | [Foundational] S55 Vig tutorial XO/TCXO/OCXO (L46) | S55 | Verified | S55 slides 2-6, 2-8, 7-1 | — |
| 24 | [Foundational] S56 IS-GPS-200N GPS time, leap seconds, week number (L47) | S56 | Verified | §3.3.4; §20.3.3.3.1.1; §20.3.3.5.2.4 | — |
| 25 | [F1 Clock models] ΔT = To + (Δf/f)t + ½Dt² + σx(t) (L52) | S08 PDF p.28 (printed p.19) eq.16 | Partly supported | Eq. (16) text found, but on PDF p.29 (printed p.19); PDF p.28 is printed p.18 (TVAR) | Change to PDF p.29. |
| 26 | [F1 Clock models] Without re-syntonization, aging can be the largest contributor for quartz (L53) | S08 PDF p.28 (printed p.19) | Partly supported | Text "frequency aging can cause this to be the biggest contributor … (e.g., quartz crystal oscillators and rubidium …)" is on PDF p.29 | Change to PDF p.29; source says "many frequency sources (e.g., quartz …)". |
| 27 | [F1 Clock models] Linear drift eventually dominates, even after correction; important for quartz (L54) | S07 PDF p.15 (TN-4) | Verified | "even with correction for drift, the magnitude of drift error eventually dominates … particularly important in … quartz oscillators" | — |
| 28 | [F1 Clock models] Allan variance formula eq.6, standard time-domain measure (L55) | S08 PDF p.23 (printed p.14) eq.6 | Partly supported | Eq. (6) and "most common time domain measure" are on PDF p.24 (printed p.14) | Change to PDF p.24. |
| 29 | [F1 Clock models] Allan intercept: below, averaging helps; above, hurts; NTP constants built around it (L56) | S03 p.8 | Verified | p.8 "For τ below this point … increased averaging can improve the accuracy; however, above this point … degrades" | — |
| 30 | [F1 Clock models] Computer oscillators 0.01 % accurate, several ppm with room temperature (L57) | S01 p.11 | Verified | p.11 "accurate to only .01 percent and may vary several parts-per-million (ppm) … normal room-temperature" | — |
| 31 | [F1 Clock models] Battery-backed quartz "may drift as much as a second per day" (L58) | S01 p.1 | Verified | p.1 exact phrase | — |
| 32 | [F1 Clock models] SICK LMS151 lost 92 s in 6 days; >30 ms temperature swings after skew removal (L59) | S17 p.1 Fig.1 | Verified | Fig.1 caption "lost 92 seconds compared to GMT … swings of over 30ms" | — |
| 33 | [F1 Clock models] Olson rate errors α1, α2 "a few percent"; RC oscillators larger (L60) | S11 pp.3–4 | Verified | p.3 "quite small: a few percent"; p.4 "inexpensive microcontrollers with RC oscillators" | — |
| 34 | [F1 Clock models] MTi-600 internal clock "about 10 ppm" (L61) | S40 p.37 | Verified | PDF p.37 "accuracy of about 10 ppm" | — |
| 35 | [F1 Clock models] XO / TCXO ("about a 20X") / OCXO (">1000X") categories (L62–66) | S55 PDF p.31 (slide 2-6) | Verified | Slide 2-6 exact wording | — |
| 36 | [F1 Clock models] Hierarchy XO 1e-5–1e-4 (computer timing), TCXO 1e-6, OCXO 1e-8 incl. −40…+75 °C and 1 yr aging (L67) | S55 PDF p.33 (slide 2-8) | Verified | Slide 2-8 table and footnote | — |
| 37 | [F1 Clock models] Temp. stability −55…+85 °C: TCXO 5e-7, OCXO 1e-9; aging 5e-7, 5e-9 /yr (L68) | S55 PDF p.237 (slide 7-1) | Verified | Slide 7-1 table | — |
| 38 | [F1 Clock models] GPS time continuous, zero at midnight 5/6 Jan 1980 UTC(USNO); leap seconds differ (L69) | S56 §3.3.4 PDF p.56 | Verified | §3.3.4 exact | — |
| 39 | [F1 Clock models] GPS within 1 μs of UTC mod 1 s; GPS–UTC data within 20 ns 1σ (L70) | S56 §3.3.4 PDF p.56 | Verified | "within one microsecond of UTC (modulo one second)"; "within 20 nanoseconds (one sigma)" | — |
| 40 | [F1 Clock models] LNAV sends 10 LSBs of week, "a modulo 1024 binary representation" (L71) | S56 §20.3.3.3.1.1 PDF p.112 | Verified | p.112 exact phrase | — |
| 41 | [F1 Clock models] 23:59:60.xxx; decrementing; "must consistently implement carries or borrows" (L72) | S56 §20.3.3.5.2.4 b PDF p.147 | Verified | p.147 exact phrases | — |
| 42 | [F1 Clock models] chrony leap smear 62500 s (~17.36 h), max 32 ppm; identical smearing servers (L73) | S26 leapsecmode | Verified | "take 62500 seconds (about 17.36 hours) … maximum of 32 ppm"; "smear the leap second in exactly the same way" | — |
| 43 | [F1 Clock models] CLOCK_REALTIME settable, jumps, slewed by NTP, ignores leap seconds (L75) | S29 DESCRIPTION | Verified | clock_getres.2 CLOCK_REALTIME paragraph | — |
| 44 | [F1 Clock models] CLOCK_MONOTONIC not settable, never backwards, affected by frequency adjustments (L76) | S29 DESCRIPTION | Verified | "is affected by frequency adjustments"; "will not go backwards" | — |
| 45 | [F1 Clock models] MONOTONIC_RAW no freq adj; BOOTTIME counts suspend; TAI counts leap seconds (L77) | S29 DESCRIPTION | Verified | Each clock's paragraph | — |
| 46 | [F1 Clock models] Unix time has no 23:59:60; chrony default kernel steps back at 00:00:00 (L78) | S26 leapsecmode | Verified | "The system clock cannot have time 23:59:60 …"; system mode default | — |
| 47 | [F1 Clock models] Timing receiver pulse aligned to GPS or UTC (L79) | S30 §1 | Verified | §1 "synchronized to either GPS or UTC" | — |
| 48 | [F1 Clock models] Lamport: partial ordering, logical clocks, drift bound (L80) | S04 p.558 | Verified | Abstract and intro, p.558 (page image) | — |
| 49 | [F2 Effects] Syncline eq.10 definition of terms (L83) | S14 p.3 eq.10 | Verified | eq.(10) and variable definitions | — |
| 50 | [F2 Effects] No relative motion → no sync-induced error (L84) | S14 p.3 | Verified | p.3–4 "if there is no relative movement … there is also no sync-induced error" | — |
| 51 | [F2 Effects] First term = distance travelled; second = orientation × distance, small angle (L85) | S14 p.3 | Verified | p.3 "For small angles this is an approximation" | — |
| 52 | [F2 Effects] Accuracy limited by sensor noise or sync; slow platforms noise-bound, fast (UAV) sync-bound at hundreds of ms (L86) | S14 p.1 | Verified | p.1 abstract and intro (USV vs UAV example) | — |
| 53 | [F2 Effects] Olson 90 deg/s, 10 m, 10 ms → 15.7 cm (L87) | S11 p.1 | Verified | p.1 exact | — |
| 54 | [F2 Effects] Unmodelled GNSS–IMU timing error → increased covariance and forward-acceleration bias (L88) | S10 PDF p.11 | Partly supported | Text is in the Paper E summary on PDF p.15 (printed p.3); PDF p.11 is not it | Change to PDF p.15. |
| 55 | [F2 Effects] Adding timing error to estimator → "almost perfect time synchronization"; observable with turns or accelerations (L89) | S10 PDF p.11 | Partly supported | Same text, PDF p.15 (printed p.3) | Change to PDF p.15. |
| 56 | [F2 Effects] Three latency failures: residual editing, repeated processing → divergence, OOSMs (L90–94) | S15 p.2 | Verified | p.2 "rejected by residual editing … filter divergence … out-of-sequence measurements" | — |
| 57 | [F2 Effects] Lunar sim: uncertainty most affected by camera jitter in low-dynamics periods; timing hardware > processing (L95) | S15 p.27 | Verified | p.27 "most strongly impacted by terrain camera jitter during periods of little dynamic activity … much more important than … improved processing strategies" | — |
| 58 | [F2 Effects] Pure delay: unit gain, phase lag ωτ linear in frequency (L96) | S23 PDF p.21 (p.10-20) | Verified | "additional phase lag of ωτ … increases linearly with frequency" | — |
| 59 | [F2 Effects] τc = (π + θ0)/ω0 makes L(iω0) = −1; oscillation may result (L97) | S23 PDF p.5 (p.10-4) | Verified | p.10-4 exact | — |
| 60 | [F2 Effects] Delay margin definition; more relevant with high-frequency peaks (L98) | S23 PDF p.18 (p.10-17) | Verified | p.10-17 exact | — |
| 61 | [F3 Delayed/OOSM] Larsen's four methods (augment, recalculate, Alexander correction, extrapolation) (L101–106) | S09 PDF p.2 (p.3972) | Verified | p.3972 intro lists all four | — |
| 62 | [F3 Delayed/OOSM] No method fully compensates (all >1); recalculation = modified Alexander (L107) | S09 PDF p.6 (p.3976) | Verified | "normalized variances … all higher than one"; "modified Alexander … yields the same results as recalculation" | — |
| 63 | [F3 Delayed/OOSM] In-sequence processing keeps history and re-processes (L108) | S15 p.2 | Verified | p.2 exact | — |
| 64 | [F3 Delayed/OOSM] Bar-Shalom exact update within current interval; older needs smoothing-like reprocessing (L109) | S15 p.2 | Verified | p.2 exact | — |
| 65 | [F3 Delayed/OOSM] Fixed-lag/reprocessing: lagging optimal estimate, heavy overhead (L110) | S15 pp.2–3 | Verified | p.2–3 "optimal estimate that lags behind … significant computational overhead" | — |
| 66 | [F3 Delayed/OOSM] Brouk & DeMars latency as filter state; temporal measurement update (L111) | S15 pp.1,3 | Verified | p.1 abstract; p.3 "treats the latency as a filter state" | — |
| 67 | [F3 Delayed/OOSM] OOSMs on time-triggered nets; buffering can compete when a = n·b and synchronizable (L112) | S16 p.10 | Verified | p.10 conclusion | — |
| 68 | [F3 Delayed/OOSM] smooth_lagged_data reverts and reprocesses (L113) | S36 state_estimation_nodes.rst | Verified | "revert to the last state prior to the lagged measurement, then process all measurements" | — |
| 69 | [F3 Delayed/OOSM] history_length "must be at least as large as the lag" (L114) | S36 ~history_length | Partly supported | Source: "This value should be at least as large as …" | Use "should". |
| 70 | [F3 Delayed/OOSM] smooth off: Δt ≤ 0 → no predict, correct current state (L115) | S36 filter_base.cpp processMeasurement | Verified | "Only want to carry out a prediction if it's forward in time. Otherwise, just correct." | — |
| 71 | [F3 Delayed/OOSM] History too short → debug error, continues (L116) | S36 ros_filter.cpp integrateMeasurements | Verified | RF_DEBUG "ERROR: history interval is too small to revert"; warning commented out | — |
| 72 | [F3 Delayed/OOSM] Indelman: factor graph, delayed measurements "in a natural way", adaptive fixed-lag (L117) | S57 p.1 | Verified | p.1 abstract | — |
| 73 | [F3 Delayed/OOSM] Delayed GPS at tl as factor on node tl; "as easily as any other measurement" (L118) | S57 pp.2,4 | Verified | p.4 "time-delayed since usually tk > tl"; p.2 "as easily as any other measurement" | — |
| 74 | [F3 Delayed/OOSM] Buffer of past solutions "only an approximated solution"; simulation only (L119) | S57 p.1 | Verified | p.1 exact; abstract "simulated environment using IMU, GPS and stereo" | — |
| 75 | [F4 NTP/chrony] NTP offset/delay formulas (L122–125) | S02 §8; S01 p.5 | Verified | RFC l.1600–1604 exact; S01 p.5 equivalent a,b form | — |
| 76 | [F4 NTP/chrony] True offset within θ ± δ/2 (L126) | S01 p.5 | Verified | "θi − δi/2 ≤ θ ≤ θi + δi/2" | — |
| 77 | [F4 NTP/chrony] Asymmetry: stable to ns but off by ms; half root delay max asymmetry error (L127) | S24 FAQ 7.2 | Verified | FAQ 7.2 exact | — |
| 78 | [F4 NTP/chrony] Last eight samples, lowest delay chosen (L128) | S01 pp.9–10 | Verified | "eight-stage shift register … sorted in order of increasing δ" | — |
| 79 | [F4 NTP/chrony] PLL better with jitter, FLL with wander; NTPv4 combines (L129) | S03 p.2 | Verified | p.2 exact; p.3 "new Version 4 discipline … true hybrid" | — |
| 80 | [F4 NTP/chrony] Tc = 1024 s: 53-min rise, 5 % overshoot (L130) | S03 p.3 | Verified | p.3 exact | — |
| 81 | [F4 NTP/chrony] 1991 Internet "a few milliseconds" (L132) | S01 p.1 | Verified | Abstract | — |
| 82 | [F4 NTP/chrony] 1998: < 1 ms LAN, < few tens of ms Internet (L133) | S03 p.1 | Verified | p.1 "better than a millisecond in LANs and better than a few tens of milliseconds in most places" | — |
| 83 | [F4 NTP/chrony] NTPv4: primary tens of μs; LAN clients few hundred μs (L134) | S02 §1 | Verified | §1 "precise within …" (poll intervals up to 1024 s) | — |
| 84 | [F4 NTP/chrony] STEPT 125 ms steps and resets associations; PANICT 1000 s exit (L135) | S02 §§11.2.3, 11.3 | Verified | l.2517–2531 "SHOULD cause the program to exit"; "all associations MUST be reset" | — |
| 85 | [F4 NTP/chrony] Step only after WATCH 900 s; resists congestion steps (L136) | S02 §11.3 | Verified | l.2782–2785 exact | — |
| 86 | [F4 NTP/chrony] chrony slews by default; makestep 0.1 3 (L137) | S26 makestep; S24 FAQ 3.4 | Verified | makestep text and example | — |
| 87 | [F4 NTP/chrony] Step only at boot "before starting programs that rely on time advancing monotonically forwards" (L138) | S26 makestep | Verified | exact | — |
| 88 | [F4 NTP/chrony] Test 1: 35±8 / 234±46 / 857±226 μs; 1 ms jitter 475±93 vs 454±94 (L139) | S25 Test 1 | Verified | Table exact | — |
| 89 | [F4 NTP/chrony] 30 min/day: chrony ~7–26 ms, ntp ~0.6–1.1 s (L140) | S25 Test 3 | Verified | 7273–26105 μs vs 580679–1115961 μs | — |
| 90 | [F4 NTP/chrony] hwtimestamp: NIC clock, Linux ≥ 3.19, ethtool -T (L141) | S26 hwtimestamp | Verified | exact | — |
| 91 | [F4 NTP/chrony] "a sub-microsecond accuracy and stability of a few tens of nanoseconds might be possible" (L142) | S24 FAQ 2.7 | Verified | exact | — |
| 92 | [F4 NTP/chrony] Constant CPU frequency, disable EEE, prioritize NTP in switches (L143) | S24 FAQ 2.7 | Verified | FAQ 2.7 last paragraph of that example | — |
| 93 | [F4 NTP/chrony] Harrison & Newman: NTP undesirable (hours; frequency adjustments) (L144) | S17 p.1 | Verified | p.1 exact | — |
| 94 | [F5 PTP] 1588 aims sub-μs in localized systems (L147) | S05 p.10 | Verified | slide 10 | — |
| 95 | [F5 PTP] Offset and one-way delay formulas (L148) | S05 p.23 | Verified | slide 23 | — |
| 96 | [F5 PTP] Symmetric assumption; example 55 vs 60 min (L149) | S05 pp.23–24 | Verified | slide 24 "55 minutes (not actual 60)" | — |
| 97 | [F5 PTP] HW timestamping at MAC/PHY "potentially the most accurate"; boundary clocks reduce fluctuations (L150) | S05 pp.70–72 | Verified | slides 70–72 | — |
| 98 | [F5 PTP] ~100 ns achievable; <20 ns "requires better oscillators, faster sampling, boundary clocks and temperature control" (L151) | S05 p.73 | Partly supported | Slide 73: "<20 ns will require some combination of faster sampling, better oscillators, boundary clocks, sophisticated statistics and servo algorithms and careful control of environment" | Say "some combination of"; add statistics/servo algorithms. |
| 99 | [F5 PTP] Slave servo typically PI (L152) | S05 p.79 | Verified | slide 79 | — |
| 100 | [F5 PTP] ptp4l implements BC, OC, TC (L154) | S27 ptp4l.8 | Verified | DESCRIPTION | — |
| 101 | [F5 PTP] ptp4l defaults E2E and hardware timestamping (L155) | S27 ptp4l.8 | Verified | -E "This is the default mechanism"; -H / time_stamping default hardware | — |
| 102 | [F5 PTP] delayAsymmetry option (L156) | S27 ptp4l.8 | Verified | delayAsymmetry entry | — |
| 103 | [F5 PTP] phc2sys syncs system clock to PHC kept by ptp4l (L157) | S27 phc2sys.8 | Verified | DESCRIPTION | — |
| 104 | [F5 PTP] ts2phc syncs PHC to external timestamps (1-PPS); ToD from NMEA RMC (L158) | S27 ts2phc.8 | Verified | DESCRIPTION; -s "nmea … RMC" | — |
| 105 | [F5 PTP] ts2phc.nmea_delay "must be set" so NMEA stamps match the right pulse (L159) | S27 ts2phc.8 | Partly supported | "If the maximum delay is longer than 1 second … this option needs to be set accordingly"; default 0 | Add the condition (only needed when NMEA delay can exceed 1 s / pulse width). |
| 106 | [F5 PTP] SOF_TIMESTAMPING_RX_HARDWARE from adapter (L161) | S28 timestamping.rst 1.3.1 | Verified | exact | — |
| 107 | [F5 PTP] RX_SOFTWARE "just after a device driver hands a packet to the kernel receive stack" (L162) | S28 1.3.1 | Verified | exact | — |
| 108 | [F5 PTP] SO_TIMESTAMP "not necessarily monotonic" (L163) | S28 §1 | Verified | exact | — |
| 109 | [F5 PTP] chrony no PTP; PHC as refclock / HW timestamping; NTP over PTP (L164) | S24 FAQ 2.14 | Verified | FAQ 2.14 | — |
| 110 | [F5 PTP] "if it had the same support as PTP, it could perform equally well" (L165) | S24 FAQ 2.14 | Verified | exact | — |
| 111 | [F5 PTP] gPTP options neighborPropDelayThresh, 802.1AS-capable checks (L166) | S27 ptp4l.8 | Verified | neighborPropDelayThresh; asCapable | — |
| 112 | [F5 PTP] PI scale defaults 0.7/0.3 HW, 0.1/0.001 SW (L167) | S27 ptp4l.8 | Verified | pi_proportional_scale / pi_integral_scale | — |
| 113 | [F5 PTP] gPTP profile of 1588; P2P only; no TCs; OC/BC; each node runs servo (L168) | S58 PDF p.5 §1; S59 pp.1–2 | Verified | S58 "profile (or subset)"; S59 p.1–2 P2P only, OC/BC, "each node runs the servo" | — |
| 114 | [F5 PTP] Each hop another PI loop; accuracy falls "significantly"; tuning trade-off; embedded measurements (L169) | S59 pp.1,3 | Verified | p.1 abstract; p.3–4 overshoot/settling trade | — |
| 115 | [F5 PTP] AUTOSAR: 802.1AS with restrictions; no BMCA/Announce/Signaling; Annex B.1.2 (L170) | S58 PDF pp.5–6 | Verified | §1, §1.2.2, §1.2.3 | — |
| 116 | [F6 GNSS/PPS] PPS API captures timestamp ASAP after edge; compare with time code (L173) | S06 §1 | Verified | §1 "record ('capture') a high-resolution timestamp as soon as possible" | — |
| 117 | [F6 GNSS/PPS] Fixed offsets can be applied (L174) | S06 §3.2 | Verified | §3.2 "adding offsets to the timestamps that are captured" (also §3.1) | — |
| 118 | [F6 GNSS/PPS] PPS on DCD or GPIO; with NTPD "sub-millisecond synchronisation to UTC" (L175) | S28 pps.rst | Verified | Overview exact | — |
| 119 | [F6 GNSS/PPS] PPS lacks full time; pairing refclock offset < 0.4 s (L176) | S24 FAQ 3.9; S26 refclock PPS | Verified | FAQ 3.9 "smaller than 0.4 seconds"; conf "PPS refclocks do not supply full time" | — |
| 120 | [F6 GNSS/PPS] NMEA large offset; example +504 ms corrected with offset (L177) | S24 FAQ 3.9 | Verified | "common to have a larger offset"; "+504ms" | — |
| 121 | [F6 GNSS/PPS] Serial message timing "accurate to milliseconds"; PPS "much more accurate" (L178) | S24 FAQ 2.13 | Verified | exact | — |
| 122 | [F6 GNSS/PPS] SOCK can beat PPS with sawtooth correction (L179) | S24 FAQ 2.13 | Verified | exact | — |
| 123 | [F6 GNSS/PPS] Three time-pulse error parts and their removal (L180–184) | S30 §2.1 | Verified | §2.1 bullet list | — |
| 124 | [F6 GNSS/PPS] u-blox 6 pulse from 48 MHz clock → jitter (L185) | S30 §1.2.2 | Verified | exact | — |
| 125 | [F6 GNSS/PPS] LEA-6T 6.7 ns deviation; datasheet 30 ns / 15 ns RMS (L186) | S30 §2.1, pp.5–7 | Verified | p.5 "6.7 ns"; p.6–7 "30 ns … 15 ns" | — |
| 126 | [F6 GNSS/PPS] F9T-10B 5 ns 1σ abs, 2.5 ns diff, ±4 ns jitter; compensate propagation delay (L187) | S31 §1.2 | Verified | Table 1 and note p.5 | — |
| 127 | [F6 GNSS/PPS] Holdover: OCXO instead of TCXO (L188) | S30 §2.3.2 | Verified | p.9 "oven-controlled oscillator should be used instead of a temperature-controlled oscillator" | — |
| 128 | [F6 GNSS/PPS] Void GPRMC may keep producing PPS from internal clock (L189) | S43 p.52 | Verified | p.52 exact | — |
| 129 | [F7 Stamping] Four sync primitives (1PPS, network, TOV, trigger) (L192–197) | S14 pp.1–2 | Verified | §II-A list | — |
| 130 | [F7 Stamping] HW timer only deterministic; GPIO not deterministic on Linux; arrival fallback (L198–202) | S14 p.2 | Verified | §II-B list | — |
| 131 | [F7 Stamping] Deep buffers in USB-serial, DAQ boards, non-RT OS; no upper bound (L203) | S11 p.1 | Verified | p.1 exact | — |
| 132 | [F7 Stamping] FTDI send conditions; 16 ms default; single char 16 ms (L204) | S39 p.7 | Verified | PDF p.7 (printed 6) §3.1 | — |
| 133 | [F7 Stamping] FTDI timer 1–255 ms in 1 ms steps (L205) | S39 p.8 | Verified | PDF p.8 | — |
| 134 | [F7 Stamping] Linux FTDI latency timer to 1 ms (L206) | S11 p.1 footnote | Verified | footnote 1 | — |
| 135 | [F7 Stamping] Offset = max(p − q); lowest latency tightest (L207) | S11 p.3 eqs.3–4 | Verified | eqs.(3)–(4) | — |
| 136 | [F7 Stamping] Drift bound; two-pass O(N); causal O(1) (L208) | S11 pp.4–5 | Verified | §III-B, §III-D | — |
| 137 | [F7 Stamping] Never worse than naive (L209) | S11 p.5 Claim 2 | Verified | Claim 2 | — |
| 138 | [F7 Stamping] 0.5 s latency; naive 0.25 s; substantially reduced (L210) | S11 p.6 Fig.6 | Verified | Fig.6 caption | — |
| 139 | [F7 Stamping] MIT DUC: 12 SICK, HDL-64E, 15 radars, Applanix, microcontrollers (L211) | S11 p.2 | Verified | p.2 exact | — |
| 140 | [F7 Stamping] TICSync offset+skew, O(1), probabilistic bounds (L212) | S17 p.1 | Verified | p.1 abstract and intro | — |
| 141 | [F7 Stamping] Better than ms within seconds; 162 ms RTT → ~35 μs (L213) | S17 pp.1,7 | Verified | p.7 "stabilizes at around 35µs … granularity" | — |
| 142 | [F7 Stamping] VersaVIS EKF skew/offset, symmetric delay (L214) | S20 p.3 eqs.1–2 | Verified | p.3 | — |
| 143 | [F7 Stamping] ±5 ms raw → ±0.2 ms after ~60 s (L215) | S20 p.5 | Verified | p.5 exact | — |
| 144 | [F7 Stamping] LaserScan stamp = first ray; time_increment (L216) | S35 LaserScan.msg | Verified | exact | — |
| 145 | [F7 Stamping] KITTI three Velodyne stamps, "rolling shutter" (L217) | S21 p.2 | Verified | p.2 exact | — |
| 146 | [F7 Stamping] KITTI reed contact triggers cameras facing forward (L218) | S21 p.4 | Verified | p.4 exact | — |
| 147 | [F7 Stamping] VersaVIS MCU triggers, HW timer stamps, half-exposure early (L219) | S20 pp.2–3 | Verified | p.2 "hardware timers"; p.3 "starting exposure half the exposure time earlier" | — |
| 148 | [F7 Stamping] Mid-exposure ideal; slope 0.498 vs 0.5 (L220) | S12 pp.5–6 | Verified | p.5 "middle of the exposure time constitutes the ideal point"; p.6 slope 0.498 | — |
| 149 | [F7 Stamping] Nikolic "established fact in photogrammetry"; trigger shift (L221) | S66 p.3 | Verified | p.3 exact | — |
| 150 | [F7 Stamping] IMU delay "in general fixed", compensated by polling time (L222) | S66 p.3 | Verified | p.3 exact | — |
| 151 | [F7 Stamping] "only about 7 µs"; periodic trigger → exposure-dependent (L223) | S66 p.6 Fig.6 | Verified | p.6 caption | — |
| 152 | [F7 Stamping] Start-of-exposure stamping → "a varying, exposure dependent offset" (L224) | S66 p.2 | Verified | p.2 (said of a specific comparison system) | — |
| 153 | [F7 Stamping] Lidar motion distortion; neglect at high rate; "can be severe" when slow (L225) | S67 pp.1–2 | Verified | p.2 exact | — |
| 154 | [F7 Stamping] LOAM constant velocity, per-point linear interpolation, reprojection (L226) | S67 pp.4–5 eq.4 | Verified | p.4–5 | — |
| 155 | [F7 Stamping] Tests at 0.5 m/s cart/vehicle (L227) | S67 p.7 | Verified | p.7 "All tests use a speed of 0.5m/s" | — |
| 156 | [F7 Stamping] FPGA stamping leaves logic/filter/polling delays (L228) | S12 p.5 | Verified | p.5 exact | — |
| 157 | [F7 Stamping] VersaVIS offset depends on IMU filter; compensate in driver/estimator (L229) | S20 p.5 | Verified | p.5 exact | — |
| 158 | [F8 Temporal calib] Furgale joint offset + transform, ML batch, B-splines (L232) | S12 pp.1,3 | Verified | p.1, p.3 | — |
| 159 | [F8 Temporal calib] ±0.2 ms of best fit, "just 4%" (L233) | S12 p.6 Fig.5 | Verified | Fig.5 caption | — |
| 160 | [F8 Temporal calib] ~55°/s and 1.1 m/s² motion (L234) | S12 p.5 | Verified | p.5 exact | — |
| 161 | [F8 Temporal calib] Kalibr temporal calibration on by default; 20 Hz / 200 Hz (L235) | S38 | Verified | "turned on by default"; "camera rate of 20 Hz and an IMU rate of 200 Hz" | — |
| 162 | [F8 Temporal calib] Excite axes, "low jitter timestamps in same clock"; IMU DT bursts 1 ms/6 ms (L236) | S38 "2) Collect images" Tips; "4) The output" WARNING | Partly supported | Tips quote correct; the DT burst text is in section "4) The output" but not under a "WARNING" label (the only WARNING block is in section 2, about symmetric targets) | Cite "4) The output" without "WARNING". |
| 163 | [F8 Temporal calib] Qin & Shen shift features along image-plane velocity (L237) | S18 p.2 | Verified | §III-A/B | — |
| 164 | [F8 Temporal calib] V101 +5/15/30 ms converge in seconds (L238) | S18 pp.6–7 | Verified | Fig.9 caption | — |
| 165 | [F8 Temporal calib] Prior methods as summarized by Qin & Shen (L239–244) | S18 p.2 | Verified | §II Related Work | — |
| 166 | [F8 Temporal calib] Li & Mourikis td in EKF state; applications (L245) | S53 PDF p.1 | Verified | Abstract | — |
| 167 | [F8 Temporal calib] Locally identifiable except degenerate; zero / constant rotation (L246) | S53 PDF pp.9–10 | Verified | p.10 "cases of zero or constant rotational velocity result in loss of observability even if the time offset … is perfectly known" | — |
| 168 | [F8 Temporal calib] MTi-G 100 Hz, camera 20 Hz; converge in seconds; final std 0.40 ms (L247) | S53 PDF pp.10–11 | Verified | p.11 "standard deviation of td at the end … only 0.40 msec" | — |
| 169 | [F8 Temporal calib] td 20→520 ms over 500 s, "severe clock drift", consistent (L248) | S53 PDF p.15 | Verified | p.15 exact | — |
| 170 | [F8 Temporal calib] Kelly et al.: delay in EKF state → bias, inconsistency, initial-condition sensitivity (L249) | S19 pp.1,8 | Verified | p.1 abstract; p.7–8 | — |
| 171 | [F8 Temporal calib] Remedy: sliding window covering any feasible delay (L250) | S19 p.8 | Verified | p.8 exact | — |
| 172 | [F8 Temporal calib] Constant offsets calibratable; changing ones critical (L251) | S20 p.4 | Verified | p.4 exact | — |
| 173 | [F9 ROS 2] Three time abstractions (L254) | S13 Time Abstractions | Verified | exact | — |
| 174 | [F9 ROS 2] ROSTime = SystemTime unless use_sim_time; /clock (L255) | S13 | Verified | "ROS Time", "Default Time Source" | — |
| 175 | [F9 ROS 2] Zero = uninitialized (L256) | S13 Default Time Source | Verified | exact | — |
| 176 | [F9 ROS 2] SteadyTime not comparable (L257) | S13 Implementation | Verified | exact | — |
| 177 | [F9 ROS 2] Backward jumps; jump callbacks API (L258) | S13 | Verified | "Challenges…", "Public API" | — |
| 178 | [F9 ROS 2] Synchronized system clock assumed; GPS via NTP (L259) | S13 Background, Custom Time Source | Verified | exact | — |
| 179 | [F9 ROS 2] Header sec+nsec stamp and frame_id (L260) | S35 Header.msg | Verified | exact | — |
| 180 | [F9 ROS 2] Image stamp "should be acquisition time of image" (L261) | S35 Image.msg | Verified | exact | — |
| 181 | [F9 ROS 2] TimeReference pairs system stamp and external time (L262) | S35 TimeReference.msg | Verified | exact | — |
| 182 | [F9 ROS 2] tf2 default cache 10 s (L264) | S32; S33 | Verified | BUFFER_CORE_DEFAULT_CACHE_TIME, TIMECACHE_DEFAULT_MAX_STORAGE_TIME = 10 s; S33 "up to 10 seconds by default" | — |
| 183 | [F9 ROS 2] Too-old data rejected; TF_OLD_DATA (L265) | S32 cache.cpp insertData; buffer_core.cpp l.295 | Verified | insertData "exceeds the max_storage_time_"; l.295 message | — |
| 184 | [F9 ROS 2] Linear + slerp interpolation (L267) | S32 cache.cpp interpolate | Verified | setInterpolate3 / slerp | — |
| 185 | [F9 ROS 2] Future / past extrapolation errors (L268) | S32 cache.cpp findClosest | Verified | Messages at cache.cpp l.85, l.95, raised via findClosest | — |
| 186 | [F9 ROS 2] Time 0 = latest; chain uses latest common time (L269) | S33 §1; S32 getLatestCommonTime | Verified | S33 "time 0 means 'the latest available'" | — |
| 187 | [F9 ROS 2] "usually a couple of milliseconds"; optional timeout (L270) | S33 §§1–2 | Verified | exact | — |
| 188 | [F9 ROS 2] MessageFilter caches until transformable (L271) | S33 tf2 message filter doc | Verified | l.36 exact | — |
| 189 | [F9 ROS 2] FilterFailureReason OutTheBack, QueueFull (L272) | S32 message_filter.hpp | Verified | enum l.80–96 | — |
| 190 | [F9 ROS 2] transform_timeout default 0 = latest; non-zero may miss rate (L273) | S36 ~transform_timeout | Verified | exact | — |
| 191 | [F9 ROS 2] ExactTime identical stamps (L275) | S34 index.rst 6.2 | Verified | exact | — |
| 192 | [F9 ROS 2] ApproximateEpsilonTime epsilon (L276) | S34 6.3 | Verified | exact | — |
| 193 | [F9 ROS 2] ApproximateTime adaptive (L277) | S34 6.4 | Verified | exact | — |
| 194 | [F9 ROS 2] Python allow_headerless uses current ROS time (L279) | S34 6.4 | Verified | exact | — |
| 195 | [F9 ROS 2] Warns once on out-of-order (L280) | S34 approximate_time.h l.193 | Verified | RCUTILS_LOG_WARN_ONCE "arrived out of order" | — |
| 196 | [F9 ROS 2] age penalty 0.1, unlimited max interval, queue 1 drops many (L281) | S34 approximate_time.h ll.121–127 | Verified | constructor and comment | — |
| 197 | [F9 ROS 2] TimeSequencer delay, order, discards older (L282) | S34 index.rst §4 | Verified | exact | — |
| 198 | [F9 ROS 2] Kronauer overhead up to 50 % (per Bédard) (L283) | S22 p.2 | Verified | p.2 exact | — |
| 199 | [F9 ROS 2] Up to 50 % for 128 B; DDS + rclcpp notification delay dominate (L284) | S60 pp.4–5 | Verified | §4.3 p.5 exact | — |
| 200 | [F9 ROS 2] Rules of thumb; energy saving; scaling governor (L285) | S60 pp.4,6–7 | Verified | p.6 rules list; p.7 "highly depends on energy saving features"; p.4 governor | — |
| 201 | [F9 ROS 2] rmw source/received timestamps; publication_sequence_number "shows how many messages were lost" (L286) | S61 rmw_message_info_s | Partly supported | Timestamps text exact; but psn2−psn1−1 is the number of messages sent in between, which "might have already been taken by other rmw_take*() calls … or lost"; also may be UNSUPPORTED | Say "how many were sent in between (lost or taken elsewhere), where the rmw supports it". |
| 202 | [F9 ROS 2] getTransform latest + transform_tolerance timeout (L288) | S63 robot_utils.cpp | Verified | l.98 TimePointZero, transform_tolerance | — |
| 203 | [F9 ROS 2] ObservationBuffer transforms at cloud stamp (L289) | S63 bufferCloud | Verified | l.98, l.116–117 | — |
| 204 | [F9 ROS 2] Purge by keep time; 0 keeps newest only (L290) | S63 purgeStaleObservations | Verified | l.192–205 | — |
| 205 | [F9 ROS 2] isCurrent warning; 0 disables (L291) | S63 isCurrent | Verified | l.214–227 | — |
| 206 | [F9 ROS 2] Docs defaults transform_tolerance 0.3; persistence and update rate 0.0 (L292) | S64 | Verified | index.md l.131; obstacle.md l.80, l.86 | — |
| 207 | [F10 Verification] Root distance = dispersion + ½ root delay; + System time (L295) | S24 FAQ 7.2 | Verified | exact | — |
| 208 | [F10 Verification] maxclockerror default 1 ppm (L296) | S24 FAQ 7.2 | Verified | exact | — |
| 209 | [F10 Verification] noselect + sourcestats comparison (L297) | S24 FAQ 3.9 | Verified | exact | — |
| 210 | [F10 Verification] Rubidium-referenced receiver; 1 s samples over 6 h (L298) | S30 §2.1 | Verified | §2.1 exact | — |
| 211 | [F10 Verification] LED board, 400 pairs, better than 0.5 ms (L299) | S20 p.4 | Verified | p.4 | — |
| 212 | [F10 Verification] Repeated Kalibr offsets consistent, < 0.05 ms (L300) | S20 p.5 | Verified | "synchronization accuracy below 0.05 ms" | — |
| 213 | [F10 Verification] T265 bimodal offset "about ±15 ms apart (half a frame period)" (L301) | S20 p.5 | Partly supported | "bi-modal distribution delimited by half the inter-frame time of ≈ 15 ms" — modes ~15 ms apart, not ±15 ms | Write "modes about 15 ms apart". |
| 214 | [F10 Verification] ros2_tracing LTTng; 0.0033 ms (L302) | S22 pp.1,5 | Verified | p.1 abstract | — |
| 215 | [F10 Verification] TimeStampStatus −1 s / 5 s / zero stamps (L303–307) | S37 update_functions.hpp | Verified | l.233–236, l.364–376 | — |
| 216 | [F10 Verification] Velodyne driver uses TimeStampStatus (L308) | S44 driver.cpp ll.169,282 | Verified | l.169 TimeStampStatusParam; l.282 tick(stamp) | — |
| 217 | [F10 Verification] Kalibr: inspect IMU DTs (L309) | S38 "4) The output" WARNING | Partly supported | Text is in "4) The output" but not under a WARNING label | Drop "WARNING". |
| 218 | [F10 Verification] PPS comparison, ~5.9 ns; NICs lacked PPS output (L310) | S59 pp.4–5 | Verified | p.5 "approx. 5.9 ns"; "none of the NIC-s have easily available PPS output" | — |
| 219 | [F10 Verification] Glass-to-glass LED + phototransistor, 2 kHz, 0.5 ms, sampling-limited (L311) | S65 pp.1,3 | Verified | p.1 abstract; p.3 "sampled at 2kHz" | — |
| 220 | [F10 Verification] LED negligible; 10 μs rise/fall (L312) | S65 p.2 | Verified | p.2 exact | — |
| 221 | [F10 Verification] Uniform camera share; 19.1–52.4 ms, σ 6.9 ms at 50 Hz (L313) | S65 pp.3–4 | Verified | p.3 uniform; p.4 values | — |
| 222 | [F10 Verification] Earlier method 200 Hz, "average imprecision of 5 milliseconds" (L314) | S65 p.2 | Verified | exact | — |
| 223 | [F11 Domains] MIT DUC passive sync (L317) | S11 p.2 | Verified | p.2 | — |
| 224 | [F11 Domains] KITTI trigger, nearest 100 Hz sample, 5 ms, host clock (L318) | S21 p.4 | Verified | p.4 exact | — |
| 225 | [F11 Domains] DRIVE OS ptp4l automotive profile; TSC aligned via 1 Hz PPS; camera fsync (L319) | S46 | Verified | "PTP - TSC HW synchronization … 1HZ PPS"; "AVNU PTP for Development" | — |
| 226 | [F11 Domains] OOSMs from lidar/radar/camera even on FlexRay/TTCAN (L320) | S16 pp.1–2 | Verified | p.1–2 | — |
| 227 | [F11 Domains] Syncline curves compare platforms (L321) | S14 pp.1,4 | Verified | p.4 platform table | — |
| 228 | [F11 Domains] Lander latency modelling + error budget (L322) | S15 pp.1,27 | Verified | p.1 abstract; p.27 | — |
| 229 | [F11 Domains] HW-triggered mid-exposure VI suites sub-ms (L323) | S20 p.1; S12 p.6 | Verified | S20 abstract "less than 1 ms"; S12 ±0.2 ms | — |
| 230 | [F11 Domains] 1588 original target industrial measurement/control (L324) | S05 p.10 | Verified | slide 10 | — |
| 231 | [F11 Domains] AUTOSAR targets airbag/braking (L325) | S58 PDF p.5 §1.2 | Verified | §1.2 exact | — |
| 232 | [F11 Domains] TSN gPTP accuracy falls with hops (L326) | S59 p.1 | Verified | abstract | — |
| 233 | [F11 Domains] Factor-graph smoothers take delayed GPS at true time (L327) | S57 pp.2,4 | Verified | p.2, p.4 | — |
| 234 | [F12 Products] SampleTimeFine 10 kHz ticks; Coarse s; combined (L331) | S41 pp.47–48 | Verified | p.47–48 | — |
| 235 | [F12 Products] UtcTime ns, date, time, flags (L332) | S41 p.47 | Verified | p.47 | — |
| 236 | [F12 Products] GnssPvtPulse same clock domain (L333) | S41 p.52 | Verified | p.52 exact | — |
| 237 | [F12 Products] Clock bias: 10 ppm; 670G/680G always GNSS-referenced, not configurable (L335) | S40 p.37 | Verified | PDF p.37 | — |
| 238 | [F12 Products] ClockSync: timestamps on unadjusted internal clock (L336) | S40 p.38 | Verified | PDF p.38 exact | — |
| 239 | [F12 Products] 1PPS time-pulse always enabled on 680G (L337) | S40 p.38 | Verified | exact | — |
| 240 | [F12 Products] Two SyncIn, one SyncOut (L339) | S40 p.35 | Verified | exact | — |
| 241 | [F12 Products] SyncIn functions incl. TriggerIndication timestamped (L340) | S40 pp.36–38 | Verified | p.36–37 | — |
| 242 | [F12 Products] SyncOut 400 Hz; 1 Hz synced to GNSS 1PPS (L341) | S40 pp.35,38 | Verified | p.38 "400 Hz SDI sampling clock"; p.35 1 Hz | — |
| 243 | [F12 Products] NMEA-input mode syncs clock to UTC (L342) | S40 p.30 | Verified | exact | — |
| 244 | [F12 Products] time_option 0 UTC (default, recommended) (L344) | S42 yaml | Verified | l.69, l.74 | — |
| 245 | [F12 Products] time_option 1 SampleTimeFine (L345) | S42 yaml | Verified | l.70 | — |
| 246 | [F12 Products] time_option 2 host time (L346) | S42 yaml | Verified | l.71 | — |
| 247 | [F12 Products] First packet host now(); +timeDiff×1e5 ns; wraparound (L347) | S42 xsens_time_handler.cpp | Verified | l.79, l.85–105 | — |
| 248 | [F12 Products] Missing requested field → host now() (L348) | S42 xsens_time_handler.cpp | Partly supported | With time_option 0 and no UTC, the code first falls back to SampleTimeFine (l.70); host now() only when neither applies (l.116) | Describe the two-step fallback. |
| 249 | [F12 Products] VLP-16 TOH μs 0–3,599,999,999, internal oscillator, every packet (L351) | S43 pp.133–134 | Verified | p.133–134 | — |
| 250 | [F12 Products] PPS rising edge realigns sub-second; GPRMC/GPGGA sets min/s (L352) | S43 pp.43,61,134 | Verified | p.43 GPRMC or GPGGA; p.134 | — |
| 251 | [F12 Products] PPS lock status 0x02 (L353) | S43 p.134 | Verified | exact | — |
| 252 | [F12 Products] Unstable PPS → free-run; Delay default 5 s (L354) | S43 p.135 | Verified | exact | — |
| 253 | [F12 Products] ≥50 ms / ≥300 ms gaps; pulse width 10 μs–200 ms (L355) | S43 p.44 | Verified | exact | — |
| 254 | [F12 Products] Packet stamp = first point; 55.296 / 2.304 μs offsets (L356) | S43 p.68 | Verified | exact | — |
| 255 | [F12 Products] gps_time false → mean of times around recvfrom + time_offset (L357) | S44 input.cpp getPacket | Verified | l.214–250; driver.cpp l.62 default false | — |
| 256 | [F12 Products] gps_time true → TOH + host hour; ±1 h if > half hour (L358) | S44 time_conversion.hpp | Verified | resolveHourAmbiguity, rosTimeFromGpsTimestamp | — |
| 257 | [F12 Products] Scan stamp last packet unless timestamp_first_packet (L359) | S44 driver.cpp ll.274–276 | Verified | exact | — |
| 258 | [F12 Products] ZED common low-drift clock; stamped on host reception, Epoch ns (L362) | S45 time-sync page | Verified | exact | — |
| 259 | [F12 Products] TIME_REFERENCE IMAGE vs CURRENT (L363) | S45 | Verified | API enum and time-sync page | — |
| 260 | [F12 Products] IMAGE at readout start; center of exposure half earlier (L364) | S45 API TIME_REFERENCE | Verified | exact | — |
| 261 | [F12 Products] CENTER_OF_EXPOSURE exact on ZED X, getTimestamp only, 0 on USB (L365) | S45 API | Verified | Note text | — |
| 262 | [F12 Products] Wrapper live stamps with TIME_REFERENCE::IMAGE (L366) | S62 main.cpp ll.5086–5103 | Verified | l.5101–5102 | — |
| 263 | [F12 Products] use_pub_timestamps (default false) → now(); comment re latency (L367) | S62 yaml l.200; main.cpp l.744 | Verified | yaml l.200 exact; video_depth.cpp l.1825 uses now() | — |
| 264 | [F12 Products] Sensors stamped with SDK times unless sensors_image_sync (L368) | S62 main.cpp ll.5329–5366; video_depth.cpp ll.2784–2795; yaml l.61 | Verified | code and yaml as cited | — |
| 265 | [F12 Products] SVO without use_svo_timestamps → CURRENT (L369) | S62 main.cpp ll.5086–5092 | Verified | l.5089–5090 | — |
| 266 | [F12 Products] FTDI 16 ms default, 1–255 ms (L372) | S39 pp.7–8 | Verified | as above | — |
| 267 | [F12 Products] Syncline: FTDI "up to 16ms buffering delay" (L373) | S14 p.2 | Verified | p.2 exact | — |
| 268 | [F12 Products] Jetson r36.4.4 EQOS lists "IEEE 1588-2008 (PTP)" (L376) | S47 | Verified | l.984 | — |
| 269 | [F12 Products] GTE deprecated from JP 6.0, replaced by HTE (L377) | S48 | Verified | exact | — |
| 270 | [Rec practice] 1 Stamp at acquisition; HW timers/triggers/PPS (L380) | S14 p.2; S12 p.1 | Verified | S14 §II-B | — |
| 271 | [Rec practice] 2 Lowest-latency skew-aware mapping (L381) | S11 p.5; S17 p.1; S20 p.3 | Verified | as above | — |
| 272 | [Rec practice] 3 Mid-exposure stamping / half-exposure early trigger (L382) | S12 p.5; S20 pp.2–3; S45 | Verified | as above | — |
| 273 | [Rec practice] 4 NIC HW timestamping, several sources (L383) | S26 hwtimestamp; S24 FAQ 2.7 | Verified | FAQ 2.7 "better to use more than one server" | — |
| 274 | [Rec practice] 5 Step only at boot (L384) | S26 makestep | Verified | as above | — |
| 275 | [Rec practice] 6 Pair PPS with ToD; set NMEA delay/offset (L385) | S24 FAQ 3.9; S27 | Verified | as above | — |
| 276 | [Rec practice] 7 Compensate fixed delays (L386) | S31 §1.2; S30 §2.1 | Verified | as above | — |
| 277 | [Rec practice] 8 Calibrate with rich motion; check consistency (L387) | S12 p.5; S38; S20 p.5 | Verified | as above | — |
| 278 | [Rec practice] 9 Fuse lagged data at true time; history ≥ lag (L388) | S36; S09 p.2 | Verified | as above ("should") | — |
| 279 | [Rec practice] 10 Monitor stamp age (L389) | S37 | Verified | as above | — |
| 280 | [Rec practice] 11 Reduce FTDI latency timer (L390) | S39 p.8 | Verified | as above | — |
| 281 | [Rec practice] 12 Compensate fixed sensor delays in acquisition (L391) | S66 p.3 | Verified | as above | — |
| 282 | [Rec practice] 13 Per-point lidar de-skew (L392) | S67 pp.1–2,4–5 | Verified | as above | — |
| 283 | [Rec practice] 14 Report latency distribution (L393) | S65 pp.3–4 | Verified | as above | — |
| 284 | [Key numbers] 15.7 cm (L398) | S11 p.1 | Verified | exact | — |
| 285 | [Key numbers] "hundreds of milliseconds", loaded non-RT OS (L399) | S11 p.1 | Verified | exact | — |
| 286 | [Key numbers] FTDI 16 ms; 1–255 ms (L400) | S39 pp.7–8 | Verified | exact | — |
| 287 | [Key numbers] NTP few ms, 1991 (L401) | S01 p.1 | Verified | exact | — |
| 288 | [Key numbers] NTPv4 tens μs / few hundred μs (L402) | S02 §1 | Verified | exact | — |
| 289 | [Key numbers] chrony 35 ± 8 μs (L403) | S25 Test 1 | Verified | exact | — |
| 290 | [Key numbers] chrony HW sub-μs (L404) | S24 FAQ 2.7 | Verified | exact | — |
| 291 | [Key numbers] 125 ms / 1000 s (L405) | S02 §11.3 | Verified | l.2708–2710 table | — |
| 292 | [Key numbers] PTP ~100 ns; <20 ns better hardware (L406) | S05 p.73 | Verified | slide 73 (see #row for L151 on wording) | — |
| 293 | [Key numbers] F9T 5 / 2.5 / ±4 ns (L407) | S31 §1.2 | Verified | exact | — |
| 294 | [Key numbers] LEA-6T 30 / 15 ns RMS (L408) | S30 pp.6–7 | Verified | exact | — |
| 295 | [Key numbers] PPS pairing < 0.4 s (0.2 s pre-4.1) (L409) | S24 FAQ 3.9 | Verified | exact | — |
| 296 | [Key numbers] 0.01 %; several ppm (L410) | S01 p.11 | Verified | exact | — |
| 297 | [Key numbers] MTi-600 ~10 ppm (L411) | S40 p.37 | Verified | exact | — |
| 298 | [Key numbers] 92 s / 6 days; >30 ms (L412) | S17 p.1 | Verified | exact | — |
| 299 | [Key numbers] "Passive clock mapping (TICSync)" ~35 μs, 162 ms RTT (L413) | S17 p.7 | Partly supported | Number and condition correct, but TICSync uses two-way round-trip exchanges; it is not a passive method | Relabel "Two-way clock mapping (TICSync)". |
| 300 | [Key numbers] VersaVIS ±5 → ±0.2 ms; 1 s updates, ~60 s (L414) | S20 p.5 | Verified | exact | — |
| 301 | [Key numbers] ±0.2 ms, 4 %, 40 datasets (L415) | S12 p.6 | Verified | exact | — |
| 302 | [Key numbers] KITTI 5 ms worst case (L416) | S21 p.4 | Verified | exact | — |
| 303 | [Key numbers] tf2 10 s (L417) | S32 buffer_core.hpp l.73 | Verified | exact | — |
| 304 | [Key numbers] VLP-16 55.296 / 2.304 μs (L418) | S43 p.68 | Verified | exact | — |
| 305 | [Key numbers] SampleTimeFine 10 kHz (L419) | S41 p.47 | Verified | exact | — |
| 306 | [Key numbers] ros2_tracing 0.0033 ms (L420) | S22 p.1 | Verified | exact | — |
| 307 | [Key numbers] XO/TCXO/OCXO accuracy hierarchy (L421) | S55 PDF p.33 | Verified | exact | — |
| 308 | [Key numbers] TCXO 5e-7 / OCXO 1e-9 temp stability (L422) | S55 PDF p.237 | Verified | exact | — |
| 309 | [Key numbers] GPS–UTC 1 μs; 20 ns (L423) | S56 §3.3.4 | Verified | exact | — |
| 310 | [Key numbers] Week number 10 bits, mod 1024 (L424) | S56 §20.3.3.3.1.1 | Verified | exact | — |
| 311 | [Key numbers] Leap smear 62500 s, 32 ppm (L425) | S26 leapsecmode | Verified | exact | — |
| 312 | [Key numbers] ptp4l PI 0.7/0.3; 0.1/0.001 (L426) | S27 ptp4l.8 | Verified | exact | — |
| 313 | [Key numbers] ROS 2 overhead up to 50 %, 128 B (L427) | S60 p.5 | Verified | exact | — |
| 314 | [Key numbers] Online td std 0.40 ms (L428) | S53 PDF p.11 | Verified | exact | — |
| 315 | [Key numbers] ~7 µs HW-synced camera–IMU (L429) | S66 p.6 | Verified | exact | — |
| 316 | [Key numbers] G2G 0.5 ms, 2 kHz (L430) | S65 p.1 | Verified | exact | — |
| 317 | [Key numbers] PPS resolution ~5.9 ns at 195.5 MHz (L431) | S59 p.5 | Verified | exact | — |
| 318 | [How tested] Time pulse vs rubidium reference; 6.7 ns, mean −2.35 ns (L436) | S30 §2.1 | Verified | exact | — |
| 319 | [How tested] Root distance / chronyc tracking (L437) | S24 FAQ 7.2 | Verified | exact | — |
| 320 | [How tested] noselect vs NTP in sourcestats (L438) | S24 FAQ 3.9 | Verified | exact | — |
| 321 | [How tested] LED counter, 400 pairs, <0.5 ms (L439) | S20 p.4 | Verified | exact | — |
| 322 | [How tested] Repeated Kalibr, <0.05 ms (L440) | S20 p.5 | Verified | exact | — |
| 323 | [How tested] Offset vs exposure slope 0.498; ±0.2 ms (L441) | S12 p.6 | Verified | exact | — |
| 324 | [How tested] Injected offsets converge in seconds (L442) | S18 pp.6–7 | Verified | exact | — |
| 325 | [How tested] Clock translation innovation ±0.2 ms (L443) | S20 p.5 | Verified | exact | — |
| 326 | [How tested] TimeStampStatus [−1 s, 5 s], no zero (L444) | S37 | Verified | exact | — |
| 327 | [How tested] IMU timestamp interval plot; 1 ms bursts / 6 ms gaps (L445) | S38 "WARNING" | Partly supported | Content correct; text is in "4) The output", not a WARNING block | Cite "4) The output". |
| 328 | [How tested] ros2_tracing trace, not pass/fail (L446) | S22 p.5 | Verified | p.5 Fig.3 | — |
| 329 | [How tested] PPS input capture; step response vs model (L447) | S59 pp.4–5 | Verified | p.5–6 | — |
| 330 | [How tested] G2G min/max match frame model 19.1–52.4 ms (L448) | S65 pp.2–4 | Verified | p.4 | — |
| 331 | [How tested] Periodic vs compensated triggering; ~7 µs (L449) | S66 p.6 | Verified | exact | — |
| 332 | [How tested] Source vs received stamps; "sequence numbers expose lost messages" (L450) | S61 | Partly supported | Sequence gap counts messages sent in between, which may be lost or taken by other take calls; may be unsupported | Say "sequence gaps count messages not received by this take (lost or taken elsewhere)". |
| 333 | [Common mistakes] Arrival stamping on loaded host (L453) | S11 p.1 | Verified | exact | — |
| 334 | [Common mistakes] FTDI 16 ms for small messages (L454) | S39 p.7 | Verified | exact | — |
| 335 | [Common mistakes] 62 bytes within 16 ms → 64-byte packet every 16 ms (L455) | S39 p.7 | Verified | exact | — |
| 336 | [Common mistakes] Symmetric path assumption; 55 vs 60 (L456) | S05 p.24; S24 FAQ 7.2 | Verified | exact | — |
| 337 | [Common mistakes] Steps reset associations; step only at boot (L457) | S02 §11.2.3; S26 makestep | Verified | exact | — |
| 338 | [Common mistakes] Uncorrected NMEA delay → wrong second (L458) | S24 FAQ 3.9; S27 | Verified | FAQ 3.9; ts2phc.nmea_delay | — |
| 339 | [Common mistakes] Offset without skew drifts (L459) | S20 p.5 | Verified | "constantly decreasing offset … importance of estimating the skew" | — |
| 340 | [Common mistakes] Readout vs mid-exposure (L460) | S12 p.6; S45 | Verified | as above | — |
| 341 | [Common mistakes] tf2 at now without timeout (L461) | S33 §1 | Verified | exact | — |
| 342 | [Common mistakes] robot_localization lagged data without smoothing (L462) | S36 filter_base.cpp | Verified | as above | — |
| 343 | [Common mistakes] ApproximateTime queue 1 (L463) | S34 approximate_time.h l.127 | Verified | exact | — |
| 344 | [Common mistakes] Void status with PPS (L464) | S43 p.52 | Verified | exact | — |
| 345 | [Common mistakes] Start-of-exposure stamping (L465) | S66 p.2 | Verified | exact | — |
| 346 | [Common mistakes] Leap second carries/borrows (L466) | S56 §20.3.3.5.2.4 b | Verified | exact | — |
| 347 | [Common mistakes] Mixing smearing servers (L467) | S26 leapsecmode | Verified | exact | — |
| 348 | [Common mistakes] Lidar sweep as one instant (L468) | S67 pp.1–2 | Verified | exact | — |
| 349 | [Common mistakes] Power-saving on during latency tests (L469) | S60 pp.4,7 | Verified | exact | — |
| 350 | [Disagreements] Harrison & Newman: NTP undesirable (L473) | S17 p.1 | Verified | exact | — |
| 351 | [Disagreements] ROS 2 design: integrate GPS via NTP (L474) | S13 Custom Time Source | Verified | exact | — |
| 352 | [Disagreements] chrony fast sync; sub-μs with HW (L475) | S25 Summary; S24 FAQ 2.7 | Verified | "can usually synchronise the clock faster and with better time accuracy" | — |
| 353 | [Disagreements] "Sources differ mainly in which NTP implementation and configuration they assume" (L476) | none | Partly supported | Uncited reviewer inference; no source states it | Label as an inference. |
| 354 | [Disagreements] Skog "almost perfect time synchronization" (L478) | S10 PDF p.11 | Partly supported | Quote is on PDF p.15 (printed p.3) | Change to PDF p.15. |
| 355 | [Disagreements] Qin & Shen summarize Li & Mourikis as "successful" (L479) | S18 p.2 | Partly supported | S18 p.2 describes Li's MSCKF method neutrally ("significant advantage in computation complexity") and says its own method "outperforms in term of accuracy"; it does not call it successful | Reword to "describe it as computationally efficient but less accurate than their own". |
| 356 | [Disagreements] Kelly et al.: EKF delay state flawed (L480) | S19 pp.1,8 | Verified | as above | — |
| 357 | [Disagreements] "Optimization and batch methods are not criticized on these grounds" (L481) | S12; S18 | Partly supported | Unlabelled inference from absence; S19 p.8 itself proposes a sliding-window estimator, which is the relevant support | Label as inference and cite S19 p.8. |
| 358 | [Disagreements] Mauthner "find" buffering can compete (L483) | S16 p.10 | Partly supported | Conclusion: "We have hinted at the possibility, that the simple buffering approach can be competitive" | Use "suggest" / "hint". |
| 359 | [Disagreements] Brouk & DeMars: reprocessing/smoothing burdensome and lagging (L484) | S15 pp.2–3 | Verified | exact | — |
| 360 | [Disagreements] Brouk & DeMars: timing hardware > processing (L486) | S15 p.27 | Verified | exact | — |
| 361 | [Disagreements] Larsen: real differences between methods (L487) | S09 p.6 | Verified | Table 1–2 differ across methods | — |
| 362 | [Disagreements] Indelman: buffering approximate; smoother handles delay easily (L489) | S57 pp.1–2 | Verified | exact | — |
| 363 | [Disagreements] Brouk: fixed-lag heavy and lagging (L490) | S15 pp.2–3 | Verified | exact | — |
| 364 | [Disagreements] Mauthner "find" buffering competitive (L491) | S16 p.10 | Partly supported | Same as L483: "hinted at the possibility" | Use "suggest". |
| 365 | [Disagreements] GPS spec expects 23:59:60, allows decrementing (L493) | S56 §20.3.3.5.2.4 b | Verified | "may be designed to approximate UTC by decrementing" | — |
| 366 | [Disagreements] chrony default kernel step at midnight (L494) | S26 leapsecmode | Verified | exact | — |
| 367 | [Disagreements] chrony smear ~17 h, identical servers (L495) | S26 leapsecmode | Verified | exact | — |
| 368 | [Disagreements] Li & Mourikis consistent even with drift vs Kelly (L496) | S53 PDF pp.11,15; S19 pp.1,8 | Verified | S53 p.15 "estimates remain consistent" | — |
| 369 | [Open questions] S49–S54 read only through secondary descriptions (L499–505) | Sources table | Verified | Sources table marks S49–S52, S54 "not downloaded" | — |
| 370 | [Open questions] Vig gives range-level stabilities (L506) | S55 | Verified | slides 2-8, 7-1 are range-level | — |
| 371 | [Open questions] Only GPS spec read for leap/rollover (L507) | S56 | Verified | — | — |
| 372 | [Open questions] AUTOSAR refers to 802.1AS Annex B.1.2, no number (L508) | S58 | Verified | §1.2.3 | — |
| 373 | [Open questions] Factor-graph late-measurement evidence is one simulation (L509) | S57 | Verified | p.1 abstract "simulated environment" | — |
| 374 | [Open questions] ZED wrapper stamping covered by S62 (L512) | S62 | Verified | as above | — |
| 375 | [Open questions] G2G rigs for video, 0.5 ms (L513) | S65 | Verified | p.1 | — |

## Corrections applied (2026-09-28)

Editor pass applying this claim review and SOURCE_AUDIT.md. Each Partly supported claim was re-checked against its source before rewriting. README Status set to **Verified**: no Partly supported or Not supported claims and no failing sources remain.

### Claims (all 25 Partly supported items)
- L22 (Summary): now "from arrival or round-trip timing"; Olson is arrival-only, TICSync and VersaVIS use two-way exchanges. VersaVIS cite moved to pp. 6, 11 (see S20 below).
- L35 (S49 Cristian): marked "described via L5-S11, p. 2"; the "behind NTP and PTP" claim removed and replaced by Olson's statement that most schemes are variants of Cristian's algorithm.
- L36 (S05/S50): "Eidson chaired the 1588 committee" removed; row now says what the tutorial covers.
- L52, L53 (S08): PDF p. 28 → p. 29. L53 reworded to "many frequency sources, for example quartz crystal and rubidium oscillators".
- L55 (S08): PDF p. 23 → p. 24.
- L88, L89, L478 (S10): PDF p. 11 → p. 15.
- L114 (S36): "must" → "the documentation says it should be".
- L151 (S05 p. 73): now "some combination of" faster sampling, better oscillators, boundary clocks, statistics and servo algorithms, and environment/temperature control.
- L159 (S27 ts2phc): the condition added (needed when the maximum NMEA delay can exceed 1 s, or the pulse width when both edges are stamped; default 0 ns).
- L236, L309, L445 (S38): "WARNING" dropped; cited as "4) The output".
- L286, L450 (S61): the sequence-number gap now counts messages sent in between (lost or taken by other `rmw_take` calls), where the rmw supports it.
- L301 (S20): "modes about 15 ms apart", which the authors relate to half the inter-frame time; one-frame shifts given as the authors' reading.
- L348 (S42): two-step fallback described (UTC → SampleTimeFine → host `now()`).
- L413 (S17): relabelled "Two-way clock mapping (TICSync)".
- L476: labelled as an inference of this review.
- L479 (S18 p. 2): reworded to "significant advantage in computational complexity, but less accurate than their own optimization-based method".
- L481: replaced by S19 p. 8 (sliding-window remedy) plus an explicit inference label.
- L483, L491 (S16 p. 10): "find" → "suggest" ("hinted at the possibility").
- Optional note on L15 applied: "on a loaded non-real-time OS".
- L292 (S64): defaults labelled as rolling documentation, not checked against the Humble source.

### Sources
- S01: DOI 10.1109/26.103043 added (checked via Crossref).
- S03: DOI added as **10.1109/90.731182**. The audit's DOI 10.1109/90.731187 resolves (Crossref) to a different paper ("Performance of checksums and CRCs over real data"), so it was not used.
- S04: file replaced with the author-hosted PDF (https://lamport.azurewebsites.net/pubs/time-clocks.pdf, 8 pages, text layer); same file name. DOI 10.1145/359545.359563 added. Cited location re-checked: abstract on PDF p. 1 = printed p. 558; citation now "p. 558 (PDF p. 1)".
- S16: peer-review status of the FAS 2006 workshop is not stated in the file or found elsewhere; level A → C, reason given in the citation. Venue location added.
- S20: arXiv v1 replaced by the published MDPI *Sensors* PDF (18 pages, CC-BY), fetched from MDPI's CDN because the main MDPI URL bot-blocks scripts; same file name. DOI 10.3390/s20051439 and full author list added. All 18 S20 citations re-located in the new PDF: p. 5 → p. 11 (clock translation ±5 → ±0.2 ms, ~60 s); p. 3 eqs. 1–2 → p. 6; pp. 2–3 → pp. 4–5 (MCU timers, half-exposure trigger); p. 4 LED board → pp. 8–9; p. 4 constant vs changing offsets → p. 9; p. 5 IMU-filter dependence, <0.05 ms consistency and T265 bimodal → p. 10; p. 5 skew drift → pp. 10–11; p. 1 unchanged.
- S22: pages 6511–6518 and DOI 10.1109/LRA.2022.3174346 added (checked via Crossref).
- S30: link changed from the u-blox homepage to the document URL (downloaded file is byte-identical to the saved copy); currency caveat (u-blox 6 generation, general principles only) added.
- S35: all five file names written out in full.
- S43: year 2019 → 2022 (last updated 2022-03-07), "DRAFT" and Ouster hosting noted; file renamed `velodyne_2022_vlp16_user_manual_revf.pdf`.
- S45: links changed to the real docs.stereolabs.com pages; version given as live ZED SDK 5.x pages with no version number printed.
- S55: level A → B (tutorial, not peer-reviewed; consistent with S05); third-party mirror noted (IEEE UFFC copy returned 404/403).
- S64: marked as rolling documentation in the citation.
- Foundational references: S13 row completed with the tf2 and message_filters sources named in SCOPE (S32, S33, S34); S50 row now also names Eidson's book; S04 row gives the CACM volume.
- No source failed the audit; no files or rows were deleted.

### Final mechanical check
- `file` on all 91 files in `sources/`: every `.pdf` is a PDF; every other file is text or troff, none HTML.
- Every file in `sources/` appears in the Sources table; every table row has a file or is marked *not downloaded* with a reason.
- Every cited ID exists in the table, and every table ID is cited. IDs run L5-S01 to L5-S67 with no gaps or duplicates.
- Counts: 312 cited bullets and 66 cited table rows (Foundational 17, Key numbers 34, How tested 15) in README.md; 67 rows in the Sources table.
