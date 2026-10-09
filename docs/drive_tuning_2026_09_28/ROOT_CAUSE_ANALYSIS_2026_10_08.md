# Root-Cause Analysis & Test Design: Low-Speed / Low-Rate Drive Accuracy (2026-10-08)

Continues `TUNING_LOG.md` (E0-E4a) and `TUNING_TEST_PLAN.md` (Phases 0/B/C/D/E). This document does not replace either — it adds three things they don't have yet: (1) a probability-ranked root-cause breakdown across domains, so testing isn't guesswork; (2) new isolated tests for causes the existing plan doesn't cover, found by re-reading the actual data; (3) a concrete logging/data schema.

Methodology is grounded in: FMEA/fishbone/fault-tree practice for separating causes across domains (electrical/mechanical/control/software); the standard breakaway + steady-state-sweep procedure for identifying Stribeck/stiction friction; the DOE-vs-one-factor-at-a-time tradeoff; and run-manifest/FAIR conventions for structuring experimental data (sources at the end). Domain-specific claims about *our* robot are grounded in a fresh re-read of `TUNING_LOG.md`, `GROUND_2026_10_02.md`, `BENCH_2026_09_28.md`, `F_firmware_characterization_report.md`, and `actuator_node.py` — cited inline.

## 1. The headline finding this analysis turned up

**The Stribeck/friction story is real but incomplete, and the data already shows exactly where it breaks.**

E1 identified true kS ≈ 0.40 V (both tracks, both directions) against an in-use 0.18 V — a 55% undersized feedforward. E2 applied the correction (kV 0.00211, kS 0.40/0.39). Result:

| | Before (kS 0.18) | After (kS 0.40/0.39) |
|---|---|---|
| **Reverse** crawl (0.05 m/s) | −11.2 / −8.4% | **−1.9 / −1.2%** — fixed |
| **Forward** crawl (0.05 m/s) | −32.6 / −30.5% | **−30 / −35%** — unchanged |

Correcting a *direction-symmetric* kS/kV fixed reverse crawl almost completely and did essentially nothing for forward crawl. That means the forward-crawl shortfall is **not** explained by an undersized static-friction term — something direction-specific is still missing, and no existing document names it. This is the single highest-value open question, not generic "friction is nonlinear."

> **UPDATE 2026-10-08, same evening — N1 ran, question resolved.** Ran the 0.05 m/s crawl in strictly alternating forward/reverse order (5 reps each, R0 reference gains reconfirmed live via `PR` before the run: kV 0.00211 res=0, kP 0.0004 res=0, kS 0.400/0.390) instead of blocked order. Result: L fwd −1.49% (±0.87%), L rev −4.35% (±0.29%), R fwd +2.27% (±0.70%), R rev +3.02% (±0.29%) — **all pass the ±10% crawl target**, and the forward/reverse split is gone. This confirms **A2a (test-order/warm-up confound)** as the explanation: every prior session ran all forward passes before any reverse pass, and the −30/−35% forward number was an artifact of that ordering, not a real direction-dependent friction or software asymmetry. A2b/A2c/A2d (ground-load, kinetic-friction, sign-handling hypotheses) are therefore **not needed** to explain this — do not spend test time on N2/N5's forward/reverse-symmetry angle. Raw data: `~/ground_tests/2026-10-08_1846_outdoor/runs/*_n1_{fwd,rev}_r{1..5}` on the Jetson. New finding from the same data: **L/R match is 3.8-7.7%, above the A4 target (1.5%)**, right track delivering more than left with the P-term active — feeds into N3 below.

**Second confirmation at 0.1 m/s (3 reps/direction, same alternating protocol):** L fwd −0.24% (±0.13%), L rev −2.16% (±0.54%), R fwd +2.89% (±0.42%), R rev +3.02% (±0.27%) — ALL PASS again, vs. the original blocked-order result at this speed (E2/E3a: forward −12/−19%, baseline −12.9/−11.5%). The fix isn't specific to 0.05 m/s. The R-over-L pattern also repeats at this speed (+3.1 to +5.3%), same direction and similar magnitude to the 0.05 m/s result — this looks like a real, repeatable track-level effect, not noise.

Second headline: spin at 0.1 rad/s produces **zero motion** (tracks fully stuck, confirmed `GROUND_2026_10_02.md:39-47`), worse than straight-line crawl at a comparable track speed. Turning has its own failure mode, not just a scaled-down version of straight-line friction.

## 2. Root-cause taxonomy (fishbone), with current evidence and likelihood

Likelihood is relative confidence given everything measured so far, not a formal probability — stated so it can be argued with.

### A. Nonlinear (Stribeck-shaped) static friction — CONFIRMED, but insufficient alone
- **For:** E1 found kS 55% undersized; correcting it fixed reverse crawl (−11%→−1.9%). Baseline-vs-corrected gap at 0.1 m/s also shrank (−12.9%→smaller after E3a's kP step).
- **Against as the whole story:** can't explain why forward crawl didn't move at all after the same correction (§1).
- **Likelihood: High** that this is one real contributor (already the basis of Phase B/C). **Not sufficient** by itself.

### A2. Forward-crawl-specific residual — RESOLVED 2026-10-08 (was: OPEN, highest priority)
**A2a confirmed by N1 (see the update box in §1): alternating-order crawl at R0 gains gave L fwd −1.49%, L rev −4.35%, R fwd +2.27%, R rev +3.02% — all within the ±10% target, gap gone.** The four hypotheses below are kept for the record; A2b/A2c/A2d did not need to be tested since A2a already fully explains the observation.

| Hypothesis | Reasoning | Likelihood |
|---|---|---|
| **A2a. Test-order / warm-up confound.** Every session so far ran forward passes before reverse passes. Breakaway friction is known to drop after initial motion (warm-up). If forward was always measured "cold" and reverse always "warm," this alone produces exactly this asymmetry. No existing test controls for run order. | Cheapest to rule in/out; no prior test guards against it | **Medium-High** |
| **A2b. Direction-dependent dynamic friction below the Stribeck transition**, from track/sprocket/tensioner geometry, not captured by a single (correctly symmetric) static kS. | E1's kS actually came out symmetric (0.398-0.406 across all 4 track/direction combos) — so if this is real, it's a *kinetic*, not static, asymmetry, which a static kS fundamentally cannot fix regardless of its value. | Medium |
| **A2c. Sign/command-path asymmetry interacting with the inverted SparkMAX.** One SPARK has "Motor Inverted" set so `L+ R+` means forward on both tracks (`CLAUDE.md`). If the inverted controller's internal sign handling (CAN-level kS arbFF, velocity PID sign convention) behaves even slightly asymmetrically near small setpoints, it would show up only as a forward/reverse split, only on that track. | Low-Medium, cheap to check | Medium-low |
| **A2d. Weight transfer during the breakaway accel ramp.** `actuator_params.yaml` slew caps are asymmetric by accel-vs-decel (0.3 vs 1.3 m/s²), not by direction — worth confirming the slew code doesn't conflate "decelerating" with "reverse" anywhere, since that would silently reintroduce a direction effect from a magnitude-only design. | Low (likely a non-issue, but free to check by code read) | Low |

**Test N1 (ground) — DONE 2026-10-08.** Alternating forward/reverse order (not blocked), 5 reps/direction at 0.05 m/s. Gap disappeared (§1 update box). A2a confirmed; Phase B's own protocol must use the same alternating order before its data is trusted (carried into §6 below).

**Test N2 (bench, tracks lifted) — SKIPPED, not needed.** Was meant to separate A2b/A2c from ground-load confounds, but N1 already fully explains the residual as a pure order effect, so there is nothing left for N2 to isolate on this question. (The bench right/left current-symmetry check is still needed — see N3 under category B — this is a different question.)

### B. Right-track mechanical asymmetry — CONFIRMED mechanical, severity is direction-dependent and bigger than assumed
- **For:** Bench, tracks **off the ground** (eliminates ground-load confound entirely), 5% duty: L≈183 RPM vs R≈165 RPM (~10%) (`BENCH_2026_09_28.md:50`). This is internal (bearing/gearbox/belt), not surface load. Stable since 2026-04-23. A right-motor bearing failed 2026-05-13 under high kP.
- **New finding, changes the picture:** one session (2026-05-26) measured **reverse-only asymmetry of 24.86%** (L 102%, R 79%) — more than double the "stable" 8-10% forward figure, and "not seen in prior testing because all prior testing was forward-only." Direction matters far more here than the existing docs assume.
- **Critical methodology gotcha, confirmed in `F_firmware_characterization_report.md` (H3):** closed-loop testing with heading-hold ON **masks** this asymmetry — heading-hold commands up to +24% extra to the right wheel to compensate, so any closed-loop test under-reports the true mechanical asymmetry. **Every isolation test for this must run heading-hold OFF, open-loop.** (The current Jetson test config already sets `heading_hold_deadband: 0.0`, which — per `actuator_node.py`'s `abs(w) < deadband` condition — effectively disables heading-hold already, since `abs(w) < 0.0` is never true. Good, but make this explicit in the test protocol so it's not accidentally reverted.)
- **Likelihood: High (confirmed real, internal).** Open question is whether it's symmetric enough to fix with a software correction table, or bad enough in reverse to need a physical fix (bearing/gearbox inspection or replacement).

**Test N3 (bench, extends the existing right-track check in `TUNING_TEST_PLAN.md` §"Right-track check"):** that check currently only specifies "drive both at the same voltage (2/4/6V ramp)" without saying which direction. Run it **explicitly in both directions**, tracks lifted, comparing current draw at equal voltage. If reverse shows a much bigger current/speed gap than forward (matching the 24.86% ground finding) → there's a direction-dependent internal fault (e.g., a one-way-loaded bearing or asymmetric gear mesh) that likely needs a physical fix, not just a bigger software correction. If both directions show the same ~10%, it's safe to treat as a simple symmetric internal friction offset and fold into the correction table.

### C. Control-architecture / gains
- **Spin-specific failure is its own axis, not scaled-down crawl.** 0.3 rad/s delivers only 60-70%; 0.1 rad/s: full stuck. This is worse than straight crawl at a comparable track speed, pointing at scrub/ICR torque load unique to turning (consistent with the Mandow correction already being a *kinematic*, not dynamic, fix — it doesn't add torque margin). **This needs its own speed-dependent correction map, separate from the straight-line one** — not just the existing A6 pass/fail spin check.
- **Untested, concrete, cheap:** SPARK param 97 `allowedClosedLoopError` (currently 0) — "below this error only feedforward acts" (`TUNING_LOG.md:23`). No document anywhere tests this against the crawl-speed error. If forward crawl's velocity error happens to sit inside a feedforward-only dead zone from this parameter while reverse's doesn't, that would directly explain part of §1's residual — speculative, but it's a single bench parameter sweep, no new tooling needed.
- **Electrical bus sag is real but not the baseline cause.** Full 26-run delivery campaign never dropped below 10.1V with no faults — bus sag was **not** present when the original crawl shortfall was measured (kP≤0.0002). Sag only shows up later, as a **ceiling on how far kP can be pushed**: kP 0.0006→9.0V, kP 0.0008→8.0V/52A/oscillating. Ruled out as cause of the Section 1 finding; kept as a hard constraint for Phase D.

**Test N4 (bench, tracks lifted):** sweep `allowedClosedLoopError` (e.g. 0, 10, 50 RPM) at a fixed crawl setpoint with the corrected kS/kV, holding everything else fixed. Read back the value via `PR` to confirm it actually took. See if forward crawl delivery changes.

**Test N6 (ground):** a spin-rate friction map analogous to Phase B's translational grid — 0.05, 0.1, 0.15, 0.2, 0.3, 0.45, 0.6 rad/s, both directions, 5 reps — feeding a *second* correction table (rotational), not folded into the straight-line one.

### D. Encoder/firmware quantization (Hall sensor)
- Hall sensors are inherently low-resolution (6 edges per pole-pair); velocity-from-position-differencing quantizes into discrete "steps" that a loop can amplify into oscillation — this is a real, literature-documented failure mode at low RPM, independent of friction.
- **Against this being the dominant cause here:** our measured crawl errors are explicitly "systematic, not noise" — tight bands (±0.1-0.5% at speed, ±2-4% at crawl, `GROUND_2026_10_02.md:35`). A quantization-driven effect shows up as *variance*, not a repeatable 30% mean bias. The band does widen 4-8x at crawl vs at speed, though, which quantization could partly explain.
- SPARK hall-filter lag (depth 3→2, 140-185ms→50ms) is already fixed and only affects transients, not steady-state — not implicated here.
- Command-path RPM rounding (`actuator_node.py:496`, `f'L{l_rpm:.0f}'`): 1 RPM of rounding at a ~150 RPM crawl command is 0.3-0.7% — **ruled out**, two orders of magnitude too small to explain 30%+ errors.
- **Likelihood: Low** as cause of the mean bias; **Medium** as a contributor to the crawl-speed noise band specifically (not the shortfall).

**Test N5 (bench, tracks lifted, nearly free — logging only, no new runs):** log the raw Hall-derived velocity feedback (STATUS_7/8, already decoded by the firmware) at a fixed crawl setpoint and look directly for discrete stepping vs. the smoothed estimate. This settles whether quantization artifacts are even present, independent of whether they explain the bias.

### E. Electrical — ruled out for the baseline problem, confirmed as a separate constraint
Covered under C. Bus never sagged during the campaign that found the original shortfall; sag is real but only becomes relevant once kP is pushed past ~0.0006 (Phase D territory).

### F. Command-path / software — ruled out / already guarded
- RPM rounding: ruled out (above).
- Native SPARK kS creep-at-zero (param 204, would add +kS even at a zero setpoint): **not an active cause** — the design deliberately avoids it by sending kS through the Teensy's arbFF path instead (1-RPM deadband, zero at zero), and `CHK` already asserts the native kS param is 0 at boot. This is a guarded regression risk, not an open item — flagging only so nobody "fixes" it by switching to the native parameter later without realizing why it was avoided.

### G. Test-methodology confounds (apply across all of the above)
- Heading-hold masking L/R asymmetry in closed-loop tests — covered under B, must be OFF for every isolation test.
- Forward-always-before-reverse ordering — covered under A2a.
- The at-speed forward/reverse gap is "**a constant in RPM**, not a percentage" (~20 RPM left, ~35 RPM right at every commanded speed, `TUNING_LOG.md:140`) and is explicitly attributed to "sidewalk slope and/or direction-dependent friction" — **unresolved**. Phase 0's 180° check (robot physically turned around, forward/reverse re-measured in each heading) is designed to separate slope from drivetrain for exactly this, and **has not been run yet**. This is a different question from A2 (A2 is crawl-specific and survived kS correction; this is a speed-independent RPM offset) but uses the same diagnostic.

## 3. Ranked summary — what's actually likely wrong

1. ~~**Forward-crawl residual (A2)**~~ — **RESOLVED 2026-10-08 by N1: pure test-order confound, gap gone under alternating order.**
2. **Right-track asymmetry, worse in reverse (B)** — confirmed mechanical, severity by direction not yet isolated (N3); N1 adds a new, related data point (L/R match 3.8-7.7%, fails A4) to fold in.
3. **Spin/turning is a separate failure axis (C)** — confirmed severe, needs its own map and correction table (N6), not a scaled copy of the straight-line one.
4. **`allowedClosedLoopError` interaction (C)** — untested, cheap, was a candidate explanation for #1 — now moot for that purpose since #1 is resolved, but still worth the cheap bench sweep on its own merits (Phase D gain-tuning relevance).
5. **Nonlinear static friction generally (A)** — confirmed real, already the basis of the existing Phase B/C. N1's clean ±10%-passing numbers (once order is controlled) suggest the *existing* R0 feedforward may already be closer to sufficient at crawl than assumed — re-run Phase B's full grid before concluding a correction table is even needed at every speed point.
6. **Hall quantization (D)** — confirmed present (N1's raw logs show discrete ~22 RPM delivered-speed steps at a 150 RPM command — about 15% per step), consistent with contributing to the noise band, not the mean bias. Still low confidence as cause of any remaining bias.
7. **Electrical sag, RPM rounding, filter lag, native kS creep (E/F)** — ruled out or already mitigated for this problem; kept only as guardrails for later phases.

## 4. Test plan additions (on top of `TUNING_TEST_PLAN.md`'s existing A1-A11/Phase B-E)

| ID | What | Isolates | Where | Robot time |
|---|---|---|---|---|
| N0 | Re-derive §1's numbers directly from `GROUND_2026_10_02.md`/`TUNING_LOG.md` by hand before spending any robot time, to rule out a transcription error producing a residual that isn't real | sanity check | desk, 0 min | none |
| N1 | **DONE.** Alternating-order forward/reverse crawl (0.05 m/s, 5 reps/direction) | A2a (warm-up/order confound) — **confirmed** | ground | ~15 min (actual) |
| N2 | **Skipped** — not needed, see A2 above | A2b/A2c vs ground-load | bench | — |
| N3 | Right-track hand-spin + equal-voltage current ramp, **explicitly both directions** (extends the existing right-track check) | B severity-by-direction | bench | ~15 min |
| N4 | `allowedClosedLoopError` sweep (0/10/50 RPM) at fixed crawl setpoint, tracks lifted | C (param 97) | bench | ~15 min |
| N5 | Log raw STATUS_7/8 Hall feedback at fixed crawl setpoint, tracks lifted | D (quantization) | bench | ~5 min (logging only) |
| N6 | Spin-rate friction map: 0.05-0.6 rad/s, both directions, 5 reps | C (turning is a separate axis) | ground | ~30 min |

**Order (respects isolation — bench before ground, cheapest/no-robot first):**
N0 (desk, done) → ~~N2~~ (skipped) + N5 + N4 (bench, lifted, no ground confound) → N3 (bench, right-track, both directions — raised in priority given N1's L/R finding) → Phase 0 session start (done 2026-10-08, `~/ground_tests/2026-10-08_1846_outdoor`) + the still-unrun 180° slope check → N1 (ground, alternating order — **done**, resolved) → Phase B (ground friction map, now run with the alternating-order protocol N1 validated) → N6 (ground spin map) → Phase C (fit straight-line **and** spin correction tables — scope may shrink given N1's clean crawl numbers, re-check against A1-A5 before building a full correction table) → Phase D (bounded integral, only if C leaves gaps) → Phase E (confirmation).

**Status as of 2026-10-08 evening:** N0 and N1 complete. Session is open and gains are live (R0 pushed and confirmed via `PR`). Next recommended step while the robot is already out: either N3 (bench, quick, resolves the right-track direction question) or go straight into Phase B's full speed grid using the now-validated alternating-order protocol — both are ground-ready right now, no further setup needed.

This is still mostly one-factor-at-a-time, matching `TUNING_TEST_PLAN.md`'s own principle #1 — but note DOE methodology flags a real gap in that approach: OFAT can't detect an interaction between factors, and that's exactly what happened here (kS/kV correction alone looked sufficient until direction was crossed with it). **N1, N2, and N4 are explicitly 2-factor checks (speed × direction, parameter × crawl) for that reason, not pure OFAT** — keep that pattern for any other factor pair with a plausible interaction (e.g., don't assume the spin correction table (N6) is independent of the right-track asymmetry (N3); fit N3 before N6 so the spin map isn't contaminated by an unresolved mechanical offset).

## 5. Logging / data structure

No established standard exists for robotics test-campaign data (confirmed by the research — FAIR principles and run-manifest conventions exist generically, but nothing robotics-specific is standardized), so this defines ours, applying the general principle "metadata is first-class, not an afterthought":

```
docs/drive_tuning_2026_09_28/sessions/<YYYYMMDD>_<surface>_<session_id>/
  manifest.json        # session-level: git commit, firmware version (FV read), SPARK param
                        # snapshot (PR read-back of kP/kI/kD/kV/kS_L/kS_R/filter depth+period),
                        # surface, operator, start bus voltage, ambient temp if known
  runs/
    <run_id>.json       # one per run — schema below
    <run_id>_raw.csv     # raw E-line / STATUS_7-8 log for that run
  journal.csv           # existing convention (drive_tuner.py already writes this) — append-only,
                         # human-browsable index into runs/
  RESULTS.md            # written after the session; references run_ids, not raw numbers inline
```

**run_id convention:** `<session_id>-<test_id>-<track>-<direction>-<value>-<repeat>`, e.g. `20261008a-N1-L-fwd-0.05-03`. Test IDs are the ones in §4 plus the existing A1-A11/Phase letters — one controlled vocabulary across both documents, not per-session ad hoc labels.

**Per-run manifest fields:** `run_id`, `session_id`, `test_id`, `timestamp_utc`, `git_commit`, `firmware_version`, `spark_params_snapshot` (object), `track`, `direction`, `commanded_value`, `repeat_index`, `random_order_position` (null if not applicable — explicit so N1-style tests are distinguishable from blocked-order ones at a glance), `bus_v_min`, `current_max_A`, `fault_flags`, `guard_tripped` (bool + reason), `raw_log_sha256`, `result_summary` (delivered value, error_pct), `operator_notes`.

Why a hash of the raw log and a version-tagged manifest: so a result can never be silently misattributed to the wrong firmware/config after a later change — this is the specific failure mode that caused the "is this from our newest setup?" confusion earlier in this project (Jetson vs. laptop config drift). Tying every run to `git_commit` + a snapshot of the actual SPARK read-back (not just "what the yaml says") closes that gap permanently, not just for this campaign.

**Statistics convention:** mean / 95% band / n computed from the manifests programmatically (this is `TUNING_TEST_PLAN.md`'s own T4 tool requirement — this schema just gives T4 a concrete file format to read instead of requiring it to parse `journal.csv` by convention alone).

## 6. What this changes about the existing plan, concretely
- Before running Phase B for real: insert N0, N1, N2 first. If N1 shows the forward-residual is a pure order artifact, Phase B's own grid (currently blocked by direction, not randomized) needs the same randomization fix before its output can be trusted as ground truth for the Phase C correction table.
- Phase C should produce **two** correction tables, not one: straight-line (as planned) and spin-rate (N6) — they are evidenced to be different failure modes (§1, §3).
- The right-track check in `TUNING_TEST_PLAN.md` gets one concrete edit: run it in both directions explicitly (N3), given the 24.86% reverse finding.
- Add `allowedClosedLoopError` (N4) to the tool/parameter list Phase 0 reads back and logs — it's currently invisible to the whole plan despite being a live SPARK parameter.

## 7. Full ground campaign, firmware finalization, and the actuator_node gain-push bug (2026-10-08, same evening)

### 7.1 Full straight-line grid: alternating order holds across the whole speed range
Beyond the two confirmations in §1's update box, the alternating-order protocol was run across the full Phase B speed grid (0.03–0.7 m/s, both directions, both tracks, n=3–5), all at R0 (kV 0.00211, kP 0.0004, kS 0.40/0.39):

**Result: ALL PASS against A1 (±2% at ≥0.3 m/s), A2 (±4% at 0.1–0.2 m/s), A3 (±10% at 0.05 m/s, never stuck), and A8 (stop rollback ≤5mm) — at every single point on the grid.** Worst case anywhere was R at 0.3 m/s reverse (+1.56%, still inside the 2% band). Bus never dropped below ~11.6 V; zero faults across 76 runs.

Two targets did **not** pass, and only below 0.4 m/s:
- **A4 (L/R match, ±1.5%):** right over-delivers left by 3–9% from 0.03–0.3 m/s, converging to 0.1–0.5% by 0.4 m/s and above.
- **A5 (fwd/rev match, ±1.5%):** off by ~2–3% below 0.4 m/s, fine above it.

**Conclusion: the planned Phase C speed-dependent correction table is very likely unnecessary for straight-line motion.** The existing R0 feedforward already meets spec across the whole range once tested without the order confound. The one real open item for straight-line driving is the low/mid-speed L/R and fwd/rev mismatch (which §7.3 below complicates further) — not a full correction table. **Spin/turning remains untested tonight** and is still the one confirmed-severe open problem (0.1 rad/s fully stuck, 0.3 rad/s 60–70%, from the Oct 2 data) — N6 is still the highest-value remaining test.

### 7.2 Firmware: verified and finalized as v2d
Asked directly whether the running firmware was current: circumstantial evidence (CHK auto-running at boot, v2d-only DIAG fields `iacc/sp8/atsp/slot/mt/blk/blkn/chk` present all night) said yes, but the literal boot banner couldn't be read without forcing a reset (the Teensy had been up since the Jetson's 16:11 boot). Resolved by reflashing deliberately rather than guessing:
1. Compiled `firmware/teensy_diff_drive_v2/` fresh on the Jetson (`arduino-cli compile --fqbn teensy:avr:teensy41`) — 69 KB code, clean, no warnings.
2. Flashed with `teensy_loader_cli --mcu=TEENSY41 -s -w` — soft reboot worked automatically, no physical button needed.
3. Read the boot banner directly off the serial port: **`# avros diff-drive bridge v2d ready (SPARK MAX FW 26.1.5)`** — confirmed.
4. SPARK encoder position (3218/3916 rotations) read back unchanged across the reflash, confirming the SPARKs themselves (separate hardware, CAN-attached) were undisturbed.
5. `CHK OK` both sides; R0 gains re-pushed and read back via `PR` matching exactly (kV 0.00211, kP 0.0004 both tracks).
6. A same-evening alternating-order retest at 0.05/0.1 m/s (`n1e_*` runs) — see §7.3.

**This firmware (byte-identical to commit `3a3aecc` on the never-merged `drive/fw26-motor-control` branch) is now the designated final v2d** — status updated in `PROTOCOL.md` and `firmware/README.md`. It was not committed to `main` as part of this session (still sitting uncommitted in the Jetson's working tree per the earlier branch-switch work); committing it is a separate decision.

### 7.3 Post-reflash parity check — and an honest complication
Re-ran the alternating-order test at 0.05/0.1 m/s post-reflash (`n1e_*`, n=3/point). **ALL PASS**, same bands as before reflashing. But: the L/R mismatch that was a consistent 3–8% throughout the pre-reflash campaign (§7.1) had shrunk to **-0.65% to +0.58%** — inside the A4 target.

The reflash is identical source code, so it should not by itself change behavior. The more likely explanation is that **the L/R mismatch is, like the original forward/reverse problem, a session-history/warm-up-dependent artifact rather than a fixed property** — by the time `n1e` ran, both tracks had been extensively exercised by the ~76-run grid that preceded it. This is *not* a confirmed causal claim — it's a second piece of evidence (alongside §2's A2a finding) that something about track history/warm-up state, not a fixed track-to-track difference, is driving the low-speed asymmetry numbers. **This makes N3 (bench, tracks lifted, isolated from ground-driving history) more important, not less** — it's the only test design that can separate "right track is mechanically different" from "whichever track worked harder recently reads differently," and tonight's ground data alone cannot resolve that.

**Reconciliation re-check (`n1f_*`, run immediately after, same conditions):** repeated the identical 0.05/0.1 m/s alternating test a second time to see whether the post-reflash L/R-match number was itself stable or just another one-off reading. Result: **-0.06% to +0.56%** — matching `n1e`'s -0.65% to +0.58% almost exactly, and clearly distinct from the pre-reflash single measurement's 3.8-7.7%. Two independent post-reflash measurements now agree with each other and disagree with the one pre-reflash measurement. That's evidence of a real, reproducible *state change* across the session (not measurement noise) — consistent with the warm-up/session-history theory, though which specific physical mechanism (track seating, bearing warm-up, temperature) caused the shift is still open and is exactly what N3's isolated bench test is designed to pin down.

### 7.4 The actuator_node gain-push bug: confirmed, root-caused precisely, and fixed
Earlier tonight this was mis-diagnosed as "`actuator_node` never pushes yaml gains at startup." Rereading `actuator_node.py` showed that's wrong — it does push them (`__init__` always ran the `KF/KP/KI/KD/KZ/KSL/KSR` writes). The real, more precise bug:

- The background serial-reader thread (which parses the Teensy's/SPARK's replies) didn't start until **after** the startup gain-push block. So during that push, nothing was reading the port.
- `_serial_write` is fire-and-forget: no wait, no check, no retry. The startup code logged `SparkMAX gains set: ...` **unconditionally**, regardless of whether the SPARK ever actually confirmed the write.
- `kS_left`/`kS_right` (Teensy-local state, no CAN round trip — see `PROTOCOL.md` "KS is not a parameter write") reliably took effect every time. `kFF`/`kP` (a full CAN `PARAMETER_WRITE` round trip to the SPARK) did not — tonight's own data proved it: after ~90 minutes of a normal webui session, the SPARK readback showed the *old* baseline (kV 0.0023, kP 0.0002), while `webui.log` showed the startup code claiming `kFF=0.00211 kP=0.0004` had been "set." The webui.log's own startup line even proves the correct value was *attempted* — it just was never confirmed, and nothing checked.

**Fix implemented in `actuator_node.py`** (deployed and verified on the Jetson, rebuilt via `colcon build --symlink-install --packages-select avros_control`):
1. Moved the startup gain-push to occur *after* the reader thread starts, so replies are actually read.
2. Added `_write_gain_verified()`: sends the `K<gain><val>` command, then waits (bounded, 0.8 s × up to 3 attempts) for the SPARK's own `PWR ... res=0` confirmation on **both** tracks before considering it set; retries on timeout/mismatch; logs an **ERROR** (not just INFO) naming exactly which gain failed if it's never confirmed.
3. Added `PWR` line parsing to the serial-reader thread (it previously only logged `OK ...` Teensy-side acks, which only confirm the Teensy *parsed* the line, not that the SPARK *applied* it).
4. Applied the same verified-write helper to the runtime `ros2 param set` path (`_on_param_change`), which had the identical unverified-write weakness.

**Verified fixed, twice:** first with a bare `ros2 run avros_control actuator_node` (log: `... — confirmed by PWR replies`, independently cross-checked via `drive_tuner.py cmd 'PR B 16' 'PR B 13'` matching). Then, since the bug only matters through the paths operators actually use, repeated through the **real launch file** — `ros2 launch avros_bringup webui.launch.py` — the exact command a normal session runs. Same result: `confirmed by PWR replies` in the launch log, and an independent `PR` readback afterward (via a separate process, after killing the launch) again showed **kV=0.00211, kP=0.0004 on both tracks**. The fix holds through the actual deployed launch path, not just a minimal `ros2 run`.

**Practical implication:** any `actuator_node`-driven session (webui, Nav2) before tonight's fix may have silently been running on stale/old gains with no indication in the logs. This is now closed for the startup path and the live-param-set path.

## Sources (methodology)
- [Comparing Common Root Cause Analysis Techniques](https://taproot.com/root-cause-comparing-common-rca-techniques/) — FMEA/fishbone/FTA/5-Whys comparison
- [Friction identification in mechatronic systems](https://staff-beta.najah.edu/media/sites/default/files/Friction_Identification_In_Mechatronic_Systems.pdf) — Stribeck/breakaway identification procedure
- [Identification and control of the motor-drive servo turntable with the switched friction model](https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/iet-epa.2019.0568) — Stribeck (medium/high speed) vs. LuGre (low speed) model choice
- [One-factor-at-a-time method](https://en.wikipedia.org/wiki/One-factor-at-a-time_method) and [Factorial experiment](https://en.wikipedia.org/wiki/Factorial_experiment) — DOE vs. OFAT, interaction detection
- [A framework for FAIR robotic datasets](https://www.nature.com/articles/s41597-023-02495-3) — FAIR principles applied to robotics field data; confirms no robotics-specific metadata standard exists
- [The FAIR Guiding Principles for scientific data management and stewardship](https://www.nature.com/articles/sdata201618)
- Low-frequency noise with Hall encoders — [ODrive forum](https://discourse.odriverobotics.com/t/low-frequency-noise-with-hall-encoders-odrive-3-6/10464), [motor oscillations at low RPM](https://discourse.odriverobotics.com/t/motor-oscilations-at-low-rpm-encoder-questions/5409) — Hall-sensor quantization mechanism
