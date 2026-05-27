# Yaw Diagnostic Verdict — 2026-05-27 (DRAFT — pending agent reports)

## TL;DR
**Final decision:** (NO_FIX_NEEDED / FIX_1 / FIX_1+2 / FIX_4 / HOLD / NEW_ROOT_CAUSE)
**Confidence:** (HIGH / MEDIUM / LOW)
**Headline:** (2–3 sentence summary)

## Multi-agent findings

| Agent | Verdict | Headline finding |
|---|---|---|
| A — Sensor Health | (pending) | |
| C — EKF Behavior | (pending) | |
| D — Actuator Fidelity | (pending) | |
| E — Map-EKF / GPS | (pending) | |

## Evidence chain

(Populate after agents return.)

## Cross-agent reconciliation

(If contradictions, explain how resolved.)

## Issue #13 verdict

Based on the multi-agent analysis:

- **EKF tracking under motion** (Agent C headline): ___
- **Recommended fix from staged patches**: ___
- **Issue #13 status**: (KEEP_OPEN / CLOSE_NO_FIX / CLOSE_AS_PARTIAL / RESOLVE_OTHERWISE)

## New findings to file as follow-up issues

| Finding | Recommended issue title | Owner |
|---|---|---|
| (Agent D's L motor weakness) | "L motor delivers 7% less than R in forward direction" | TBD |
| (Agent E lever-arm if detected) | "GPS antenna lever arm needs measurement" | TBD |
| (M0 drift regression) | "Map-EKF M0 drift 1.56 cm/s — 6× M4 baseline regression" | TBD |
| (Run 1 Xsens incident if confirmed) | "Xsens stuck-bias under motion — needs reliable detection/recovery" | TBD |

## What to commit / what to leave

| Item | Action |
|---|---|
| navsat.yaml SE Michigan datum | Already committed locally (f71d8dc), not pushed |
| yaw_diag.launch.py | Should be committed (clean test-mode launch file) |
| New scripts (preflight, monitors, analyzers) | Should be committed |
| Pre-staged Fix 1/2/4 patches | Keep as docs, do not apply |
| Session docs in docs/yaw_diag_session_2026_05_27/ | Commit when verdict finalized |

## Re-test plan (if a fix applied)

Per P3.2: re-run Test A only (single forward 3-5m drive) on the clean stack, compare yaw drift to pre-fix. If clean, proceed to full M1+M2+M3.

In this session's case: probably no patch will be applied. The verdict is likely "investigate chassis-side asymmetry" + "close #13 as wrong hypothesis."
