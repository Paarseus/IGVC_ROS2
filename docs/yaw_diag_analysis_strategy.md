# Yaw Diagnostic — Analysis Strategy

How we go from "bag on disk" to "fix decision" using a multi-agent parallel analysis. The execution strategy (field session) is in `session_2026_05_27_yaw_diag_field_plan.md`. This doc is the post-bag plan.

---

## Goals (three nested layers)

| Layer | Question | Drives |
|---|---|---|
| **Primary** | Which yaw source is biased — IMU, wheel, or both? | Fix decision (apply Fix 1/2/4 or nothing) |
| **Secondary** | Phase 2 motion data — inner-PID delivery, EKF integrity in motion, GPS noise floor under motion | Advances issue #12 Phase 2 → Phase 3 |
| **Opportunistic** | Are there other bugs hiding in this bag we should surface now? | New issues, CLAUDE.md updates, memory entries |

This one bag answers all three layers. Cost is in the analysis, not the data collection.

---

## Pipeline overview

```
   bag → extract → preflight ─→ tf audit ─┐
                                          ↓
                          ┌───────────────┴───────────────┐
                          │  5 agents in parallel         │
                          │  A   B   C   D   E            │
                          └───────────────┬───────────────┘
                                          ↓
                                synthesis (me)
                                          ↓
                       ┌──────────────────┴──────────────────┐
                       │ verdict clear?                       │
                       │  yes → apply patch                   │
                       │  no  → deep dive (Agent F or G)      │
                       └──────────────────────────────────────┘
                                          ↓
                                re-test → re-analyze (same agents)
                                          ↓
                                document + close #13
```

---

## Stage I — Bag intake (mechanical, no agents)

Each step exits non-zero on failure. Don't proceed to Stage II if any of these fail.

| Step | Tool | What it does |
|---|---|---|
| 1 | `python3 scripts/extract_bag.py <bag> <csv_dir>` | Bag → per-topic CSVs |
| 2 | `python3 scripts/bag_preflight.py <csv_dir>` | Verifies all required topics present, gaps in timestamps < 100 ms, expected message counts |
| 3 | `python3 scripts/tf_audit.py <csv_dir>` | Walks TF tree, reports frame_id/child_frame_id for each odometry source. Catches frame-mismatch silent-corruption (e.g., ZED VIO child_frame pointing at camera_link not base_link, which silently breaks EKF lever-arm) |

If preflight reports missing topics, **stop** — the agent verdicts will be unreliable. Most likely cause: ZED VIO not launched (`enable_zed_front:=true` missing) → Test B has no truth source.

If tf_audit reports a frame surprise (e.g., a topic claims a frame the URDF doesn't have), that's a finding before we even look at numbers.

---

## Stage II — Multi-agent parallel analysis

Five agents, all independent, all read from the same CSV dir. Each produces a markdown report with a structured verdict line at the top.

### Common verdict taxonomy

Every agent emits one of:
- `PASS` — data looks healthy, no action
- `MARGINAL` — within tolerance but worth noting; trend to watch
- `FAIL` — data violates threshold; action required
- `ANOMALY` — unexpected pattern; flag for cross-correlation hunt

Agents must cite specific timestamps + values for every claim. No hand-waving — every assertion is grounded in a CSV row.

---

### Agent A — Sensor health

**Question:** is each raw sensor publishing clean data on its own merits, independent of any fusion?

**Input CSVs:**
- `imu_data.csv` — Xsens MTi-680G
- `wheel_odom.csv` — actuator_node Mandow-corrected
- `zed_front_zed_node_odom.csv` — ZED VIO (if present)
- `zed_front_zed_node_imu_data.csv` — ZED's internal IMU (if recorded; per our recommendation)
- `gnss.csv` — Xsens GPS
- `avros_wheel_debug.csv` — raw motor telemetry

**Analyses:**

1. **Stationary periods** (M0 plus any inter-test pauses):
   - IMU gyro bias: mean(`/imu/data.wz`) — expect \|μ\| < 0.05 °/s
   - IMU gyro noise: σ(`/imu/data.wz`) — expect < 0.15 °/s
   - Wheel encoder noise: wheel_odom.wz should be exactly 0 (no encoder noise leak to vyaw); if nonzero, something is mis-publishing
   - ZED VIO stability: yaw drift while stationary
   - GPS noise floor: σ of `gnss.lat`, `gnss.lon` over the same window

2. **Publishing health:**
   - Topic-rate consistency (gaps > 100 ms count as drops)
   - Timestamp monotonicity (any out-of-order messages?)
   - Coverage of each test window (no sensor went silent mid-test?)

3. **Sensor-internal consistency:**
   - IMU quaternion-derived yaw vs IMU gyro-integrated yaw (within a short stationary window) — should agree
   - ZED VIO has internal IMU (if recorded) — compare to Xsens. Independent confirmation of any IMU bias.

**Output report:** `agent_a_sensor_health.md` with verdict per sensor + an overall PASS/MARGINAL/FAIL.

**Why it matters first:** If a sensor is fundamentally broken (e.g., Xsens stuck in mag-cal mode), no covariance tune will help. Agent A is the gate on whether B/C/D/E's verdicts are even meaningful.

---

### Agent B — Yaw forensics (THE #13 QUESTION)

**Question:** which yaw source is biased? IMU, wheel, both?

**Input CSVs:** all of the above plus `odometry_filtered.csv`, `cmd_vel.csv`

**Tools to run (already built):**
1. `python3 scripts/analyze_M1.py <csv_dir>` — Test A verdict on straight drive
2. `python3 scripts/analyze_M2.py <csv_dir>` — Test B verdict on rotation
3. `python3 scripts/analyze_M3.py <csv_dir>` — Test C closing error

**Beyond the scripts, the agent should additionally compute:**

1. **Within-window IMU self-consistency**: for each test window, IMU quaternion-derived Δyaw vs IMU gyro-integrated Δyaw should differ by < 0.5°. If they diverge, the Xsens internal filter is producing inconsistent orientation vs angular_velocity → Xsens filter quality issue (not just bias).

2. **Wheel-debug.yaw vs /odometry/filtered.yaw**: both should be tied (heading-hold consumes EKF yaw; wheel_debug echoes the input). If they differ, the actuator_node is consuming a stale or different yaw than what EKF publishes → architectural issue.

3. **Time-series cross-correlation**: lag between IMU vyaw spikes and /wheel_odom.wz spikes. If wheel lags IMU by ~50 ms consistently, that's just publish-rate alignment. If lag varies, something's wrong.

4. **Four-way consensus (with ZED IMU if recorded)**:

| Sources | If all 4 agree | If 3 agree | If 2v2 |
|---|---|---|---|
| Xsens IMU, ZED VIO, ZED IMU, wheel | strong consensus = no fix | the odd one is biased | the wheels-vs-IMUs split tells us EXACTLY where the bias lives |

ZED IMU is a *huge* differentiator if recorded. Xsens biased AND ZED IMU clean = magnetic interference (Fix 4). Both biased = environmental (motion shake → both gyros affected → need a different fix).

**Output report:** `agent_b_yaw_forensics.md` with the consolidated Fix decision + per-test verdicts.

**This is the load-bearing agent. The fix decision flows from here.**

---

### Agent C — EKF behavior audit

**Question:** did the EKF behave as configured? Will the proposed fix actually do what we think?

**Input CSVs:**
- `odometry_filtered.csv` (local EKF output)
- `odometry_global.csv` (map EKF output)
- `odometry_gps.csv` (navsat output, EKF input)
- `imu_data.csv`, `wheel_odom.csv` (EKF inputs)
- `tf.csv` (map→odom + odom→base_link transforms — EKF outputs)
- Reference: `src/avros_bringup/config/ekf.yaml` (the configuration claimed to be in effect)

**Analyses:**

1. **Did rejection gates fire?** `imu0_pose_rejection_threshold: 5.0` + `imu0_twist_rejection_threshold: 5.0` are configured. If they fired (because of the alleged 14.7° IMU disagreement), we should see periods where EKF yaw rate suddenly disconnects from /imu/data.wz. Look for windows where `|/odometry/filtered.wz - /imu/data.wz| > 5σ` of normal disagreement. If many such rejections, the EKF is already gating IMU and the fix is "raise the rejection threshold," not Fix 1.

2. **State covariance evolution** (if we can extract `/diagnostics` from r_l): does P[5,5] (yaw) actually stay small as #13 hypothesizes? If we didn't record /diagnostics, infer from output behavior — does /odometry/filtered.yaw_rate track wheel or IMU more closely under known-truth motion?

3. **Map→odom drift in motion**: compare M4 stationary baseline (0.26 cm/s) to in-motion drift. Should stay bounded (< 5 cm/s under M1/M2/M3). If it explodes during motion, the map EKF tune from 2026-05-19 was stationary-overfit — separate issue.

4. **wheel_odom contribution analysis**: per the local EKF config, wheel_odom contributes vx + vyaw (with covariance 0.0001). Plot the EKF state's vyaw vs the two inputs (`/imu/data.wz` and `/wheel_odom.wz`). The output's correlation coefficient with each input tells us which input is winning — directly validates or refutes #13's "wheel wins fusion" claim. **This is the most important plot in the whole session.**

5. **Counterfactual prep**: extract the EKF input streams to standalone CSVs in a format that could be replayed offline (Agent G work).

**Output report:** `agent_c_ekf_behavior.md`.

---

### Agent D — Actuator → chassis fidelity

**Question:** does the chassis deliver what MPPI / cmd_vel asks for? Inner-PID delivery? Mandow correction working?

**Input CSVs:**
- `cmd_vel.csv` (MPPI output)
- `avros_actuator_command.csv` (also an input to actuator_node)
- `avros_actuator_state.csv` (actuator_node echo)
- `avros_wheel_debug.csv` (16-field motor telemetry)
- `wheel_odom.csv` (kinematic-derived chassis velocity)

**Analyses:**

1. **Inner-PID delivery ratio** per wheel:
   - For windows where `wheel_debug.L_cmd_rpm > 200`: ratio = `L_meas_rpm / L_cmd_rpm`. Expect 0.93–0.96 per the 2026-05-18 retune.
   - Same for right wheel.
   - Asymmetry: `R_meas / L_meas` for symmetric commands.

2. **Chassis-level delivery ratio**:
   - Plot `cmd_vel.linear.x` vs `/wheel_odom.vx`. Slope should be ~1.0.
   - Plot `cmd_vel.angular.z` vs `/wheel_odom.wz`. Slope should be ~1.0 (with Mandow correction working) or ~0.85 (without).

3. **Slew rate engagement**: does `wheel_debug.v_slewed` lag `v_target` during velocity step changes? Confirms actuator_node's slew limits are active.

4. **Heading-hold engagement**: does `wheel_debug.heading_locked = 1` during straight drives (M1)? Does the post-IMU correction `w_after_imu` differ from `w_slewed` during straight drives? Quantifies heading-hold's contribution.

5. **Per-wheel asymmetry on straight drives** (the wheel-bias hypothesis):
   - If wheels are biased (Test A → "wheels biased" verdict), the cause should show up here: `L_meas - R_meas` non-zero during forward drive → asymmetric friction or PID gain mismatch.
   - CLAUDE.md notes 8% higher friction on right track. Confirm or refute on this surface.

**Output report:** `agent_d_actuator_fidelity.md`.

**Why it matters for #13:** if wheels are biased (Fix 1+2 path), Agent D should localize WHY — is it Mandow undercorrection (wrong α for grass), per-wheel friction asymmetry, or inner-PID slip? The fix is different in each case.

---

### Agent E — Map-EKF / GPS integrity

**Question:** how does the map EKF + GPS pipeline perform under motion? Does the M4 stationary baseline hold?

**Input CSVs:**
- `odometry_global.csv` (map EKF output)
- `odometry_gps.csv` (navsat_transform output)
- `gnss.csv` (raw GPS)
- `tf.csv` (map→odom transform)

**Analyses:**

1. **Map→odom drift rate**:
   - Stationary (M0): expect ≤ 0.26 cm/s (M4 baseline). If much worse, EKF map tune from 2026-05-19 has degraded.
   - In motion (M1/M2/M3): expect ≤ 5 cm/s. If unbounded growth, map EKF is not constraining odom EKF properly.

2. **GPS noise floor**:
   - σ(`gnss.lat`, `gnss.lon`) in M0 stationary — expect < 0.5 m peak-to-peak per M4.
   - Under M1 motion — does noise increase with motion (vibration, multipath)?

3. **GPS fix quality**:
   - `gnss.status.status` distribution — should be 0 (SBAS) or higher.
   - If consistently -1 (no fix), navsat_transform output is unreliable → /odometry/gps is garbage → map EKF has no GPS anchor.

4. **`/odometry/gps` rate**:
   - Should match GPS rate (~4 Hz). If lower, navsat_transform is dropping fixes (often due to IMU yaw quality — feeds back to #13).

5. **navsat lever-arm sanity**: the GPS antenna offset is `[0,0,0]` per `xsens.yaml`. Compare /odometry/gps vs /odometry/filtered xy during M1. Any consistent lateral bias = lever-arm needs measurement (separate issue from #13).

**Output report:** `agent_e_map_ekf_gps.md`.

---

## Stage III — On-demand deep dives

Only run if Stage II surfaces ambiguity.

### Agent F — Cross-correlation hunt

**Trigger:** any agent emits `ANOMALY`, OR Agent B verdict is `MARGINAL`, OR the agents contradict each other.

**Question:** is there a hidden coupling explaining the anomaly?

**Analyses:**
- IMU bias vs motor current proxy (`cmd_vel.linear.x`, `wheel_debug.L_meas_rpm + R_meas_rpm`)
- IMU bias vs GPS reception (`gnss.status.status`)
- IMU bias vs ZED VIO frame interval (proxy for compute load)
- EKF yaw vs map→odom jumps
- Any time-series correlation > 0.5 magnitude gets reported

**Output report:** `agent_f_correlations.md`.

### Agent G — Counterfactual EKF replay

**Trigger:** Agent B verdict is `MARGINAL` between Fix 1 alone vs Fix 1+2.

**Question:** what would the EKF have output if Fix 1 (or 1+2) had been applied?

**Approach:** spin up an offline `ekf_node` with a modified `ekf.yaml`, feed it the bag's `/imu/data` + `/wheel_odom` topics, compare its `/odometry/filtered` output to the as-recorded output.

**Cost:** ~30 min setup. Defer until needed.

**Output report:** `agent_g_ekf_replay.md`.

---

## Stage IV — Synthesis (me)

After agents finish, I:

1. Read all 5 (or 6/7 if F/G ran) reports.
2. Reconcile contradictions. Most likely contradiction: Agent A says "IMU clean" but Agent B says "IMU biased." Means the bias is motion-only (mag interference) — that's information.
3. Write a one-page consolidated verdict — `docs/yaw_diag_verdict_2026_05_27.md` — with:
   - Final fix decision (one of: nothing / Fix 1 / Fix 1+2 / Fix 4 / hold)
   - Confidence level (high / medium / low)
   - Key evidence cited from each agent
   - Any unrelated findings worth filing as new issues
4. Append to `docs/yaw_diag_session_LOG.md`.

If confidence is LOW, trigger Agent G replay before applying anything.

---

## Stage V — Apply + re-test

1. `git apply docs/yaw_diag_patches/fixN_*.patch`
2. If patch touches actuator_node: `colcon build --packages-select avros_control && source install/setup.bash` on Jetson
3. User re-runs Test A only (5 min) — record second bag
4. Re-run Stages I-IV on the re-test bag — same agents, fresh reports
5. **Compare pre-fix vs post-fix verdicts**:
   - Did Agent B's M1 verdict change from "WHEELS BIASED" → "BOTH CLEAN"? Fix worked.
   - Did anything new break? (Most likely: Agent C reports the EKF yaw became more noisy — expected, acceptable if M3 closing error improves.)
6. If fix worked: proceed to M2 + M3 re-test. If not: bisect — apply the OTHER fix in the same category, repeat.

---

## Stage VI — Documentation + close-out

1. Append §10 "EKF yaw weighting follow-up" to `docs/skid_steer_kinematics_findings_2026_05_18.md` with:
   - The pre-fix data (M1/M2/M3 numbers)
   - The chosen fix + reasoning
   - The post-fix data
   - Lessons learned
2. Add a row to CLAUDE.md Known Issues table.
3. Close #13 with the verdict doc as the summary.
4. Comment on #12 — Phase 2 motion data is collected; Phase 3 unblocked.
5. If any new issues were filed (e.g., from Agent E's lever-arm check or Agent D's friction asymmetry finding), link them to #13.
6. Update memory if anything generalizable came out (e.g., "ZED IMU is a useful 4th independent yaw source for IGVC_ROS2 — record it in every diagnostic session").

---

## Agent invocation prompts (templates)

When the bag arrives, I'll spawn the 5 agents using these prompts. They're saved here so we can re-run them on any future yaw-diagnostic bag.

### Agent A prompt template

```
Subagent type: general-purpose
Description: Sensor health audit for yaw-diag bag

Prompt: Audit per-sensor data quality in <csv_dir> for the IGVC yaw-diag
session. Inputs: imu_data.csv, wheel_odom.csv, zed_front_zed_node_odom.csv
(if present), zed_front_zed_node_imu_data.csv (if present), gnss.csv,
avros_wheel_debug.csv.

For each sensor: compute mean + σ of relevant fields over the M0
stationary window (use cmd_vel.csv to find when |vx| + |wz| < 0.05 for
sustained periods). Check publishing rates against expected Hz (IMU
100, wheel 20, ZED 15, GPS 4, wheel_debug 50). Flag dropped messages
(gaps > 100ms).

Output: agent_a_sensor_health.md in <csv_dir>/reports/, with verdict
line at top (PASS/MARGINAL/FAIL/ANOMALY), per-sensor table, and any
anomalies with specific timestamp citations.

Reference numbers: docs/yaw_diag_decision_thresholds.md M0 table.
```

### Agent B prompt template

```
Subagent type: general-purpose
Description: Yaw forensics + Fix decision

Prompt: Run the three pre-built analyzers on <csv_dir>:
  python3 scripts/analyze_M1.py <csv_dir>
  python3 scripts/analyze_M2.py <csv_dir>
  python3 scripts/analyze_M3.py <csv_dir>

Capture their full output verbatim. Then do additional checks: (1) IMU
quaternion vs gyro-integrated yaw self-consistency within each test
window, (2) wheel_debug.yaw vs odometry_filtered.yaw correspondence,
(3) if ZED IMU CSV present, add it as a 4th truth source and resolve
4-way consensus (see Stage II of strategy doc).

Output: agent_b_yaw_forensics.md with verdict (one of: NO_FIX_NEEDED /
APPLY_FIX_1 / APPLY_FIX_1_PLUS_2 / APPLY_FIX_4 / HOLD_INSUFFICIENT_DATA),
confidence (HIGH/MEDIUM/LOW), and the cross-test evidence chain.
```

### Agent C prompt template

```
Subagent type: general-purpose
Description: EKF behavior audit

Prompt: Analyze EKF behavior in <csv_dir>. Compare /odometry/filtered
state evolution to its inputs (/imu/data + /wheel_odom). Specifically
quantify: which input drives the EKF's vyaw output more strongly
(correlation coefficient per test window)? Did rejection gates appear
to fire (large gaps between input and output)? Was map→odom drift
stationary-overfit (compare M0 to M1/M2/M3 windows)?

Reference: src/avros_bringup/config/ekf.yaml for the configured
covariance values. Cite ekf.yaml line numbers when explaining behavior.

Output: agent_c_ekf_behavior.md with verdict + the "which input wins
fusion?" answer as the headline.
```

### Agent D prompt template

```
Subagent type: general-purpose
Description: Actuator → chassis fidelity

Prompt: Quantify command-to-delivery fidelity in <csv_dir>. Inputs:
cmd_vel.csv, avros_actuator_*.csv, avros_wheel_debug.csv, wheel_odom.csv.

Compute: per-wheel PID delivery ratio (L_meas/L_cmd, R_meas/R_cmd) for
windows where commands > 200 rpm. Chassis-level linear delivery
(wheel_odom.vx / cmd_vel.linear.x slope). Angular delivery
(wheel_odom.wz / cmd_vel.angular.z slope) — compare to 1.0 (Mandow on)
and 0.85 (Mandow off) benchmarks. Per-wheel asymmetry on M1 straight
drive (L_meas - R_meas under symmetric commands).

Output: agent_d_actuator_fidelity.md. If wheel asymmetry exceeds 5%
on the straight drive, headline that as the wheel-bias root cause for
Agent B's verdict.
```

### Agent E prompt template

```
Subagent type: general-purpose
Description: Map-EKF / GPS integrity

Prompt: Audit map EKF + GPS pipeline in <csv_dir>. Compute: map→odom
drift rate during M0 (compare to M4 baseline 0.26 cm/s) and during
each motion test. GPS xy noise floor (σ over stationary window).
GPS fix-status distribution (gnss.status.status). /odometry/gps
publishing rate vs gnss rate (drops = navsat_transform issues).

Reference: docs/session_2026_05_19_mppi_planner_costmap_gps.md and
issue #12 M4 baseline numbers.

Output: agent_e_map_ekf_gps.md. If any drift exceeds 2× the M4
baseline, flag as REGRESSION since 2026-05-26.
```

---

## What we deliberately do NOT do in v1

- **Statistical significance testing** — drift gaps in #13 are 10°+; tested via inspection, not p-values
- **Pre-vs-post bag diff script** — manually compare two sets of verdicts at first
- **Online EKF tuning** — out of scope; covariance values are the lever
- **ML-based bias detection** — overkill for this
- **All-frame TF history analysis** — single tf_audit.py check covers it

Add these later only if a v1 pass reveals they'd have helped.
