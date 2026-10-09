# C4 — Controller–vehicle interface: claim verification

| | |
|---|---|
| **Topic** | C4 — Controller–vehicle interface (`README.md` as of 2026-09-27 gap-fill) |
| **Date** | 2026-09-28 |
| **Reviewer** | independent — claims (STANDARDS.md §5, step 4) |
| **Method** | Every PDF converted with `pdftotext -layout` and split on form-feeds; each claim checked on the cited page (PDF page unless the README states otherwise: S05 book pages = PDF − 50; S44 journal pages = PDF + 614; S45 article pages = PDF − 1; S50 journal pages = PDF + 2179). McKinnon Fig. 6 and Pannocchia Fig. 1 were rendered and read visually. Code, YAML and Markdown sources read at the cited function / key / section / comment timestamp. Sources themselves are not graded here (see `SOURCE_AUDIT.md`). |

## Counts

| Item | Count |
|---|---|
| Claims checked (Summary, Foundational refs, Findings §1–11, Recommended practice, Key numbers, How it is tested, Common mistakes, Disagreements, cited Open questions) | **245** |
| Verified | **229** |
| Partly supported | **16** |
| Not supported | **0** |
| Foundational rows not graded (source not downloaded, no content claims: S02, S09, S10) | 3 |

## Claims

### Summary

| # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|
| Σ1 | Command is only the mean of what the vehicle receives; noise "enters through the controls"; blend by cost; send first, shift rest | S01 pp. 2, 8 | Partly supported | p. 2: "direct control over the mean ut … commanded input has to pass through a lower level of control"; p. 8 Alg. 1 send u0, shift. The quoted phrase is not on pp. 2/8; nearest wording is p. 9 "the noise enters the system through the control input" | Drop the quotation marks or cite p. 9 for the phrase |
| Σ2 | MPPI failed where learned model was wrong (predicted over-steer, got under-steer); Autoware late turn-in "often due to incorrect delay time and time constant" | S01 pp. 17–18; S38 "Other tips" | Verified | S01 p. 17 "incorrectly predicts over-steer when in fact the vehicle under-steers"; S38 "If the onset of steering in curves is late, it's often due to incorrect delay time and time constant in the steering model" | — |
| Σ3 | Powertrains "commonly modelled" as delay + first-order lag; 0.81 s / 0.16 s; "instrumental in removing large prediction errors" | S17 pp. 20–21 | Partly supported | p. 20 "the response of many powertrains can be modeled adequately by a time delay and a first-order transient response"; same page: "Most WMR motion models in related work omit powertrain dynamics"; p. 21 "time constant = 0.81, delay = 0.16 sec", quote verbatim | Replace "commonly modelled" with "can often be modelled adequately" |
| Σ4 | Delay compensation by forward prediction / actuator state / replay; hydraulic-steered vehicle, 600 ms, oscillated vs "visibly better" | S16 pp. 1–2; S15 p. 14; S30 "Per-Axis Delay Compensation" | Partly supported | S16 p. 1 shift initial state, first-order actuator ODE; S15 p. 14 "simulated forward for this time interval"; S30: "vehicle with 600 ms steering delay … tracking is visibly better" — S30 does not say that vehicle is hydraulic (only "platforms with hydraulic steering or any other source of lag"); S37 l. 38 names it "Four-wheel steered vehicle with hydraulic steering" | Add S37 to the citation for the hydraulic-vehicle detail |
| Σ5 | Heavy skid-steer rotated more per command on snow; traction estimated online or model error learned | S20 p. 6; S12 pp. 5–6; S11 pp. 1, 13 | Verified | S20 p. 6 "over 30◦, the curve for resulting angular displacement grows faster … on snow"; S12 p. 6 µ, κ estimated; S11 p. 1 learned disturbance model | — |
| Σ6 | Constant gain error leaves offset unless estimated; integrating disturbance; ×0.5 turn rate on 900 kg robot "adapts quickly"; L1 inner loop survived 40% power loss | S05 pp. 52–53; S50 pp. 2186–2187; S51 p. 5 | Verified | S05 p. 52 Lemma 1.10; S50 p. 2186 "multiplying the turn rate commands by 0.5 … adapts quickly"; S51 p. 5 case 5 "reduction in motor thrust control power by 40%", Table II | — |
| Σ7 | Humble seeds rollouts from odometry; later: accel limits, forward prediction, open_loop, per-axis delay | S26 `updateInitialStateVelocities`; S28 `prepare`; S29 | Verified | S26 l. 255–256; S28 l. 309–332; S29 `open_loop`, `model_delay_vx` | — |

### Foundational references

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| F1 | S49 first defined MPPI (path-integral derivation, IS with changed mean and variance, GPU sampling) | S49 | Verified | p. 1 "enables for both the mean and variance of the sampling"; p. 6 "sample a large number of trajectories in real-time" | — |
| F2 | S01 full IT-MPC theory; 100+ km real-vehicle results | S01 | Verified | p. 14 "over 100 kilometers of driving data" | — |
| F3 | S02 not downloaded | — | not graded | no content claim | — |
| F4 | S03 MPPI with learned NN model from driving data | S03 | Verified | p. 6 "bootstrap the neural network with 30 minutes" | — |
| F5 | S04 main reference on MPPI robustness (Tube-MPPI, RMPPI) | S04 | Verified | p. 1 abstract | — |
| F6 | S05 standard MPC textbook (offset-free MPC, disturbance models) | S05 | Verified | Sec. 1.5 | — |
| F7 | S06 standard description of PP and lookahead | S06 | Verified | p. 14 | — |
| F8 | S07 Nav2 reference tracker paper; states its assumptions | S07 | Verified | pp. 3–4 | — |
| F9 | S08 "Most-cited survey comparing vehicle models and trackers" | S08 | Partly supported | Survey content confirmed (Table II p. 18); "most-cited" is an uncited bibliometric claim not supported by any source file | Drop "Most-cited" or cite a citation count |
| F10 | S09 not downloaded | — | not graded | no content claim | — |
| F11 | S10 not downloaded; covered via S19 | — | not graded | no content claim | — |
| F12 | S11 field MPC learning model error on skid-steer robots | S11 | Verified | p. 2 "50 to 600 kg with both skid and Ackermann steering" (includes skid-steer) | — (see 8.8) |
| F13 | S12 tracked-robot MPC with online traction estimation in fields | S12 | Verified | p. 1, p. 6 µ, κ | — |

### §1 MPPI formulation

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 1.1 | Dynamics "affine in control and subject to an affine brownian disturbance" | S49 p. 2 | Verified | p. 2 exact | — |
| 1.2 | Noise limited to directly actuated states; fails with "known strong disturbances on indirectly actuated state variables or … only partially known" | S49 p. 2 | Verified | p. 2 exact | — |
| 1.3 | Special case for all experiments: noise as "random change in the control input" | S49 p. 5 | Verified | p. 5 "a special case which we use for all of our experiments … a random change in the control input" | — |
| 1.4 | IS changes mean and variance; natural variance "typically too low" | S49 pp. 3, 8 | Verified | p. 8 "the natural variance of the system is typically too low to achieve good performance" | — |
| 1.5 | Needs "rapid convergence…", "ability to sample a large number of trajectories in real-time"; warm start | S49 p. 6 | Verified | p. 6 i), ii); "un-executed portion of the previous trajectory to warm-start" | — |
| 1.6 | All results simulated (cart-pole, race car w/ non-linear tyre model, quadrotor) at 50 Hz; no model-mismatch test | S49 pp. 6–8 | Verified | p. 6 "three simulated platforms … controller operates at 50 Hz"; p. 8 "tire model"; no mismatch experiment found on pp. 6–8 | — |
| 1.7 | x(t+1)=F(x,v), v~N(u,Σ); "reasonable noise assumption…" | S01 p. 2 | Verified | p. 2 exact | — |
| 1.8 | IT derivation does not need control-affine dynamics | S01 p. 9; S03 p. 1 | Verified | S01 p. 9; S03 p. 1 "without making the control affine assumption" | — |
| 1.9 | Algorithm steps; λ low → concentrated, high → plain average; SG smoothing; shift | S01 pp. 6–8 | Verified | p. 6 Fig. 3 "Low values of λ result in many trajectories being rejected, high values … un-weighted average"; p. 8 Alg. 1. (Note: Alg. 1 calls λ "Inverse Temperature"; p. 6 uses "temperature" loosely) | — |
| 1.10 | Limits by clamping g(v) in dynamics; "works well in practice" | S01 p. 7 | Verified | p. 7 exact | — |
| 1.11 | Chattering removed by SG smoothing | S01 p. 7 | Verified | p. 7 "significant chattering"; Savitsky-Galoy filter | — |
| 1.12 | Warm start "key to achieving a high level of performance"; pure-noise sample fraction | S01 p. 8 | Verified | p. 8 exact; "very small (less than one…" percent) around zero | — |
| 1.13 | 40 Hz, 2 s, λ=12.5, γ=0.1, Σ; 40–60 Hz with a few thousand samples | S01 pp. 7, 13 | Verified | p. 13 Table II; p. 7 "40-60 HZ using a few thousand samples of 2-3 second long trajectories" | — |
| 1.14 | "highly non-linear, but not unstable" | S01 p. 11 | Verified | p. 11 exact | — |
| 1.15 | "require accurate state feedback"; 10 Hz factor graph → 200 Hz | S01 pp. 11–12 | Verified | p. 11 exact; p. 12 "10Hz … A 200 Hz state estimate" | — |
| 1.16 | MPPI "may encounter chattering"; "burden the actuators…" | S46 p. 1 | Verified | p. 1 exact | — |
| 1.17 | External filter may violate limits; "can cause a delay in the system response" (history values) | S46 p. 3 | Verified | p. 3 "may violate the constraint conditions"; "it can cause a delay in the system response. It is common to provide history values" | — |
| 1.18 | SMPPI samples rate of change, spread from "physical limit…"; lower std → "unable to respond…" | S46 pp. 3–4 | Verified | p. 3 exact quotes | — |

### §2 Baseline trackers

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 2.1 | PP arc to goal point one lookahead ahead; one parameter | S06 pp. 10, 14 | Verified | p. 10 arc/goal point; p. 14 "There is one parameter" | — |
| 2.2 | Longer lookahead "more gradually and with less oscillation"; less "curvy" path | S06 p. 14 | Verified | p. 14 exact; "acts as a damping factor" | — |
| 2.3 | PP = P-controller gain 2/ℓd²; lookahead scaled with speed and saturated | S14 PDF p. 17 | Verified | p. 17 "gain of 2/ℓ2d"; "commonly saturated at a minimum and maximum value" | — |
| 2.4 | Shorter → "eventually oscillation"; longer → "eventually stability"; corner cutting; SS error grows with speed | S14 PDF pp. 18, 20, 73 | Verified | p. 20 exact quotes; p. 18 "cutting corners"; p. 73 SS error in curves | — |
| 2.5 | PP stable on straight path at constant speed; small SS error on constant curvature; undefined beyond lookahead | S08 p. 18 | Verified | p. 18 "has a small steady state tracking error"; "controller output is not defined" | — |
| 2.6 | PP variants account for no dynamic effects; diff-drive and Ackermann | S07 pp. 3–4 | Verified | p. 4 "Pure Pursuit nor its variations account for dynamic effects"; p. 3 Ackermann and differential-drive | — |
| 2.7 | RPP speed-scaled lookahead, curvature/proximity slow-down, "prevents consequential undershoot" | S07 pp. 4–5 | Verified | p. 5 exact | — |
| 2.8 | Sim (TurtleBot 3, ground truth): 0.03 vs 0.10 vs 0.19 m | S07 p. 7 | Verified | p. 7 "merely 0.03m … 0.10m and 0.19m"; ground truth p. 7 (TurtleBot 3/Gazebo named on p. 6) | — |
| 2.9 | DWPP: clipping "can result in overshoot … inability to fully realize the planned velocity"; closest to ω=κv | S41 pp. 4, 10 | Verified | p. 4 exact; p. 10 | — |
| 2.10 | Survey table: kinematic trackers LES; linear MPC "depends on horizon length"; NMPC "Not guaranteed"/"Works well in practice" | S08 p. 18 | Verified | p. 18 Table II | — |
| 2.11 | CarSim comparison: geometric trackers fine "until speeds are significantly increased"; LQR+FF less SS error, overshoot | S14 PDF pp. 73–74 | Verified | p. 73 exact; p. 74 "significant overshoot occurs during rapid … changes in path curvature" | — |
| 2.12 | Boss MPC: "very simple kinematic model…, a time delay and rate limits on steering" | S14 PDF p. 8 | Verified | p. 8 exact | — |
| 2.13 | Stanley delays reduce precision/stability; curvature read at t_ff ahead | S44 pp. 615, 634 | Verified | p. 615 abstract; p. 634 constant feedforward time | — |
| 2.14 | Real car: RMS 0.106→0.033 m, max 0.262→0.084 m | S44 p. 630 Table 4 | Verified | p. 630 Table 4, demonstrator vehicle columns | — |

### §3 Motion models inside controllers

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 3.1 | Kinematic models suitable at low speeds | S08 p. 5 | Verified | p. 5 exact | — |
| 3.2 | Simple kinematic model "can suffice…"; unmodelled dynamics → high gains | S14 PDF pp. 72, 74 | Verified | p. 72 exact; p. 74 "same high gain compensation for unmodeled effects" | — |
| 3.3 | Inner loops assumed fast; review treats them as internal disturbance | S23 p. 3 | Verified | p. 3 "commonly assumed that the responses the inner-loops are sufficiently fast … treat the effect of the inner-loops as an internal disturbances" | — |
| 3.4 | 7-state model; learned part = regression on 25 physics-derived basis functions or 2×32 tanh NN | S01 pp. 12–13 | Partly supported | p. 12 seven states, "a set of 21 basis functions" plus extra ones for roll and throttle, "described in appendix A"; p. 13 two hidden layers, 32 neurons, tanh. The number 25 appears only in Appendix A, p. 18 ("form 25 basis function") | Cite p. 18 for "25", or cite pp. 12–13, 18 |
| 3.5 | NN beat BF (R² 0.78 vs 0.68); "both models suffer…hidden variables" | S01 p. 13 | Verified | p. 13 Table I R2 .68/.78; quote exact | — |
| 3.6 | Enhanced kinematic ≈ dynamic accuracy; compute 4.5–5.5× vs 1.3× | S18 p. 7 | Verified | p. 7 "comparable (kin.+ did slightly better on pavement, dyn. … on dirt and grass)"; "4.5-5.5× … only 1.3×" ("matched" is slightly strong; source says "comparable") | — |
| 3.7 | Higher-fidelity models "significantly more difficult to estimate, and computationally expensive" | S24 p. 21 | Verified | p. 21 exact | — |
| 3.8 | 1200 × 2.5 s at 40 Hz ≈ 4.8 M evaluations/s, GPU-only | S01 p. 7 | Verified | p. 7 "approximately 4.8 million queries … only possible using a modern GPU" | — |
| 3.9 | GP residual over fixed unicycle; query includes previous velocity and input; historic states for higher-order dynamics | S11 pp. 1, 5–6 | Verified | p. 6 "disturbance query state … required to include historic states"; "higher-order dynamics by continuing to add historic states" | — |
| 3.10 | Racing MPC GP residual, dictionary of 300 points | S45 p. 1 | Verified | art. p. 1 "small dictionary of 300 data" points | — |
| 3.11 | Nominal error +~40%, GP error "virtually constant"; lap time ~10% (20.2→18.3 s) | S45 p. 7 | Verified | art. p. 7 "increases by almost 40%"; "virtually constant"; "around 20.2 s … 18.3 s … almost 10%" | — |
| 3.12 | Prior-session GP ~20% larger error, "demonstrating the need…" | S45 p. 7 | Verified | art. p. 7 exact | — |
| 3.13 | Maintainer: model "just a pass through"; `predict` "exactly the place…"; sub-10 m/s traditional models good enough | S47 2023-12-06, 2024-01-12 | Verified | 2023-12-06 exact; 2024-01-12 "For low-speed (sub-10 m/s) … I assert traditional dynamics models … are good enough"; DNN for "skidding and very high speeds" | — |

### §4 Identifying the command-to-motion response

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 4.1 | ~30 min, five manoeuvres, 3 min per direction | S01 p. 12; S03 p. 6 | Verified | S01 p. 12 list i)–v), "3 minutes … counter-clockwise and 3 minutes … clockwise … 30 minutes" | — |
| 4.2 | One-step training though multi-step "technically the correct objective"; unregularised → unstable | S01 p. 12 | Verified | p. 12 exact | — |
| 4.3 | One-step "does not necessarily result…"; multi-step errors can grow exponentially | S03 p. 6 | Verified | p. 6 "In the worst case, compounding multi-step errors can grow exponentially" | — |
| 4.4 | Pilot drove differently per direction; model over-steer clockwise | S01 p. 17 | Verified | p. 17 | — |
| 4.5 | IPEM calibrates integrated model; low-rate observations; "optimize simulation accuracy…" | S17 pp. 1, 4 | Verified | p. 1 "requires only low frequency observations … optimize simulation accuracy for the chosen time horizon" | — |
| 4.6 | Delay + first-order model, equation, EKF online from encoders | S17 p. 20 | Verified | p. 20 eq. 50; "estimated online using an extended Kalman filter"; encoder measurement | — |
| 4.7 | DRIVE: 75–470 kg, six terrains, 14.7 km; limits, random uniform commands; 6 s hold (2 s transient + 2 × 2 s steady) | S21 pp. 1–2, 6–7 | Verified | p. 1 "75 kg to 470 kg … six terrains … 14.7 km"; p. 7 "two-second time window … one transient … two steady … six continuous seconds" | — |
| 4.8 | Unloaded calibration steps | S21 pp. 15–16 | Verified | p. 15 "raise the vehicle in the air … longitudinal command at maximum speed … encoders … commanded speed is the same as the encoder speed" | — |
| 4.9 | Earlier protocols: small accelerations, small part of command space | S21 pp. 2–3 | Verified | p. 2 "slowly increasing angular velocity"; p. 3 "only sample small accelerations" | — |
| 4.10 | Steering step → first-order fit, Kδ = 30 Hz, simulated Prius | S16 p. 5 | Verified | p. 5 "set the actuator command to 1 … Kδ = 30 hz" | — |
| 4.11 | ID data with "rapidly changing commands…", both directions, several frictions | S46 p. 5 | Verified | p. 5 exact | — |
| 4.12 | 250 initial points (~two laps) before switching on GP | S45 pp. 6–7 | Verified | art. p. 6 "initial set of 250 data points, corresponding to slightly less than two laps" | — |
| 4.13 | Validation by multi-step replay; Baril per-metre errors | S01 p. 17; S20 p. 6 | Verified | S01 p. 17 Fig. 10 "applied input sequence … initial condition"; S20 p. 6 εt [%], εθ [degree/m] | — |

### §5 Matching limits to the chassis

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 5.1 | DWPP constraints "were determined experimentally" | S41 p. 16 | Verified | p. 16 exact | — |
| 5.2 | Ignoring accel limits → overshoot; DWPP zero violations, smallest errors, order PP, APP, RPP, DWPP, 5 runs, 3 paths, longer time | S41 pp. 3, 21 | Verified | p. 3 abstract; p. 21 "decreasing in the order of PP, APP, RPP, and DWPP"; "no constraint violations"; "longer travel time" | — |
| 5.3 | Smoother limits "consistent with, or greater than" controller's | S30 "Add Dynamic Window Pure Pursuit Option" | Verified | Kilted→Lyrical l. 549–550 (stated for the RPP/DWPP controller) | — |
| 5.4 | Clearpath: controller accel = smoother accel (Husky 0.5, Jackal 10.0); filtered odom | S40 | Verified | a200 `acc_lim_x 0.5`, `acc_lim_theta 0.5`, `max_accel [0.5,0.0,0.5]`; j100 10.0; `platform/odom/filtered` | — |
| 5.5 | Horizon × max speed must fit costmap; 3 s at 0.5 m/s → 1.5 m | S25 "Prediction Horizon…" | Verified | l. 310 exact | — |
| 5.6 | `model_dt` = control period, lower "but not larger" | S25; S27 | Verified | S25 l. 298; S27 l. 309 | — |
| 5.7 | Faster robots larger std; not full speed → increase; chatter → reduce | S27 | Verified | l. 315, l. 325 | — |
| 5.8 | DRIVE wear: skipping gears, 30° wheel bending, tread 12→2 mm, 470 kg, 4 m/s²; limits per terrain or worst | S21 p. 15 | Verified | p. 15 exact figures | — |

### §6 Delay, lag and under-delivery

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 6.1 | Three delay types; "most commonly used control algorithms assume…instantaneously" | S16 p. 1 | Verified | p. 1 exact | — |
| 6.2 | Compensation: shift one step; first-order actuator state; sensor transform | S16 pp. 1–2 | Verified | p. 1 related work; p. 2 "transforming the frame of sensor values" | — |
| 6.3 | Sim: no compensation overshoots then collides; with it "closer to the reference line and smoother" | S16 pp. 5–6 | Verified | p. 5 "overshoots during the first turn … collides"; p. 6 exact | — |
| 6.4 | Racing MPC 50 Hz, forward-simulates fixed delay | S15 p. 14 | Verified | p. 14 exact | — |
| 6.5 | Contributor: "on every real robot, there is a small latency…"; "overshoot and zigzag" | S36 #6065 | Verified | issue body exact | — |
| 6.6 | 4WS hydraulic vehicle oscillated; with comp much better but early steer, corner cutting | S37 PR description | Verified | l. 38–40 exact | — |
| 6.7 | Maintainer: "another way to mask the real issue"; plans to merge | S36 2026-04-07 | Verified | exact; "plan to anyway since its a good feature" | — |
| 6.8 | Skid-steer hydraulic powertrain 0.81 s / 0.16 s | S17 p. 21 Fig. 5 | Verified | p. 21 | — |
| 6.9 | 0.1 s input + 0.2 s steering; 0.2–0.5 s elsewhere | S44 p. 616 | Verified | p. 616 exact | — |
| 6.10 | t_ff 0.18 s sim / 0.20 s real ≈ 0.01 + 0.1 + 0.05, remaining 0.04 s attributed to communication etc. | S44 p. 628 | Verified | p. 628 exact values; "leaves 0.04 s for further delays in communication and effects that are not taken into account" | — |
| 6.11 | 1.21 m at 8 m/s vs 0.12 m at 3 m/s; "drives straight a longer distance…" | S44 pp. 628, 630 | Verified | p. 628 "1.21 m"; p. 630 "(0.12 m) … drives straight a longer distance" | — |
| 6.12 | k 3.0 → 0.8 s⁻¹; yaw-rate filtering delay; breakaway torque | S44 p. 627 | Verified | p. 627 exact | — |
| 6.13 | Offset-free MPC: integrating disturbance + Kalman filter | S05 pp. 50–51 | Verified | PDF p. 100 "augment the system state with an integrating disturbance"; PDF p. 101 "estimated … using a Kalman filter" | — |
| 6.14 | Lemma 1.10 conditions; "does not require that the plant output be generated by the model" | S05 pp. 52–53 | Verified | PDF p. 102 nd = p, stable, constraints inactive; PDF p. 103 exact | — |
| 6.15 | Constrained: "either zero offset, or unbounded, or constraints active" — so a vehicle hitting an input limit "keeps its offset" | S05 p. 49 | Partly supported | PDF p. 99 quote exact. The consequence for a vehicle is the README's own inference, not stated, and not labelled | Label as inference ("i.e., by our reading…") or remove |
| 6.16 | Plain MPC offset under "permanent, nonzero mean, disturbances … and/or mismatch" | S52 p. 1 | Verified | p. 1 exact | — |
| 6.17 | 2×2 example: MPC-0 offset; costs 1.356 / 1.017 / 0.934; 45.2% | S52 pp. 5–6 | Verified | p. 5 "not able to track … without offset"; p. 6 costs, "45.2%"; Fig. 1 step disturbances | — |
| 6.18 | Poles at origin → rougher inputs than poles at 0.5; "the disturbance estimator's speed trades offset removal against noise" | S52 pp. 5–6 | Partly supported | p. 5 "input generated by MPC-1 appears less smooth … two observer poles at the origin … 0.5"; p. 6 "less sensitive to measurement noise". Both variants removed offset equally; the "trades offset removal" clause is an unlabelled generalisation | Rephrase to "faster disturbance estimation (poles at the origin) gave rougher inputs for the same offset removal" or label as inference |
| 6.19 | Offset-free stability assumed rather than proven; 2025 report, "sufficiently small plant-model mismatch" | S53 pp. 1–2 | Verified | p. 1 exact; p. 2 "assumed rather than explicitly demonstrated" | — |
| 6.20 | MPPI "often lacks robustness…"; "far from optimal or in the worst case detrimental" | S51 p. 1 | Verified | p. 1 exact | — |
| 6.21 | 15 races; plain MPPI failed at −40% power, +50% mass, pitching moment; L1 completed all, faster | S51 p. 5 | Verified | p. 5 cases 2, 4, 5; Table II; "run the race 15 times"; "L1 augmentation successfully reduces lap time" | — |
| 6.22 | ICODE-MPPI: "persistent steady-state offset" all paths; Y RMSE −69%; yaw up; additive sinusoidal | S54 pp. 1, 4 | Verified | p. 4 exact; Table II; eq. 13 composite sinusoid, ẋ = fnom + δ(t) | — |
| 6.23 | 900 kg skid-steer, ×0.5 turn rate at 2nd lap ("flat tyre…"), 2 m/s, eight repeats; fixed model "large lateral error" | S50 p. 2186 | Verified | p. 2186 exact; p. 2185 "900 kg Clearpath Grizzly skid-steer" | — |
| 6.24 | wBLR last 3 s (30 samples) each step; large error "but adapts quickly"; fast + long-term best | S50 pp. 2185–2187 | Verified | p. 2185 "last three seconds of data (30 samples)"; p. 2186; p. 2187 "lowest path tracking error and the fastest convergence" | — |
| 6.25 | Fig. 6 medians ≈0.8–0.95 / 0.15–0.25 / 0.1–0.25 m | S50 p. 2187 Fig. 6 | Verified | Rendered plot: no learning ≈0.81–0.96; fast adaptation ≈0.17–0.25; fast+long-term ≈0.09–0.25 | — |
| 6.26 | Learned part linear in [v_cmd, v]; "requires the unknown part … linear in a set of model parameters" | S50 pp. 2180, 2185 | Verified | p. 2185 eq. 31; p. 2180 exact | — |
| 6.27 | ×0.7 / ×1.2 loaded configs; rotational dynamics differ most; GP over-confident at ×1.2, wBLR calibrated | S50 p. 2186 Fig. 4 | Verified | p. 2186 exact | — |
| 6.28 | Traction µ is the same kind of gain, MHE | S12 pp. 5–7 | Verified | p. 6 "effective speed µv"; RHE | — |
| 6.29 | Constant disturbance: integral (straight lines only) or estimation (any path) | S23 p. 23 | Verified | p. 23 exact | — |
| 6.30 | PP absorbs side slip by raising k at constant speed; course-specific | S14 PDF pp. 19, 73 | Verified | p. 19 "increasing k until the circular arc … tighter"; p. 73 "over tuning Pure Pursuit to a specific course" | — |
| 6.31 | Two largest disturbances: unmodelled dynamics (incl. actuators) and wheel-terrain | S11 p. 3 | Verified | sentence starts p. 2, ends p. 3 "and the wheel-terrain interactions" | — |

### §7 Velocity / odometry feedback

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 7.1 | Open loop good "when acceleration limits are set appropriately"; closed loop "high rate and low latency" | S31 `feedback` | Verified | l. 35 exact | — |
| 7.2 | Maintainer: open loop better "only if the odometry … delayed and/or inaccurate…"; otherwise largely same; DWB/MPPI vs RPP | S35 2025-09-15 | Partly supported | l. 31: "If the acceleration limits are accurately set from empirical data and your robot has a professional quality response time, then it could theoretically be better only if…". The README drops this precondition; l. 33 DWB/MPPI/RPP part exact | Add the precondition (accurate empirical limits, fast-responding robot) |
| 7.3 | DWPP author: closed loop may not achieve proper acceleration; "the higher the publish frequency…" | S35 2025-09-18 | Verified | l. 80–81 | — |
| 7.4 | 0.1 m/s² closed loop never produced enough torque; open loop accumulates | S42 | Verified | description bullets; test config `ax_max: 0.1` | — |
| 7.5 | Main: predict one period forward, clamp to accel of last command, predict pose | S28 `prepare` | Verified | l. 312–332 | — |
| 7.6 | "Filtering feedback adds delay": Autoware LPF cutoff "induce operation delay"; curvature smoothing delays feedforward | S38 "Other tips" | Partly supported | Quotes exact, but `steering_lpf_cutoff_hz` is "the second order Butterworth filter installed in the final layer" (output command) and `curvature_smoothing` acts on the reference path — neither filters feedback | Reframe as "filtering (commands or reference) adds delay" or move to a different point |
| 7.7 | Low process-noise weight → accuracy "however, it causes time-lag…" | S12 p. 9 | Verified | p. 9 exact | — |
| 7.8 | Autoware "frequent reports…"; tyre-radius speed offset; cross-check GNSS / IMU | S38 "Confirmation…" | Verified | exact | — |
| 7.9 | Controller at 5 Hz because RTK at 5 Hz | S12 p. 4 | Verified | p. 4 "sampling frequency is set to 5-Hz due to … GNSS" | — |
| 7.10 | diff_drive `open_loop`, `velocity_rolling_window_size` 10, `cmd_vel_timeout` 0.5 s | S39 | Verified | yaml l. 84–112 | — |
| 7.11 | Contributor: closed loop "big snake motion", odometry lagging; delay comp reduced but did not remove oscillation vs open loop | S48 2026-04-07, 2026-04-09 | Partly supported | 04-07 and 04-09 comments match. The same contributor reported on 2026-04-13 (sim): "closed_loop: Together with the delay compensation, I have no oscillations anymore, but it gets stuck before the u-turn" | Add the 04-13 follow-up (oscillation gone in sim with delay comp, but stall before u-turn) |
| 7.12 | Maintainer: "(1) delay … (2) noise…"; hopes open loop unnecessary, kept for no-odometry robots | S48 2026-04-07, 2026-04-13 | Verified | exact | — |
| 7.13 | Noisy command with ground truth, no delay; clamping raw controls removed it | S48 2026-04-14 | Verified | exact | — |

### §8 Slip in skid-steer and tracked vehicles

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 8.1 | Wheels aligned → slip; ICR identification for dead reckoning and motion control | S19 pp. 1, 4–5 | Verified | p. 1; p. 5 motion-control equations | — |
| 8.2 | Asphalt lower turning efficiency "due to greater friction"; α=0.91, xICR=0.275 m vs 0.2 m | S19 p. 5 | Verified | p. 5 exact | — |
| 8.3 | 590 kg, 2 km; ideal DD much worse; >30° more turn on snow; "hard to describe with linear kinematic models" | S20 pp. 6–7 | Verified | p. 1 590 kg, 2 km; p. 6 exact | — |
| 8.4 | Angular error correlated, peaks at ~2:1; translational uncorrelated | S20 p. 7 | Verified | p. 7 exact | — |
| 8.5 | µ, κ; slip = 1−µ, 1−κ; online MHE | S12 pp. 5–7 | Verified | p. 6 exact; RHE = MHE | — |
| 8.6 | 0.0423 vs 0.0514 m; 0.88 ms / 2.85 ms on RPi 3 | S12 pp. 4, 12, 17 | Verified | p. 12, p. 17 exact; p. 4 Raspberry Pi 3 | — |
| 8.7 | EKF cannot enforce bounds; negative estimates → instability | S12 p. 11 | Verified | p. 11 exact | — |
| 8.8 | LB-NMPC "on skid-steer robots (50–600 kg…)"; ~75% and >50% reductions | S11 pp. 1, 10–11, 15 | Partly supported | p. 11 "reduced the maximum lateral and heading errors by roughly 75%"; p. 15 ">50% over … 20 trials". But the 600 kg robot is "600 kg, Ackermann-steered DMRV" (pp. 10–11); p. 2 "both skid and Ackermann steering" | Say "on skid-steer and Ackermann robots (50–600 kg…)" |
| 8.9 | Tube NMPC tractor-trailer, bumpy grass, 7.95 / 5.42 cm | S22 p. 8 | Verified | p. 8 (also p. 7) | — |
| 8.10 | Wheel odometry "not better than 10%"; VO ≥2.5% | S43 p. 3 | Verified | p. 3 exact | — |
| 8.11 | Mahalanobis χ² 95%, 11.07; residual used as slip | S43 p. 13 | Verified | p. 13 "t = 11.07 … the residual is provided to the slip compensation algorithm" | — |
| 8.12 | Yaw-slip in heading loop, crab against lateral slip, forward slip by driving longer | S43 pp. 14–15 | Verified | p. 15 eqs. 24–25 | — |
| 8.13 | ~1 Hz VO sets period; gain tuning order | S43 p. 16 | Verified | p. 16 exact | — |
| 8.14 | 10° slope: 0.05 vs 0.54 m² | S43 p. 18 | Verified | p. 18 exact | — |

### §9 Robustness

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 9.1 | "a severe disturbance can change the initial state … no longer valid" | S04 p. 1 | Verified | p. 1 | — |
| 9.2 | Tube-MPPI nominal + iLQG ancillary; divergence failure; sampler ignores feedback | S13 pp. 1–3, 5; S04 pp. 2–3 | Verified | S13 p. 5 "Ancillary Controller - iLQG"; S04 p. 3 "primary failure case"; "IS trajectories do not reflect the ancillary feedback controller" | — |
| 9.3 | RMPPI; MPPI reliable only to 9 m/s; MPPI ≈ Tube-MPPI; longer line, not faster | S04 pp. 7–8 | Verified | pp. 7–8 exact | — |
| 9.4 | Time-decaying boundary penalty; fixed impulse "fails on the actual system" | S01 p. 13 | Verified | p. 13 exact | — |
| 9.5 | Only 11 m/s failures from "systematic modeling error … under-steer" | S01 p. 18 | Verified | p. 18 exact | — |
| 9.6 | Learned models within 10% of known model (cart-pole, quadrotor) | S03 p. 6 | Verified | p. 6 exact | — |
| 9.7 | Delay-aware tube MPC, disturbance-invariant set | S16 pp. 1–3 | Verified | pp. 2–3 | — |

### §10 Testing and metrics

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 10.1 | Metrics list (cross-track, heading, violations, travel time, success rate, RTK Euclidean, per-metre errors) | S41 p. 16; S11 p. 13; S01 pp. 15–16; S12 p. 12; S20 p. 6 | Verified | each at cited page (S01 success-rate table on p. 14, discussed p. 16) | — |
| 10.2 | Test paths (45/90/135°, step + blind 90° ×10, serpentines/circles, crop rows, laps, teach-and-repeat) | S41 p. 14; S07 pp. 7–8; S37; S12 p. 17; S22 p. 8; S01; S11 | Verified | S07 p. 7 "sharp 90-degree turn", p. 8 "ten times" | — |
| 10.3 | RPP sim with ground truth "to remove the contribution…" | S07 pp. 6–7 | Verified | p. 7 exact | — |
| 10.4 | Sim-to-real: fine-tune; CarSim rate limits and delays; S16 Gazebo only | S01 p. 13; S14 PDF p. 10; S16 p. 5 | Verified | exact | — |
| 10.5 | Maintainer: compare simulated "actual" speed with controller input | S35 2025-09-17 | Verified | l. 63 | — |

### §11 Product specifics

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| 11.1 | DiffDrive, Omni, Ackermann (+ min turning radius) | S25; S26 | Verified | S25 l. 28, 78 | — |
| 11.2 | Humble step 0 from odometry; later steps = previous sampled command | S26 | Verified | optimizer l. 255–256; motion_models l. 57–61 | — |
| 11.3 | Humble params list; no ax_max/open_loop/model_delay | S26 `getParams` | Verified | l. 62–84; none of the others in file | — |
| 11.4 | Sequence clipped after weighted average | S26 | Verified | l. 387 → `applyControlSequenceConstraints` l. 231–242 | — |
| 11.5 | Period = model_dt → shifting on, element 1 sent | S26 | Verified | l. 104–109; l. 393 `offset = … ? 1 : 0` | — |
| 11.6 | Latest odom twist via `getThresholdedTwist`, no averaging/age check | S34 l. 478; `odomCallback` | Verified | controller_server l. 478; odom_subscriber l. 84–92 | — |
| 11.7 | ax_max, ax_min, ay_max, az_max added in PR #4352 | S30 Iron→Jazzy | Verified | l. 308–310 | — |
| 11.8 | Rollout limited to model_dt × a_max; clamp_raw_controls "too noisy" | S28; S29 | Verified | motion_models l. 109–155; S29 l. 174 | — |
| 11.9 | open_loop PR #5617; quote "useful when…" | S30; S29 | Verified | Kilted→Lyrical l. 414–418; S29 l. 218–222 | — |
| 11.10 | Per-axis delay, ring buffer, shift; best with open_loop; Lyrical+, no backport | S30; S37 2026-06-08 | Verified | l. 920–930; S37 l. 121–123 | — |
| 11.11 | Odometry "at least as fast as your control frequency (ideally much faster)" | S27 | Verified | l. 315 | — |
| 11.12 | MPPI "moderately higher compute costs" than simpler trackers | S33 | Partly supported | Tuning guide: "similar to DWB … MPPI however does have moderately higher compute costs" — the comparison is with DWB, not "simpler trackers" | Say "than DWB" |
| 11.13 | Maintainer: jitter; lower std smoother; obstacle critic can add jitter; low-confidence impression | S47 2023-03-07, 2023-03-13 | Verified | 03-07 "some jitter in the output path"; 03-13 exact, "I don't have defensible metrics" | — |
| 11.14 | Smoother defaults OPEN_LOOP, [2.5,0,3.2], 0.1 s, 1.0 s, deadband / stall torque | S31 | Verified | l. 33, 56, 60, 66, 84 | — |
| 11.15 | RPP max_linear_accel, lookahead params, collision look-ahead in time | S32 | Verified | l. 98, 128–140, 176–180 | — |
| 11.16 | Autoware models and tuning order | S38 | Verified | l. 27–30, 177–183 | — |
| 11.17 | Lateral offset "most often due to…"; optional offset remover | S38 | Verified | l. 224 | — |
| 11.18 | wheel_separation_multiplier, radius multipliers; limits off by default | S39 | Verified | l. 39–51; `has_*_limits` default false | — |

### Recommended practice

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| R1 | Fit delay + first-order lag; validate multi-step | S17 pp. 4, 20; S16 p. 5; S03 p. 6 | Verified | as 4.5, 4.6, 4.10, 4.3 | — |
| R2 | Cover reachable command space, transients, both directions, hold to steady state | S21 pp. 6–7; S01 p. 17 | Verified | as 4.7, 4.4 | — |
| R3 | Limits from measured capability; smoother limits ≥ controller | S41 p. 16; S30; S40 | Verified | as 5.1, 5.3, 5.4 | — |
| R4 | Horizon × speed fits map; model_dt ≤ period | S25 | Verified | as 5.5, 5.6 | — |
| R5 | Closed loop needs fast odometry; open loop needs accurate limits and responsive platform | S31; S35 | Verified | S31 l. 35; S35 l. 31 | — |
| R6 | Compensate delay in the model | S16 pp. 1–2; S15 p. 14; S30 | Verified | as 6.2, 6.4, 11.10 | — |
| R7 | Per-terrain slip kinematics, online traction, or learned residual | S19 p. 5; S12 pp. 5–7; S11 p. 1 | Verified | as 8.2, 8.5, 3.9 | — |
| R8 | Estimate a persistent gain error (offset-free, online parameter, adaptive inner loop) | S05 pp. 50–53; S52 p. 1; S12; S50 pp. 2185–2186; S51 pp. 1, 5 | Verified | as 6.13–6.16, 6.24, 6.21 | — |
| R9 | Fast adaptation vs noise | S50 p. 2186; S52 pp. 5–6 | Verified | as 6.24, 6.18 (observer-pole result) | — |
| R10 | Cross-check inputs with independent sensor | S38 | Verified | as 7.8 | — |

### Key numbers

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| K1 | 40 Hz / 2 s | S01 p. 13 | Verified | Table II | — |
| K2 | 1200 × 2.5 s ≈ 4.8 M/s, GTX 750 Ti | S01 p. 7; S03 p. 6 | Verified | S01 p. 7; S03 p. 6 "Nvidia GTX 750 Ti" | — |
| K3 | Humble defaults (0.05, 56, 1000, 0.5, 1.9, 0.2, 0.4) | S26 `getParams` | Verified | l. 69–81 | — |
| K4 | ax_max 3.0, ax_min −3.0, az_max 3.5 — "Jazzy and later" | S29 | Partly supported | S29 l. 128–154 defaults exact; S29 is rolling docs and does not state the release; "Jazzy" comes from S30 Iron→Jazzy | Add S30 to the citation |
| K5 | 0.81 s / 0.16 s LandTamer | S17 p. 21 | Verified | p. 21 | — |
| K6 | 600 ms, model_delay_wz = 0.6, hydraulic 4WS | S30; S37 | Verified | S30 l. 936; S37 l. 38 | — |
| K7 | 0.0423 vs 0.0514 m, sorghum, 5 Hz RTK | S12 p. 12 | Verified | p. 12; p. 4 | — |
| K8 | ~75%, >50% over 20 trials, 0.35–1.2 m/s | S11 pp. 11, 15 | Verified | pp. 11, 15 (condition "Skid-steer" — see 8.8; the 75% and >50% runs themselves are cited correctly) | — |
| K9 | 0.03 vs 0.19 (APP 0.10) m, sim, 1.0 m/s | S07 p. 7 | Verified | p. 7 | — |
| K10 | 1.3× vs 4.5–5.5× | S18 p. 7 | Verified | p. 7 | — |
| K11 | DRIVE 6 s hold | S21 p. 7 | Verified | p. 7 | — |
| K12 | 0.1 + 0.2 s; 0.2–0.5 s | S44 p. 616 | Verified | p. 616 | — |
| K13 | t_ff 0.18 / 0.20 s | S44 p. 628 | Verified | p. 628 | — |
| K14 | 0.106 / 0.262 vs 0.033 / 0.084 m | S44 p. 630 | Verified | Table 4 | — |
| K15 | ≥10% vs ≤2.5% | S43 p. 3 | Verified | p. 3 | — |
| K16 | 0.05 vs 0.54 m², 10°, 4.5 m | S43 p. 18 | Verified | p. 18 | — |
| K17 | ×0.5, ×0.7, ×1.2; 900 kg, 10 Hz, 3 s, 2 m/s | S50 pp. 2185–2186 | Verified | p. 2185 "10 Hz with a three second look-ahead"; p. 2186 | — |
| K18 | Fig. 6 medians | S50 p. 2187 | Verified | as 6.25 | — |
| K19 | 3 s (30 samples) window | S50 p. 2185 | Verified | p. 2185 | — |
| K20 | −40% power; +50% mass; 0.1 N m; 15 runs | S51 p. 5 | Verified | p. 5 | — |
| K21 | 1.356 vs 1.017 / 0.934; 45.2% | S52 p. 6 | Verified | p. 6 | — |
| K22 | Y RMSE −69%, additive sinusoidal | S54 pp. 1, 4 | Verified | p. 1, p. 4 | — |
| K23 | 300 points; −≈10%; +≈40% | S45 pp. 1, 7 | Verified | art. pp. 1, 7 | — |

### How it is tested

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| T1 | Repeated laps; success rate over up to ~100 laps | S01 pp. 15–16 | Verified | p. 14 Table III, p. 16 discussion | — |
| T2 | Model replay, 2 s horizon | S01 p. 17 | Verified | p. 17 | — |
| T3 | Corner paths, five runs, violations | S41 pp. 14–21 | Verified | pp. 14, 16, 21 | — |
| T4 | Step path + blind corner ×10 | S07 pp. 7–8 | Verified | p. 8 "ten times", collisions | — |
| T5 | Serpentine and circle | S37 | Verified | l. 38–47 | — |
| T6 | Crop row, 0.12 m tolerance | S12 pp. 2, 12 | Verified | p. 2 "strictly less than 0.12 m" | — |
| T7 | DRIVE random commands | S21 pp. 6–7 | Verified | p. 7 | — |
| T8 | Powertrain fit, model vs encoder | S17 p. 21 | Verified | p. 21 Fig. 5 "as the LandTamer decelerates"; "closely match the actual wheel velocities measured by the encoders" | — |
| T9 | Step-steer 3 and 8 m/s; circuit | S44 pp. 628–630 | Verified | pp. 628–630 | — |
| T10 | Slope with/without compensation | S43 p. 18 | Verified | p. 18 | — |
| T11 | ×0.5 turn rate from 2nd lap, eight repeats | S50 pp. 2186–2187 | Verified | p. 2186 | — |
| T12 | Perturbation cases, 15 runs, completion | S51 p. 5 | Verified | p. 5 | — |
| T13 | M-RMSZ −0.5 to 1.5 acceptable; >2.0 over-confident | S50 p. 2186 | Verified | p. 2186 exact | — |
| T14 | Laps before/after GP; share inside 1σ | S45 p. 7 | Verified | art. p. 7 Table I "1-σ", "65 to 69% … within the 1 − σ" | — |

### Common mistakes

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| M1 | Perfect-dynamics sim + hard penalties | S01 p. 13 | Verified | exact | — |
| M2 | Direction-imbalanced ID data → fails in that direction | S01 pp. 17–18 | Verified | pp. 17–18 | — |
| M3 | One-step fit without regularisation → unstable | S01 p. 12 | Verified | p. 12 | — |
| M4 | Ignoring delay → overshoot, oscillation, late turn-in | S16 p. 5; S37; S38 | Verified | as 6.3, 6.6, Σ2 | — |
| M5 | Downstream clipping of PP → overshoot; MPPI + smoother: "doesn't actually respect my dynamics", limits "slightly broken" with newer odometry | S41 p. 4; S47 | Verified | S41 p. 4; S47 2024-01-12 l. 421 exact | — |
| M6 | Closed loop with slow odometry or tiny steps | S31; S35; S42 | Verified | as 7.1–7.4 | — |
| M7 | Over-tuning PP lookahead to one course | S14 PDF p. 73 | Verified | p. 73 | — |
| M8 | Ideal DD on skid-steer | S20 p. 6 | Verified | p. 6 "much higher than for any of the trained models" | — |
| M9 | High random accelerations on high traction → wear | S21 p. 15 | Verified | p. 15 | — |
| M10 | Filtering yaw rate → reduced stabilisation, lower gains | S44 p. 627 | Verified | p. 627 "delay of the signal and, thus, a reduced stabilization effect" | — |
| M11 | External MPPI smoothing delays, breaks limits | S46 p. 3 | Verified | p. 3 | — |
| M12 | Wheel odometry ≥10%, ≥4× VO (≤2.5%) | S43 p. 3 | Verified | p. 3 (4× is arithmetic from the two figures) | — |
| M13 | Stale residual model ~20% worse | S45 p. 7 | Verified | art. p. 7 | — |
| M14 | No disturbance estimate → steady offset | S52 pp. 1, 5; S54 p. 4 | Verified | S52 p. 5; S54 p. 4 | — |
| M15 | Long-term learning alone: "large error on the first lap after the change" and slow convergence against outdated prior | S50 p. 2186 | Partly supported | p. 2186 "Long-term learning … incurs a large path tracking error on the first run (see Fig. 6) since there are no previous runs … converges slowly because it is constantly working against a static prior". The source's unit is the first *run* (repeat) of eight, not the first lap after the change | Replace "first lap after the change" with "first run (repeat) after the change" |

### Disagreements

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| D1 | Open vs closed loop: smoother docs; maintainer "largely the same unless odometry is delayed or inaccurate"; DWPP author; PR #5617; main blends | S31; S35; S42; S28 | Partly supported | S35 l. 31 carries the precondition "If the acceleration limits are accurately set from empirical data and your robot has a professional quality response time"; the rest matches | Add the precondition to the maintainer's view |
| D1b | Contributor: closed loop "snaked" even with delay comp; maintainer hopes open loop unnecessary | S48 | Partly supported | 04-09 "improves the oscillations but still oscillates a lot more than with the open_loop"; 04-13 (sim) "Together with the delay compensation, I have no oscillations anymore, but it gets stuck before the u-turn" | Mention the 04-13 result |
| D2 | Delay parameter needed? gains vs masking | S36; S37 | Verified | as 6.5–6.7 | — |
| D3 | One-step vs multi-step identification | S01 p. 12; S17 p. 4 | Verified | S17 p. 4 "Predictive performance is optimized for a longer horizon" | — |
| D4 | RMPPI outperformed; hardware not faster; MPPI ≈ Tube | S04 pp. 1, 8 | Verified | p. 1 abstract; p. 8 | — |
| D5 | SG smoothing vs SMPPI vs sampling spread | S01 p. 7; S46 p. 3; S47 | Verified | as 1.11, 1.17, 11.13 | — |
| D6 | MPPI robustness to model error: 10% vs "often lacks robustness", 40% loss, persistent offset | S03 p. 6; S51 pp. 1, 5; S54 p. 4 | Verified | as 9.6, 6.20–6.22 | — |
| D7 | Where to correct a gain error; no same-vehicle comparison | S12; S50; S05; S51 | Verified | as 6.28, 6.24, 6.13, 6.21; no such comparison found in the sources | — |
| D8 | `model_delay_wx` (S29) vs `model_delay_wz` (S30, S37) | S29; S30; S37 | Verified | S29 l. 80; S30 l. 928; S37 l. 25 | — |

### Cited open questions

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| Q1 | Gap-fill: ×0.5 turn rate (S50), −40% multirotor MPPI (S51), offset-free theory (S05, S52); S50 turn-rate only, gradient-based MPC not MPPI | S50; S51; S05; S52 | Verified | S50 p. 2185 "solved as a sequential quadratic program" (gradient-based); only turn-rate scaling tested | — |
| Q2 | Maintainer states noise is not modelled | S48 | Verified | 2026-04-13 "(2) noise (which we haven't)" | — |
| Q3 | Online ICR estimation covered only indirectly via S12, S43 | S12; S43 | Verified | S12 traction µ, κ; S43 slip vector | — |

## Uncited factual statements

- Foundational table, S08 row: "Most-cited survey" (counted as F9).
- Open questions 1, 2, 4, 5, 6, 8, 9: search histories and availability statements (closed access, bot checks) carry no source citation; they are process notes and cannot be verified from `sources/`. Not counted as claims.

## Items needing correction (Partly supported)

Σ1, Σ3, Σ4, F9, 3.4, 6.15, 6.18, 7.2, 7.6, 7.11, 8.8, 11.12, K4, M15, D1, D1b — corrections in the tables above. No item is Not supported.

## Corrections applied (2026-09-28)

The topic editor applied both reviews (this file and `SOURCE_AUDIT.md`). Each cited passage was re-read in the source file before rewriting.

**Partly supported (16), corrected in `README.md`**
- Σ1: the paraphrased quote "enters through the controls" replaced with the source's words from p. 2 ("has to pass through a lower level of control before reaching the actual system") and p. 9 ("the noise enters the system through the control input"); citation now pp. 2, 8–9.
- Σ3: "commonly modelled" replaced with the source wording "can be modeled adequately", plus "most WMR motion models in related work omit powertrain dynamics" (S17 p. 20).
- Σ4: the 600 ms vehicle is now described as in S30 (a vehicle with 600 ms steering delay), with the hydraulic four-wheel-steer detail cited to C4-S37 (PR #6154 description).
- F9: "Most-cited" dropped from the S08 foundational row; now "Survey comparing vehicle models and path trackers, with a stability comparison table".
- 3.4: citation extended to pp. 12–13, 18 (the number 25 is on p. 18).
- 6.15: the vehicle consequence is now labelled as our reading, outside the cited quote.
- 6.18: rephrased to "faster disturbance estimation (poles at the origin) gave rougher inputs for the same offset removal".
- 7.2 and D1: the maintainer's precondition (accurate empirical acceleration limits, fast-responding robot) added.
- 7.6: reframed as filtering inside the controller (output-command Butterworth filter, reference-curvature smoothing), not feedback filtering.
- 7.11 and D1b: added the contributor's 2026-04-13 simulation result (no oscillation with delay compensation in closed loop, but stuck before the u-turn); both S48 contributor bullets now labelled as unconfirmed user reports.
- 8.8: "skid-steer robots" → "skid-steer and Ackermann-steered robots"; the 600 kg robot is Ackermann-steered.
- 11.12: "than simpler trackers" → "than DWB", with the tuning guide's "similar to DWB" context.
- K4: C4-S30 added to the citation (release information).
- M15: "first lap after the change" → "first run (repeat) after the change".

No claim was Not supported; none removed on that ground.

**Source corrections (from `SOURCE_AUDIT.md`; no source failed, none deleted)**
- Levels: C4-S26, S28, S34, S39 (official project source code) C → B. C4-S06 and C4-S14 (non-peer-reviewed CMU technical reports) A → C, matching C4-S53; the rule is recorded above the Sources table. C4-S49 A → C (see below).
- Pinned commits (each saved file was compared with the file at the commit and matched exactly, CRLF ignored): navigation2 humble `3c3db59d6969` (S25, S26, S34), navigation2 main `7b9bcb4c2d68` (S27, S28), docs.nav2.org `588d37415e87` (S29–S33; the migration guides are `iron/Iron.md`, `jazzy/Jazzy.md`, `kilted/Kilted.md`), autoware_universe `7710ed4d86b2` (S38), ros2_controllers humble `2105376a66d5` (S39), clearpath_nav2_demos humble `40db8f47ddca` (S40). Links now point to the commits. S34's odom subscriber path corrected to `nav2_dwb_controller/nav_2d_utils/...`.
- Merge commits recorded: PR #6154 → `374cd2556640` (2026-06-08, S37); PR #5617 → `8dbc92910b02` (2025-10-23, S42).
- C4-S36: now cites only the maintainer's comment. The §6 bullet quoting the opening post ("on every real robot, there is a small latency…") was removed; the "Is a delay parameter needed?" disagreement now cites C4-S37 and C4-S30 for the gains and C4-S36 only for the maintainer.
- C4-S48: row states that contributor reports are cited only as labelled user reports (see 7.11).
- C4-S49: the source read is the 2015 preprint arXiv:1509.01149v3 ("…using covariance variable importance sampling"), now cited as such with the JGCD 2017 article named as its published successor (not read); level C; file renamed `williams_2015_mppi_covariance_importance_sampling.pdf`. Foundational row updated and the reference added to the SCOPE.md foundational table.
- C4-S41: the IAS-19 (2025) and JSME Robomec 2025 (doi:10.1299/jsmermd.2025.2a2-q09) peer-reviewed versions are named; the saved file is the 30-page extended arXiv manuscript, so it stays at level C.
- Citation details added: DOIs for S01, S07, S12, S16, S20, S22, S23; CRV 2020 pp. 198–205 (S20); TMECH 20(1):447–456 and the wrong arXiv header noted (S22); JFR 40(3):747–779 (S23); S16 marked as arXiv v3 revised after the IV paper.
- "Page numbers refer to the arXiv PDF" added to S01, S07, S08, S12, S15, S20–S24.
- File names: 25 files renamed to `<org>_<year>_<topic>` (all `nav2_*`, `autoware_*`, `clearpath_*`, `ros2_controllers_*`; year = commit year for code/docs, creation year for GitHub threads) plus the S49 rename; README file columns updated.
- Format: no PDF upgrades needed (audit found none).

**Final checks**: `file` output matches the extension for all 57 files; every file is in the Sources table and every table file exists; every cited ID exists; IDs C4-S01…C4-S54 have no gaps or duplicates. Status set to **Verified**: no Partly supported or Not supported items and no failing sources remain.
