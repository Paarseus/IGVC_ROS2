# C3 — Command pipeline: scope map

Mapped: 2026-09-27 (step 1 of STANDARDS.md §5). Nothing downloaded in this step.

## Overview

The command pipeline is the chain a motion command travels through between the navigation controller and the motors: the controller's velocity output, optional smoothing and limiting, a selector (multiplexer) that decides which source — autonomy, a human teleoperator, or a safety function — is in control, the base driver that turns body velocity into wheel commands, and the link to the motor controllers. The field this draws on is broad. Control theory explains how nested ("cascaded") loops should be split and why adding feedback loops at the wrong layer causes overshoot and windup (an integrator piling up error while its output is clipped). Machine-safety standards (ISO 13849, IEC 60204-1 / ISO 13850, IEC 61508, ISO 3691-4 for driverless trucks, ANSI/RIA R15.08, ISO 18497 for farm machines, IEC 61800-5-2 for drives) define stop categories, independent emergency stops, watchdogs and stopping-distance calculations. Real-time systems research on ROS 2 (executor response-time analysis, DDS latency, tracing) gives the tools to budget and measure latency and jitter (variation in timing). Reference implementations — ROS 2 `twist_mux`, Nav2 `velocity_smoother` and `collision_monitor`, ros2_control `diff_drive_controller`, Clearpath base drivers, Autoware's `vehicle_cmd_gate`, JPL Mars rover drive sequencing — show how these ideas are put into practice across AMRs, cars, AGVs and planetary rovers. Motor-controller internal PID/feedforward (C1), kinematics (C2) and controller internals such as MPPI (C4) are excluded.

## Subtopics

| # | Subtopic | Questions | Covered already? |
|---|---|---|---|
| 1 | Pipeline architecture and layering | (a) What are the standard layers between planner and motor in reference stacks (Nav2, ros2_control, Clearpath, Autoware, JPL rovers), and what is each layer responsible for? (b) Which layer "owns" the vehicle state estimate that closes the outer loop? (c) How do hierarchical/cascaded architectures document the interface (units, frames, stamped vs unstamped messages) between layers? (d) What does a reference stack do when a layer is missing or bypassed? | No |
| 2 | Cascade control theory and extra inner corrections | (a) What bandwidth / time-scale separation between inner and outer loops does control theory require (e.g. inner loop ≥ several times faster)? (b) Why do extra closed-loop corrections below the navigation controller (heading-hold, yaw-rate feedback in the base driver) conflict with an outer loop that already closes on the same state, and what failure modes result (fighting loops, overshoot, windup, limit cycles)? (c) When are such inner corrections acceptable (much faster loop, disturbance rejection, loop the outer layer does not close, open-loop teleop)? (d) How do anti-windup and "bumpless" techniques handle saturation or rate limits placed inside a cascade? | No |
| 3 | Command arbitration and hand-over | (a) How do priority multiplexers select among autonomy, teleop and safety sources (priorities, per-source timeouts, lock inputs)? (b) How are deadman/enable buttons and explicit mode switches (manual/auto) used, and what lockouts prevent unintended resumption? (c) How is control handed back from teleop to autonomy safely — does the planner re-plan, is state reset, is the smoother re-seeded to actual velocity (bumpless transfer)? (d) Where should the safety override sit relative to the mux (before or after the last limiter)? | No |
| 4 | Emergency stop and safety architecture | (a) What stop categories (IEC 60204-1 cat. 0/1/2) and e-stop requirements (ISO 13850) apply, and why must the e-stop not depend on software or communication? (b) How do reference robots split hardware e-stop (relay cutting motor power, independent MCU loop) from software stop topics? (c) How are performance levels / integrity levels (ISO 13849 PL, IEC 61508 SIL) assigned to stop, speed-limit and watchdog functions? (d) What do mobile-machine standards (ISO 3691-4, ANSI/RIA R15.08, ISO 18497/25119) require for protective stops, speed monitoring and resets? (e) How is a latched e-stop cleared and motion re-enabled? | No |
| 5 | Placement and consistency of speed, acceleration and jerk limits | (a) At which layers do reference stacks apply limits (controller constraints, velocity smoother, collision monitor, base-controller limiter, motor-controller ramp), and what is each layer's purpose (planning feasibility vs smoothness vs hard safety guard)? (b) What goes wrong when limits are duplicated or inconsistent (controller predicts motion the base cannot deliver, stacked ramps add lag, limits that are configured but never read)? (c) Open-loop vs closed-loop feedback in a velocity smoother — which is recommended and when? (d) How are asymmetric accel/decel limits and deadbands handled? | Partly — `sources/ros2control_2024_diff_drive_controller.md` (limiter parameters of diff_drive_controller) |
| 6 | Stale commands, timeouts, watchdogs and heartbeats | (a) What command timeouts do reference nodes use (mux per-source timeout, smoother `velocity_timeout`, diff_drive `cmd_vel_timeout`, firmware watchdog) and how do they cascade? (b) How are message age and stamps used (TwistStamped, twist_stamper) versus arrival time? (c) What do DDS/ROS 2 QoS deadline and liveliness policies offer for detecting dead publishers? (d) What does functional-safety guidance (IEC 61508 watchdogs with independent time base) say about watchdog design? (e) What happens on communication loss to the motor controller (CAN heartbeat loss, serial disconnect)? | Partly — `sources/ros2control_2024_diff_drive_controller.md` (`cmd_vel_timeout`) |
| 7 | Stopping behaviour: braking, coasting, zero-velocity hold | (a) How do controlled stops (ramped deceleration) differ from power-removal stops in stopping distance and hazard? (b) What are drive-level stop functions (STO, SS1, SS2, SOS in IEC 61800-5-2) and how do they map to "brake", "coast" and "hold zero velocity"? (c) Brake vs coast in motor drivers (slow vs fast current decay) and the effect on stopping and on creeping at "zero" command? (d) How are stopping distance and protective-field size calculated from total response time (ISO 13855-style S = K·T + C, AGV braking-distance tests)? (e) How are deadbands and "send explicit stop" logic used to avoid residual motion at rest? | No |
| 8 | Loop rates, latency and jitter budgets | (a) What loop rates do reference systems use at each layer (planner, controller ~10–50 Hz, smoother, ros2_control update loop, motor controller kHz)? (b) How does latency accumulate end to end (executor scheduling, DDS transport, serial/CAN links, motor-controller filtering) and what are typical measured values? (c) What does ROS 2 response-time analysis (Casini, Blaß, Tang) say about chain latency under the default executors? (d) What jitter is tolerable for a velocity loop and how does delay reduce stability margin (phase lag)? (e) How do real-time setups (PREEMPT_RT, thread priorities, memory locking) change jitter? | No |
| 9 | Measuring and testing the pipeline | (a) How is end-to-end command latency measured (ros2_tracing/LTTng, stamped messages, hardware loop-back, step command vs measured wheel response)? (b) What test methods verify arbitration and timeouts (fault injection: kill a publisher, unplug link, stale stamps)? (c) How are e-stop response time and stopping distance tested for AGVs/AMRs? (d) What pass criteria do sources use? | No |
| 10 | How other vehicle types do it | (a) Automotive: how does Autoware's `vehicle_cmd_gate` filter and switch commands, and how are minimal-risk manoeuvres (MRM) triggered? What do ISO 26262 / fail-operational concepts say about arbitration logic? (b) AGVs / industrial trucks: speed monitoring, protective fields, mode switching. (c) Agricultural machines: ISO 18497 obstacle protection as a safety function. (d) Planetary rovers: guarded motion, parallel safety sequences that halt a drive, command sequencing with limited bandwidth. (e) Teleoperation under delay: move-and-wait, supervisory control. | No |
| 11 | ROS 2 / product-specific implementations (last) | (a) `twist_mux` parameters (priority 0–255, timeout, locks) and `twist_stamper`; (b) Nav2 `velocity_smoother` (OPEN_LOOP/CLOSED_LOOP, `velocity_timeout`, deadband) and `collision_monitor` (stop/slowdown/approach, placement after the smoother); (c) ros2_control controller manager read→update→write loop, `diff_drive_controller` SpeedLimiter and timeout; (d) Clearpath platform twist_mux config and MCU e-stop loop; (e) Nav2 maintainer guidance on open vs closed loop across controller/smoother (e.g. navigation2 issue #5524). | Partly — `sources/ros2control_2024_diff_drive_controller.md` covers (c) |
| 12 | Control timing: latency vs jitter (added in gap check) | (a) How are sampling latency, sampling jitter and input-output latency defined? (b) Is constant latency preferable to varying latency? (c) How much sampling jitter can be ignored? (d) Why does meeting deadlines not guarantee control performance? | Added 2026-09-27 — C3-S42; findings in README §8 |
| 13 | Stopping-distance and protective-field sizing for vehicles (added in gap check) | (a) How is stopping distance built from sensor, control and braking times? (b) What supplements are added (tolerance, ground clearance, brake wear)? (c) How is speed-dependent field switching handled? | Added 2026-09-27 — C3-S46; findings in README §7 |
| 14 | Command interface conventions (added in gap check) | (a) What units and axis conventions do velocity commands follow (REP 103)? (b) What does a stamp added downstream actually mean (twist_stamper)? | Added 2026-09-27 — C3-S52, C3-S51; findings in README §1, §6 |
| 15 | Degraded modes and fail-operational arbitration (added in gap check) | (a) How do automated cars grade their response to faults (comfort stop, safe stop, emergency stop)? (b) How are latched emergencies and recovery handled (Autoware MRM)? (c) How is the arbitration logic itself verified? | Added 2026-09-27 — C3-S47, C3-S53; findings in README §3, §4, §7, §10 |
| 16 | Designed inner rate loops in other vehicle stacks (added in gap check) | (a) Where do drone and rover autopilots put a yaw-rate loop, and how is it tuned relative to outer loops? (b) When do they turn the inner loop off (non-minimum-phase cases)? | Added 2026-09-27 — C3-S40, C3-S41; findings in README §1, §2, §5 |
| 17 | Deadman/enable devices and actuator-level watchdogs (added in gap check) | (a) How does standard ROS teleop use a deadman button and what does it send on release? (b) What per-actuator and system watchdog timeouts do competition robots and motor controllers use, and what happens on signal loss? | Added 2026-09-27 — C3-S44, C3-S43, C3-S48; findings in README §3, §6, §7 |
| 18 | Competition safety rules for student ground vehicles (added in gap check) | (a) What e-stop, speed-governing and autonomous-mode indication rules apply, and how are they checked at qualification? | Added 2026-09-27 — C3-S39; findings in README §3, §4, §5, §9 |
| 19 | Fault-injection testing of the command chain (added in gap check) | (a) What published test procedures kill publishers, inject stale stamps or toggle locks, and with what pass criteria? | Added 2026-09-27 — not filled (no reputable source found) |

## Foundational references

| Citation | Why foundational | Open copy |
|---|---|---|
| K. J. Åström, R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, 2nd ed., Princeton University Press, 2021. | Standard control textbook; loop shaping, delay/phase margin, cascade and feedforward structure used to judge where loops belong. | https://fbsbook.org (2nd ed., CC licence); 1st ed. PDF: https://www.cds.caltech.edu/~murray/books/AM08/pdf/am08-complete_28Sep12.pdf |
| S. Skogestad, I. Postlethwaite, *Multivariable Feedback Control: Analysis and Design*, 2nd ed., Wiley, 2005. | Standard graduate text; chapter on control structure design (hierarchical/cascade control, time-scale separation between layers). | No open copy verified |
| K. J. Åström, T. Hägglund, *Advanced PID Control*, ISA, 2006. | Standard PID reference; cascade control, windup and anti-windup, rate limiting and bumpless transfer between manual and automatic. | Only chapter 1 sample: https://www.isa.org/getmedia/fb0e41bc-e4f3-422a-9f67-b9bd31340e16/Advanced-PID-Control_AstromHagglund_Chapter1-Introduction.pdf |
| S. Macenski, F. Martín, R. White, J. Ginés Clavero, "The Marathon 2: A Navigation System," IEEE/RSJ IROS, 2020. | Seminal Nav2 paper; defines the navigation architecture that emits the velocity commands. | https://arxiv.org/abs/2003.00368 |
| S. Macenski, T. Moore, D. V. Lu, A. Merzlyakov, M. Ferguson, "From the desks of ROS maintainers: A survey of modern & capable mobile robotics algorithms in the Robot Operating System 2," *Robotics and Autonomous Systems* 168, 104493, 2023. | Maintainer survey of the ROS 2 mobile stack, including controllers, smoothing and safety layers. | https://arxiv.org/abs/2307.15236 |
| S. Chitta et al., "ros_control: A generic and simple control framework for ROS," *Journal of Open Source Software* 2(20), 456, 2017. doi:10.21105/joss.00456 | Origin of the controller-manager / hardware-interface split used by ros2_control and diff_drive_controller. | https://www.theoj.org/joss-papers/joss.00456/10.21105.joss.00456.pdf |
| D. Casini, T. Blaß, I. Lütkebohle, B. B. Brandenburg, "Response-Time Analysis of ROS 2 Processing Chains Under Reservation-Based Scheduling," ECRTS 2019, LIPIcs 133. doi:10.4230/LIPIcs.ECRTS.2019.6 | First formal end-to-end latency model of ROS 2 executor chains; basis of later work (Blaß RTSS 2021, Tang RTSS 2020). | https://drops.dagstuhl.de/storage/00lipics/lipics-vol133-ecrts2019/LIPIcs.ECRTS.2019.6/LIPIcs.ECRTS.2019.6.pdf |
| C. Bédard, I. Lütkebohle, M. Dagenais, "ros2_tracing: Multipurpose Low-Overhead Framework for Real-Time Tracing of ROS 2," *IEEE RA-L* 7(3), 2022. | Standard method/tool for measuring message and callback latency in ROS 2. | https://arxiv.org/abs/2201.00393 |
| ISO 13849-1:2023, *Safety of machinery — Safety-related parts of control systems — Part 1: General principles for design*. | Core machine functional-safety standard (performance levels PL a–e) referenced by ISO 3691-4 and ISO 18497. | No open copy of the standard; open official guide: IFA Report 2/2017e, https://www.dguv.de/medien/ifa/en/pub/rep/pdf/reports-2019/report0217e/rep0217e.pdf |
| IEC 60204-1:2016, *Safety of machinery — Electrical equipment of machines* (stop categories 0/1/2) with ISO 13850:2015, *Emergency stop function*. | Define stop categories and e-stop requirements that all mobile-machine standards cite. | No open copy |
| ISO 3691-4:2023, *Industrial trucks — Safety requirements and verification — Part 4: Driverless industrial trucks and their systems*. | The AGV/AMR safety standard: speed monitoring, protective stops, operating modes, braking. | No open copy; open peer overview candidate: arXiv 2502.20693 ("From Safety Standards to Safe Operation with Mobile Robotic Systems Deployment") |
| IEC 61508 (2010), *Functional safety of E/E/PE safety-related systems*. | Umbrella functional-safety standard (SIL, safe state, watchdogs with independent time base). | No open copy of the standard; IEC official overview: https://assets.iec.ch/public/acos/IEC%2061508%20&%20Functional%20Safety-2022.pdf |

Strong supporting (not foundational) candidates found: Puck et al., CASE 2021 (ROS 2 real-time control latency/jitter, open copy on ResearchGate only); Kronauer et al., IEEE MFI 2021 "Latency Analysis of ROS2 Multi-Node Systems" (arXiv 2101.02074); Blaß et al., RTSS 2021 (MPG PuRe PDF); Tang et al., RTSS 2020; Macenski et al., "Robot Operating System 2: Design, architecture, and uses in the wild," Science Robotics 2022 (arXiv 2211.07752); Biesiadecki & Maimone, "Mars Exploration Rover Surface Operations: Driving" (JPL PDF) and Maimone et al. MER autonomy ICRA 2007 (JPL PDF); Clearpath Husky A200 user manual (MCU e-stop loop, relays cut motor power); ROS 2 QoS deadline/liveliness design article (design.ros2.org); ROS 2 Real-Time Working Group docs; Autoware `vehicle_cmd_gate` and control-component docs; Nav2 `velocity_smoother` and `collision_monitor` docs; `twist_mux` docs; IEC 61800-5-2 drive stop functions (manufacturer explanations, e.g. SEW, Pilz); Frontiers 2020 "A Brief Survey of Telerobotic Time Delay Mitigation".

## Existing sources in `sources/`

| File | Covers subtopics |
|---|---|
| `ros2control_2024_diff_drive_controller.md` (control.ros.org Humble user docs for diff_drive_controller) | 5 (velocity/acceleration/jerk limits), 6 (automatic stop after `cmd_vel_timeout`), 11(c) |

## Search log

| # | Query | Useful result |
|---|---|---|
| 1 | Macenski "The Marathon 2" navigation system IROS 2020 arXiv | arXiv 2003.00368 |
| 2 | "From the desks of ROS maintainers" survey … arXiv | arXiv 2307.15236; RAS 168 |
| 3 | Casini response-time analysis ROS 2 processing chains executor ECRTS 2019 pdf | Dagstuhl open PDF |
| 4 | twist_mux priority lock topics timeout ROS 2 documentation | docs.ros.org twist_mux; PAL docs |
| 5 | Nav2 collision_monitor velocity_smoother documentation stop slowdown approach polygon | Nav2 collision monitor docs; pipeline order controller → smoother → collision monitor → base |
| 6 | Puck Keller "Performance evaluation of real-time ROS2 robotic control…" | CASE 2021, IEEE 9551447 |
| 7 | Puck 2021 … arXiv | no arXiv; found Kronauer arXiv 2101.02074 |
| 8 | Astrom Murray "Feedback Systems" second edition free pdf | fbsbook.org; Caltech PDFs |
| 9 | Skogestad Postlethwaite "Multivariable Feedback Control" cascade control time scale separation pdf | Wiley 2005; no verified open copy |
| 10 | ISO 3691-4 driverless industrial trucks safety functions … | ISO pages; no open copy |
| 11 | IEC 60204-1 stop categories 0 1 2 emergency stop explanation | stop-category definitions (manufacturer/secondary) |
| 12 | Clearpath Husky A200 emergency stop architecture MCU cmd_vel timeout … | Husky A200 manual PDF; twist_mux on Husky |
| 13 | ros2_control "Architecture" controller manager realtime update_rate read update write loop | control.ros.org controller manager docs |
| 14 | Chitta ros_control generic and simple control framework ROS JOSS 2017 pdf | JOSS open PDF |
| 15 | ISO 13849-1 performance level … IFA report pdf | IFA Report 2/2017e |
| 16 | IEC 61508 functional safety overview pdf watchdog safe state … | IEC official overview PDF; ADI watchdog note |
| 17 | ISO 18497 agricultural machinery autonomous safety ISO 25119 … | ISO 18497-1/2/3:2024 pages |
| 18 | Mars rover drive command sequencing onboard fault protection … Maimone | JPL MER driving / autonomy PDFs |
| 19 | Autoware vehicle_cmd_gate emergency handler MRM … | Autoware vehicle_cmd_gate and emergency_handler docs |
| 20 | Blass Casini "A ROS 2 response-time analysis exploiting starvation freedom…" RTSS 2021 | MPG PuRe PDF |
| 21 | Kronauer "Latency analysis of ROS2 multi-node systems" | arXiv 2101.02074 |
| 22 | ros2_tracing Bédard … arXiv | arXiv 2201.00393 |
| 23 | ISO 26262 SOTIF automated driving fallback degraded mode command arbitration survey | arXiv 2011.00892 (formally verified fail-operational arbitration) |
| 24 | windup rate limiter saturation inside feedback loop anti-windup … Astrom Hagglund cascade | Advanced PID Control ch.1 (ISA); arXiv 2606.01959 anti-windup review |
| 25 | navigation2 issue 5524 closed-loop angular velocity correction … Macenski | Issue #5524 (question only; no maintainer answer captured on fetch) |
| 26 | Nav2 velocity smoother feedback OPEN_LOOP CLOSED_LOOP velocity_timeout deadband | Nav2 velocity smoother docs |
| 27 | ANSI/RIA R15.08 industrial mobile robots safety … speed separation monitoring | ANSI webstore; arXiv 2502.20693 |
| 28 | clearpath_ros2 twist_mux.yaml priorities e_stop joy_teleop … | Clearpath generators / joystick docs |
| 29 | Kelly "Mobile Robotics: Mathematics, Models, and Methods" latency delay compensation | book only; no open copy |
| 30 | ros2_control joint_limits speed_limiter diff_drive_controller … cmd_vel_timeout | diff_drive_controller source and joint-limiting docs |
| 31 | ROS 2 QoS deadline liveliness lease duration heartbeat … | ROS 2 QoS docs; design.ros2.org QoS article |
| 32 | Macenski "Robot Operating System 2: Design, architecture, and uses in the wild" | arXiv 2211.07752 |
| 33 | teleoperation time delay supervisory control Sheridan "move and wait" … | Frontiers 2020 time-delay survey |
| 34 | IEC 61800-5-2 safety functions STO SS1 SS2 SOS … | SEW / Pilz manufacturer explanations |
| 35 | motor driver brake vs coast mode H-bridge slow decay fast decay application note | decay-mode explanations (secondary; need TI/Allegro primary app note) |
| 36 | Tang "Response time analysis and priority assignment of processing chains on ROS2 executors" RTSS 2020 | doi:10.1109/RTSS49844.2020.00030; no open copy found yet |
| 37 | ISO 13855 minimum distance formula S = K x T + C … | formula and terms (secondary; standard not open) |
| 38 | Autoware control component design vehicle_cmd_gate rate limit filter jerk limit … | Autoware control design docs |
| 39 | ROS 2 real-time working group documentation PREEMPT_RT … cyclictest | ROS 2 RT WG docs; design.ros2.org realtime background |

Notes for the research step: several secondary pages found above (manufacturer blogs, calculators) are not acceptable sources under STANDARDS.md; use them only to locate primary documents (e.g. TI/Allegro driver datasheets for brake/coast, SEW/Siemens drive manuals for IEC 61800-5-2 functions, IFA report for ISO 13849). No maintainer statement on "closed-loop correction belongs in the controller, not the base" was located yet — needs a targeted search of Nav2 docs/issues and the Macenski survey.

## Gap check (2026-09-27)

Independent completeness pass (step 3 of STANDARDS.md §5). The README already answered most questions in subtopics 1–11 with cited findings; the foundational references were all downloaded or correctly marked "not downloaded" (Skogestad & Postlethwaite, Åström & Hägglund, the ISO/IEC standards) and each downloaded one is cited in the findings. The open candidate for ISO 3691-4 / R15.08 listed in this file (arXiv 2502.20693) had not been downloaded; it now is (C3-S45).

### Gaps found and how each was filled

| # | Gap | Type | How filled | New sources |
|---|---|---|---|---|
| 1 | Q2(c) when an inner yaw-rate/heading loop below the navigation layer is acceptable — only general theory cited | SCOPE question partly answered | PX4 rover rate loop (closed-loop yaw rate at the lowest layer, feed-forward first, stepwise mode unlocking, loop disabled when non-minimum-phase) and PX4 multicopter/fixed-wing cascade (integrator limits, inner-first tuning) | C3-S40, C3-S41 |
| 2 | Control timing: latency vs jitter, what is tolerable (Q8(d)) — only delay-as-phase-lag cited | Expert expectation | Cervin thesis: definitions, constant latency preferable, 10 % jitter rule of thumb, schedulability ≠ performance, output-before-update | C3-S42 |
| 3 | ROS 2 executor analysis beyond Casini 2019 (Tang, Blaß not open) | SCOPE question 8(c) | Open 2026 LITES survey: timer skipping, sink-priority rule, reaction time vs data age, measured latency reduction | C3-S50 |
| 4 | Stopping-distance / protective-field formula (Q7(d)) — was an open question | SCOPE question | SICK safety-scanner manual: ISO 13855 example formula and AGV stopping-distance and field-length formulas with supplements | C3-S46 |
| 5 | Deadman/enable buttons (Q3(b)) — only ISO 13849 list cited | SCOPE question | teleop_twist_joy source: required enable button, single zero on release, no autorepeat | C3-S44 |
| 6 | Communication loss at the motor controller (Q6(e)) and per-actuator watchdogs | SCOPE question | WPILib Motor Safety (100 ms per actuator, 125 ms system), SPARK MAX 60 ms signal loss → idle mode, status-frame periods | C3-S43, C3-S48 |
| 7 | Brake vs coast at the motor-controller level (Q7(c)) — only an H-bridge app note | SCOPE question | SPARK MAX idle-mode documentation | C3-S48 |
| 8 | Interface conventions: units/frames (Q1(c)); stamp meaning (Q6(b), twist_stamper named but not used) | SCOPE question | REP 103; twist_stamper source | C3-S52, C3-S51 |
| 9 | Minimal-risk manoeuvres and fail-operational arbitration (Q10(a)) | SCOPE question | Autoware MRM handler + emergency-stop operator (0.5 s timeout, latching, −2.5 m/s², jerk −1.5 m/s³, seeded from last command); Fu et al. five-mode degradation with hot-standby safety channel, formally verified | C3-S47, C3-S53 |
| 10 | R15.08 / mobile-robot standards landscape (Q4(d)) | SCOPE question | Belzile et al. 2025 (scope of R15.08, ISO/TS 15066 modes) | C3-S45 |
| 11 | Consistency of limits across Nav2 layers (Q5(b)) | SCOPE question | MPPI docs: default accel limits differ from the smoother's, `clamp_raw_controls` asymmetry warning, `model_dt` and odometry-rate rules | C3-S49 |
| 12 | Competition rules for this vehicle class (hardware e-stop, hardware speed governing, autonomy light, qualification checks) | Expert expectation | IGVC 2026 official rules | C3-S39 |
| 13 | Hand-back / re-seeding of the smoother (Q3(c)) | SCOPE question | Existing smoother source (C3-S20) re-read: open loop seeds from its own last output, closed loop from odometry; NaN/Inf rejection | none (C3-S20) |

### What remains
- Exact clause text of ISO 13850, IEC 60204-1, ISO 3691-4, ANSI/RIA R15.08, ISO 13855, ISO 18497/25119 and ISO 26262 (paywalled; only secondary open sources used).
- Measured end-to-end controller-to-wheel latency budget for a mobile robot (Puck et al. CASE 2021 not open).
- A source that analyses a heading-hold loop placed after a slew limiter beneath a predictive controller (only indirect evidence: PX4 designed cascade, cascade theory).
- Published fault-injection test procedures for command multiplexers (subtopic 19).
- Zero-velocity hold under load in velocity-controlled bases beyond brake idle mode and the SOS definition.
- Cervin et al., IEEE CSM 2003 paper itself (thesis used instead); Tang RTSS 2020 and Blaß RTSS 2021 only via survey.

### Search log (gap check)

| # | Query / action | Result |
|---|---|---|
| G1 | PX4-Autopilot docs `flight_stack/controller_diagrams.md`, `config_rover/` | Downloaded (C3-S40, C3-S41) |
| G2 | Cervin Henriksson Lincoln Eker Årzén "How does control timing affect performance" pdf | Lund/LUP pages HTML only; UPenn copy is Cervin's 2003 thesis (C3-S42) |
| G3 | arXiv 2502.20693 | Downloaded (C3-S45) |
| G4 | SICK safety laser scanner operating instructions protective field vehicle stopping distance | nanoScan3 I/O manual (C3-S46) |
| G5 | frc-docs `wpi-drive-classes.rst` Motor Safety | Downloaded (C3-S43) |
| G6 | ros2/teleop_twist_joy humble README + source | Downloaded (C3-S44) |
| G7 | autoware.universe `system/autoware_mrm_handler`, `autoware_mrm_emergency_stop_operator` | Downloaded (C3-S47) |
| G8 | docs.revrobotics.com SPARK MAX control-interfaces.md, operating-modes.md | Downloaded (C3-S48) |
| G9 | docs.nav2.org rolling MPPI configuration guide, tuning guide | MPPI guide downloaded (C3-S49); tuning guide had nothing on pipeline limits |
| G10 | Blaß Casini Bozhko Brandenburg RTSS 2021 pdf | No open copy; found open LITES 2026 survey (C3-S50) |
| G11 | joshnewans/twist_stamper README + source | Downloaded (C3-S51) |
| G12 | ros-infrastructure/rep rep-0103.rst | Downloaded (C3-S52) |
| G13 | arXiv 2011.00892 fail-operational safety concept | Downloaded (C3-S53) |
| G14 | IGVC 2026 official rules (repo copy of igvc.org PDF) | Copied (C3-S39) |
