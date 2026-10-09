# C4 — Controller–vehicle interface: scope map

Step 1 (Map) output. Written 2026-09-27. Nothing downloaded yet; `sources/` is empty.

## Overview

A path-tracking controller (the software that turns a planned path into velocity commands many times per second) always carries a picture of the vehicle inside it: how the vehicle moves for a given command (the *motion model*), how fast it can go and speed up (*limits*), and where it is and how fast it is moving now (*state feedback*). This topic covers how well that picture has to match the real vehicle, and what happens when it does not. The main case is sampling-based model predictive control (MPPI, "model predictive path integral": the controller simulates many randomly perturbed command sequences through the motion model and blends them, weighting each by its cost), with pure pursuit and gradient-based MPC as comparison points. The literature splits into (a) how the controller is formulated and what it assumes, (b) how the vehicle's command-to-motion response is measured and turned into a model, (c) how limits, command delay (time before a command has any effect), actuator lag (time for the vehicle to reach the commanded speed) and under-delivery (reaching less than the commanded speed) degrade tracking and how delay-aware or learned models make up for them, (d) how measured velocity is fed back into the controller, and (e) how slip on skid-steer and tracked vehicles is handled in field, agricultural and planetary robots. Product-specific material (Nav2 MPPI and Regulated Pure Pursuit, Nav2 velocity smoother, ros2_controllers, Autoware, AutoRally) comes last. Out of scope: command arbitration and where limits are applied (C3), motor-controller PID (C1), derivation of kinematic models (C2), sensor fusion (L4).

## Subtopics

| # | Subtopic | Questions | Covered already? |
|---|---|---|---|
| 1 | MPPI formulation and its assumptions about the vehicle | (a) How is MPPI derived (path integral / information-theoretic view), and what are its tuning quantities (sampling noise, temperature, horizon, time step, number of samples)? (b) What does it assume about the dynamics: that noise enters through the controls, that the model is accurate over the horizon, that the first command is applied at once? (c) How are control and state limits handled: clipping samples, cost penalties, or both? (d) How does warm-starting (shifting last cycle's command sequence forward) interact with the vehicle's actual response? | No |
| 2 | Other path trackers and what they assume (baselines) | (a) What does pure pursuit assume (kinematic unicycle/bicycle, instant velocity response), and how does its lookahead distance relate to speed, delay and stability? (b) What do Regulated/Adaptive Pure Pursuit add (curvature and proximity speed scaling)? (c) How do gradient-based MPC, Stanley, and dynamic-window methods differ in what they need from the vehicle? (d) Which surveys compare these trackers on the same vehicle, and on what metrics? | No |
| 3 | Motion models used inside controllers | (a) When is a kinematic model enough, and when do dynamic, first-order-lag or slip-aware models become necessary (speed, terrain, aggressiveness)? (b) How are learned models used in MPPI/MPC: neural-network dynamics, Gaussian-process residual/disturbance models? (c) What errors result from model mismatch (bias, oscillation, corner cutting, overshoot)? (d) How cheap must the model be to run thousands of rollouts per cycle on embedded compute? | No |
| 4 | Identifying the command-to-motion response | (a) What test inputs are used (steps, ramps, chirps, random commands) and what is measured (delay, time constant, steady-state gain, acceleration limits)? (b) How are model parameters fitted (least squares, prediction-error minimisation over multi-step horizons, Kalman-filter parameter estimation)? (c) What field protocols exist for skid-steer vehicles (e.g. DRIVE, Seegmiller/Kelly integrated prediction error)? (d) How is the identified model validated (multi-step prediction error on held-out data)? | No |
| 5 | Matching controller limits to what the chassis delivers | (a) How should velocity and acceleration limits in the controller relate to measured chassis capability, and what happens if they are higher or lower? (b) How do sampling spread and limits together decide which trajectories are even considered? (c) How do horizon length, time step, maximum speed and local map size constrain each other? (d) What do published configs for field robots (Clearpath, AutoRally, Nav2 examples) use, and how were they chosen? | No |
| 6 | Command delay, actuator lag and velocity under-delivery | (a) How do pure delay and first-order lag each affect tracking error and stability margins of pure pursuit and MPC/MPPI? (b) How is delay compensated: predicting the state forward by the delay, adding past commands to the model state, Smith predictors, first-order lag terms in the model? (c) What does persistent under-delivery (vehicle reaches only a fraction of commanded speed or turn rate) do to a predictive controller, and how is it corrected (gain terms, learned residuals, integral action)? (d) How large are the delays reported for real vehicles and actuators? | No |
| 7 | Feeding measured velocity and odometry back into the controller | (a) How do reference stacks set the controller's initial state: measured odometry, last command (open loop), or a blend? (b) What goes wrong when that feedback is noisy, low-rate or late (chatter, oscillation, false acceleration limiting)? (c) How is feedback filtered or time-aligned before use? (d) How does the choice interact with acceleration limits and warm-starting? | No |
| 8 | Slip in skid-steer and tracked path tracking | (a) How does slip change the relation between track speeds and body motion (instantaneous centres of rotation, ICR), and how does that show up as tracking error? (b) How is slip estimated online (ICR/EKF estimation, observers, visual or GNSS velocity) and fed to the controller? (c) What do agricultural, planetary and off-road field robots do (adaptive gains, learning MPC, receding-horizon estimation)? (d) How much does terrain (grass, gravel, snow) change the parameters, and how fast must adaptation be? | No |
| 9 | Robustness to model error and disturbances | (a) What do tube MPPI and robust MPPI add over plain MPPI, and when do they matter? (b) How do robust/tube MPC and learning-based MPC with constraints bound tracking error under model error? (c) What failure modes are reported when disturbances exceed the model (divergence, stuck, oscillation)? | No |
| 10 | Testing and metrics for controller–vehicle fit | (a) What metrics are used (cross-track error, heading error, speed tracking, repeatability, time to complete)? (b) What test paths and procedures are standard (straight lines, circles, figure-eights, repeated laps, step changes in speed)? (c) How are simulation results checked against hardware, and how is the sim-to-real model gap measured? | No |
| 11 | Product/software specifics | (a) Nav2 MPPI: which motion models exist, how `model_dt`, `vx_std`, `vx_max`/`wz_max` work, how the first rollout step is seeded from the measured speed, and how Humble differs from later releases (acceleration limits `ax_max`/`az_max`, `open_loop` option)? (b) Nav2 Regulated Pure Pursuit and velocity smoother: open-loop vs closed-loop feedback, limits. (c) ros2_controllers `diff_drive_controller` limit and odometry settings. (d) Autoware MPC delay/time-constant parameters and AutoRally MPPI model handling. | No |
| 12 | Command smoothness and chattering of sampling-based controllers (added in gap check) | (a) Why does MPPI produce jittery commands, and what does that do to actuators? (b) External smoothing filters vs smoothing inside the optimiser (Smooth MPPI): what does each cost (delay, limit violation)? (c) How does sampling spread trade smoothness against responsiveness? | Added 2026-09-27 |
| 13 | Delay magnitudes and delay-aware geometric trackers (added in gap check) | (a) How large are input delay and actuator lag in real vehicles? (b) How do Stanley/pure pursuit degrade with delay and speed, and how does preview/feedforward timing compensate? (c) Closed-form delay-stability limits for pure pursuit (Ollero & Heredia). | Added 2026-09-27 |
| 14 | Slip detection and compensation with independent motion sensing (added in gap check) | (a) How do planetary and field rovers detect slip by comparing wheel kinematics with visual/inertial motion estimates? (b) How is the slip vector fed into the path follower? (c) What rate limits apply? | Added 2026-09-27 |
| 15 | Online learning of model residuals during operation (added in gap check) | (a) How are residual (nominal-model error) models learned online for MPC, and how much data do they need? (b) Does a model learned in one session transfer to the next? | Added 2026-09-27 |
| 16 | Noisy vs delayed velocity feedback (added in gap check) | (a) Can the effects of feedback noise and feedback delay on a sampling-based controller be separated? (b) Which command jitter is caused by feedback and which by sampling or clamping? | Added 2026-09-27 |

## Foundational references

| Citation | Why foundational | Open copy |
|---|---|---|
| G. Williams, P. Drews, B. Goldfain, J. M. Rehg, E. A. Theodorou, "Aggressive driving with model predictive path integral control," IEEE ICRA, 2016, pp. 1433–1440. doi:10.1109/ICRA.2016.7487277 | Original MPPI algorithm on a real vehicle (AutoRally) | No open copy found (IEEE; ResearchGate request only) |
| G. Williams, P. Drews, B. Goldfain, J. M. Rehg, E. A. Theodorou, "Information-theoretic model predictive control: Theory and applications to autonomous driving," IEEE Trans. Robotics 34(6):1603–1622, 2018. doi:10.1109/TRO.2018.2865891 | Full theory of MPPI (IT-MPC), the version later implementations follow | https://arxiv.org/pdf/1707.02342 |
| G. Williams, N. Wagener, B. Goldfain, P. Drews, J. M. Rehg, B. Boots, E. A. Theodorou, "Information theoretic MPC for model-based reinforcement learning," IEEE ICRA, 2017, pp. 1714–1721. doi:10.1109/ICRA.2017.7989202 | Standard reference for MPPI with a learned (neural-network) vehicle model identified from driving data | https://homes.cs.washington.edu/~bboots/files/InformationTheoreticMPC.pdf |
| M. S. Gandhi, B. Vlahov, J. Gibson, G. Williams, E. A. Theodorou, "Robust model predictive path integral control: Analysis and performance guarantees," IEEE Robotics and Automation Letters 6(2):1423–1430, 2021. | Main reference on making MPPI robust to model error and disturbances (tube-MPPI, RMPPI) | https://arxiv.org/pdf/2102.09027 |
| J. B. Rawlings, D. Q. Mayne, M. M. Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed., Nob Hill Publishing, 2017 (5th printing 2022). | Standard graduate MPC textbook (stability, robustness, delay, estimation) | https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-5th-printing.pdf |
| R. C. Coulter, "Implementation of the pure pursuit path tracking algorithm," Tech. Rep. CMU-RI-TR-92-01, Carnegie Mellon University, 1992. | Standard description of pure pursuit, including lookahead and its limits | https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf |
| S. Macenski, S. Singh, F. Martín, J. Ginés, "Regulated pure pursuit for robot path tracking," Autonomous Robots 47:685–694, 2023. | Nav2's reference path tracker paper; states its vehicle assumptions and speed regulation | https://arxiv.org/pdf/2305.20026 |
| B. Paden, M. Čáp, S. Z. Yong, D. Yershov, E. Frazzoli, "A survey of motion planning and control techniques for self-driving urban vehicles," IEEE Trans. Intelligent Vehicles 1(1):33–55, 2016. | Most-cited survey comparing vehicle models and tracking controllers (pure pursuit, MPC, others) | https://arxiv.org/pdf/1604.07446 |
| R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012. doi:10.1007/978-1-4614-1433-9 | Standard textbook on kinematic vs dynamic vehicle models and lateral/longitudinal control, incl. actuator dynamics | No open copy |
| J. L. Martínez, A. Mandow, J. Morales, S. Pedraza, A. García-Cerezo, "Approximating kinematics for tracked mobile robots," Int. J. Robotics Research 24(10):867–878, 2005. | Origin of the experimentally identified ICR model for tracked robots used in slip-aware tracking | No open copy verified (SAGE; academia.edu upload only) |
| C. J. Ostafew, A. P. Schoellig, T. D. Barfoot, J. Collier, "Learning-based nonlinear model predictive control to improve vision-based mobile robot path tracking," J. Field Robotics 33(1):133–152, 2016. | Seminal field demonstration of MPC that learns model error (delay, slip, terrain) on an off-road skid-steer robot | https://www.dynsyslab.org/wp-content/papercite-data/pdf/ostafew-jfr16.pdf |
| E. Kayacan, S. N. Young, J. M. Peschel, G. Chowdhary, "High-precision control of tracked field robots in the presence of unknown traction coefficients," J. Field Robotics 35(7):1050–1062, 2018. | Key reference for tracked-vehicle MPC with online traction/slip estimation in the field (agriculture) | https://arxiv.org/pdf/2103.11294 |
| G. Williams, A. Aldrich, E. A. Theodorou, "Model predictive path integral control: From theory to parallel computation," J. Guidance, Control, and Dynamics 40(2):344–357, 2017. doi:10.2514/1.G001921 (added 2026-09-27 gap-fill; C4-S49) | The work that first defined MPPI (path-integral derivation, importance sampling, GPU sampling) | Journal version closed; read through the earlier preprint arXiv:1509.01149v3 (2015, different title), cited at level C |

Other strong candidates found (for step 2, not foundational): Snider 2009 CMU-RI-TR-09-08 (path-tracker comparison, open PDF at publications.ri.cmu.edu); Kanayama et al. 1990 ICRA (unicycle tracking law); Kozlowski & Pazderski 2004 AMCS (skid-steer kinematic/dynamic/motor-level control, open PDF at matwbn.icm.edu.pl — overlaps C2); Pentzer, Brennan, Reichard 2014 JFR (online ICR EKF for skid-steer prediction); Seegmiller & Kelly RSS 2014 (open PDF at roboticsproceedings.org) and Seegmiller et al. 2013 IJRR (vehicle model identification by integrated prediction error); Mandow et al. 2007 IROS (skid-steer experimental kinematics); González et al. 2013 Advanced Robotics (slip-compensated control of a tracked robot); Ostafew et al. 2016 IJRR (robust constrained LB-NMPC); Kayacan et al. 2015 IEEE/ASME TMECH (tube NMPC, tractor-trailer, arXiv 2104.02063); Baril et al. 2022 Field Robotics and the DRIVE protocol (arXiv 2506.16593); Hung et al. 2023 JFR path-following review (arXiv 2204.07319); Naveed 2023 slip tutorial (arXiv 2306.14074, unreviewed); Macenski ROSCon 2023 "On Use of Nav2 MPPI Controller" slides; Nav2 MPPI README (Humble and main branches), Nav2 velocity smoother and controller-server docs; navigation2 issue #5524 (open- vs closed-loop feedback, maintainer discussion, level D); Autoware MPC lateral controller docs (steering delay and time constant); AutoRally MPPI wiki/code.

## Search log

| # | Query / action | Useful result |
|---|---|---|
| 1 | Williams information theoretic MPC aggressive driving T-RO 2018 pdf | T-RO citation; open arXiv version found in #7 |
| 2 | Aggressive driving with model predictive path integral control ICRA 2016 Williams pdf | ICRA 2016 citation, DOI; no open PDF |
| 3 | Macenski Regulated Pure Pursuit arXiv 2023 Nav2 | arXiv 2305.20026, Autonomous Robots 2023 |
| 4 | Nav2 MPPI controller documentation motion model DiffDrive vx_std ax_max | Nav2 MPPI docs; parameter overview |
| 5 | Ostafew Barfoot learning-based nonlinear MPC path tracking field robot JFR 2016 | JFR 2016 citation; sparse-GP skid-steer MPC paper |
| 6 | Kozlowski Pazderski modeling and control of a 4-wheel skid-steering mobile robot pdf | AMCS 2004, open PDF |
| 7 | Williams information-theoretic MPC arXiv 1707.02342 | Open copy of T-RO paper |
| 8 | Gandhi Williams robust MPPI tube-MPPI arXiv | RMPPI RA-L 2021, arXiv 2102.09027 |
| 9 | Rajamani Vehicle Dynamics and Control Springer 2012 | 2nd ed. details; no open copy |
| 10 | Rawlings Mayne Diehl MPC textbook free pdf | Author-hosted PDF at UCSB |
| 11 | delay compensation model predictive control path tracking actuator time delay | Delay-augmented MPC, delay-aware robust control papers (automotive) |
| 12 | Coulter pure pursuit CMU-RI-TR-92-01 pdf | Open CMU PDF |
| 13 | Snider automatic steering methods CMU-RI-TR-09-08 | Open CMU PDF |
| 14 | Paden survey motion planning and control self-driving arXiv | arXiv 1604.07446 |
| 15 | Martinez Mandow experimental kinematics tracked robots ICR | IJRR 2005, IROS 2004/2007 |
| 16 | González … control of off-road mobile robots visual odometry slip compensation | Advanced Robotics 2013 (tracked, gravel) |
| 17 | system identification skid-steer MPC Pentzer Brennan Reichard | Pentzer JFR 2014; sparse-GP MPC skid-steer (IFAC 2017) |
| 18 | Kayacan robust tube-based decentralized NMPC tracked field experiments | Kayacan TMECH 2015; JFR 2018 tracked (arXiv 2103.11294) |
| 19 | Seegmiller Kelly enhanced 3D kinematic modeling identification slip | RSS 2014 open PDF; IJRR 2013 identification |
| 20 | Kelly Mobile Robotics Mathematics Models Methods contents | Textbook (Cambridge 2013), control chapter; no open copy |
| 21 | Fetch Nav2 MPPI README (docs.ros.org Humble) | Blocked by bot-check page; used raw GitHub instead |
| 22 | curl raw GitHub nav2_mppi_controller README + optimizer.cpp (humble, main) | Humble seeds first rollout step with measured speed, no `ax_max`; main adds `ax_max/ax_min/az_max` and `open_loop` |
| 23 | Nav2 controller server odom_topic velocity smoother open loop closed loop | Velocity smoother docs (OPEN_LOOP/CLOSED_LOOP); issue #5524 |
| 24 | "A review of path following control strategies for autonomous robotic vehicles" | Hung et al. JFR 2023 (arXiv 2204.07319); Rokonuzzaman 2021 IET review |
| 25 | Autoware MPC lateral controller input delay steer time constant | Autoware docs: `steering_tau`, input delay compensation |
| 26 | Williams IT-MPC neural network dynamics AutoRally ICRA 2017 | ICRA 2017, open PDF (Boots homepage) |
| 27 | Baril Kubelka … subarctic forests / DRIVE skid-steer identification protocol | Field Robotics 2022; DRIVE arXiv 2506.16593 |
| 28 | Burke tracked vehicle slip; Wang Low slip trajectory tracking | Tracked slip-compensation control papers (JFR 2025, agri MPC) |
| 29 | Nav2 MPPI Macenski 2023 paper / ROSCon | ROSCon 2023 slides (open PDF); no peer-reviewed Nav2 MPPI paper found |
| 30 | Kanayama stable tracking control 1990 | ICRA 1990 citation |
| 31 | arXiv 2103.11294, 2306.14074 abstract pages | Kayacan JFR 2018 details; Naveed tutorial (arXiv only) |
| 32 | Pentzer … online estimation of track ICRs pdf | JFR 31(3) 2014, DOI 10.1002/rob.21509 |
| 33 | Ostafew … pdf dsl.utias; curl checks of candidate open-copy URLs | Dynsyslab PDF live; all listed open copies return application/pdf |
| 34 | Aggressive driving MPPI pdf autorally gatech | AutoRally wiki/code; still no open PDF of ICRA 2016 |
| 35 | (gap check) "Aggressive driving with model predictive path integral control" pdf | Only IEEE Xplore (login), Scribd, ResearchGate; still no open copy |
| 36 | (gap check) Helmick … path following visual odometry Mars rover high-slip pdf | JPL author copies of Aerospace Conf. 2004 and Advanced Robotics 2006; journal version downloaded |
| 37 | (gap check) Lenain Thuilot Cariou Martinet adaptive predictive off-road path tracking | EJC 2007; INRIA host unreachable, HAL bot-check; OpenAlex marks closed → not downloaded |
| 38 | (gap check) Ollero Heredia stability path tracking pure delay; Amidi CMU-RI-TR-90-17 | No open copy of Ollero/Heredia; Amidi thesis downloaded, found little on delay, deleted |
| 39 | (gap check) Stanley controller system delay (Seiffer, Frey, Gauterin) | MDPI blocked (Access Denied); KITopen copy downloaded |
| 40 | (gap check) Kabzan learning-based MPC racing; Hewing 2020 learning-based MPC review | Kabzan via ETH Research Collection DSpace API; Hewing review behind Annual Reviews bot page → not used |
| 41 | (gap check) Smooth MPPI Kim 2022 arXiv 2112.09988 | Open arXiv PDF |
| 42 | (gap check) GitHub search navigation2 "MPPI odometry noisy" | Issue #3351, PR #5266 (maintainer discussion on motion model, jitter, open/closed loop, noise) |
| 43 | (gap check) Martínez 2005 IJRR open copy; Pentzer 2014 JFR open copy | Only academia.edu / Scribd / ResearchGate → still not downloaded |

## Gap check (2026-09-27)

Independent completeness pass (step 3). README compared against subtopics 1–11 and the foundational list; five new subtopics (12–16) added above.

**Foundational references.** 9 of 12 downloaded and used (S01, S03–S08, S11, S12). Still *not downloaded* after a second search: Williams et al. ICRA 2016 (S02; only IEEE Xplore/Scribd/ResearchGate), Rajamani 2012 (S09; no open copy), Martínez et al. IJRR 2005 (S10; academia.edu only). All three are correctly marked in the README and not cited for details.

**Existing subtopics.** 1–11 were all answered with cited findings. Thin spots found: 2(c) Stanley only named, not analysed; 6(a) no closed-form delay-stability result and few measured delay magnitudes; 7(b) noisy (vs delayed) feedback only mentioned as an open question; 8(b) online slip estimation fed to a controller covered only by traction MHE; 3(b)/6(c) no online residual-learning example outside skid-steer teach-and-repeat.

| Gap | How it was filled | Sources added |
|---|---|---|
| MPPI command chattering / smoothing (new row 12) | Smooth MPPI paper: why sampling causes chatter, costs of external filters, sampling command rates instead; Nav2 maintainer on jitter vs sampling spread; new disagreement entry | C4-S46, C4-S47 |
| Delay magnitudes and delay-aware geometric tracking (new row 13; subtopics 2c, 6a, 6d) | Stanley-with-delay study: input vs steering lag split, 0.1/0.2/0.2–0.5 s values, feedforward time equal to summed delay, speed dependence, filter-induced delay, real-car error reduction | C4-S44 |
| Slip detection and compensation with independent sensing (new row 14; subtopic 8b–c) | JPL rover: wheel vs visual odometry error, Mahalanobis slip detection, slip-compensated follower, gain-tuning order, slope results | C4-S43 |
| Online residual learning (new row 15; subtopics 3b, 6c) | GP residual MPC on a race car: dictionary size, initial data need, constant model error under aggressive driving, stale-model penalty; SMPPI identification data including MPPI-like random commands | C4-S45, C4-S46 |
| Noisy vs delayed feedback (new row 16; subtopic 7b) | Nav2 PR discussion: closed-loop snaking on a high-delay robot, maintainer's delay-vs-noise framing, jitter that persisted with ground-truth odometry and came from unclamped controls | C4-S48 |
| Nav2 Humble motion model is a pass-through (subtopic 11a) | Maintainer statement that `predict` did not model dynamics and where a model belongs | C4-S47 |

**What remains (in README Open questions).** Closed-form delay stability of pure pursuit (Ollero & Heredia 1995/2007, no open copy); slip-adaptive agricultural path tracking of Lenain et al. 2007 (not downloadable); Pentzer 2014 online ICR estimation (no open copy); a peer-reviewed study of noisy odometry in MPPI (none found); quantified effect of constant velocity under-delivery on MPPI; slip-parameter change rate on grass for small skid-steer robots.

## Targeted gap-fill (2026-09-27): velocity under-delivery

Question: what happens to MPPI or other MPC/path trackers when the vehicle systematically delivers less (or more) velocity than commanded (constant actuator gain / model–plant mismatch), and what remedies are documented with measured benefit? Also (coordinator request) add the original MPPI paper and retry an open copy of Williams et al. ICRA 2016.

| # | Query / action | Result |
|---|---|---|
| 44 | L1-adaptive MPPI robust agile control model mismatch (Pravitra) | IROS 2020, arXiv 2004.00152 downloaded → C4-S51 |
| 45 | McKinnon Schoellig learn fast forget slow ground robot changing dynamics | RA-L 2019 author PDF downloaded → C4-S50 (turn-rate commands ×0.5/×0.7/×1.2 on a 900 kg skid-steer robot); companion ECC 2019 paper checked, no gain-mismatch test, deleted |
| 46 | Maeder Borrelli Morari linear offset-free MPC; Pannocchia & Rawlings 2003 | Both closed (OpenAlex); not downloaded |
| 47 | Offset-free MPC explained (Pannocchia et al. IFAC 2015) | Open access but ScienceDirect returned an HTML bot page (deleted); Pisa ECC 2015 tutorial review found instead → C4-S52 |
| 48 | arXiv 2412.08104 (Kuntz & Rawlings) | Technical report on offset-free MPC stability under mismatch → C4-S53 (level C) |
| 49 | MPPI ground robot model mismatch adaptive / residual | ICODE-MPPI preprint (C4-S54, level C, additive disturbance, simulation); Parameter-Robust MPPI (quadrotor–payload) and meta-learned adaptation extended abstracts (Tsuchiya 2024, Levy 2025) read at abstract level, not used (no gain-mismatch quantification or workshop-only) |
| 50 | Gibson et al. ICRA 2023 multi-step dynamics (off-road MPPI) | Model-prediction paper, no controlled gain-mismatch test; not used |
| 51 | path tracking velocity mismatch longitudinal slip MPC | Mostly MDPI/Elsevier slip-compensation papers without a constant-gain experiment; Gao et al. 2023 (RL gain tuning, arXiv) not used |
| 52 | Rawlings textbook Sec. 1.5.2 re-read | Lemma 1.10 applies regardless of what generates the plant output; constrained case can keep offset → added under C4-S05 |
| 53 | Original MPPI: arXiv 1509.01149v3; Crossref for JGCD 40(2):344–357 (doi 10.2514/1.G001921) | Downloaded → C4-S49, cited as the JGCD 2017 journal version with the preprint as open copy |
| 54 | ICRA 2016 "Aggressive driving with MPPI" open copy: web search, Semantic Scholar API, OpenAlex, co-author (Drews) publications page, Google Scholar | Semantic Scholar/OpenAlex mark it closed; Drews page has no PDF link; Scholar behind bot check; only Scribd/ResearchGate. Still not downloaded |

**Found.** Offset-free MPC theory (integrating disturbance + observer removes steady offset for any mismatch that leaves the loop stable, not when input constraints are active) [C4-S05, C4-S52, C4-S53]; a controlled ground-robot experiment with artificially scaled turn-rate commands showing large lateral error with a fixed model and quick recovery with online adaptation [C4-S50]; MPPI failures under a 40% actuator-power loss in simulation, fixed by an L1 adaptive inner loop [C4-S51]; persistent MPPI offset under persistent disturbance and a 69% error reduction from residual learning (preprint, additive disturbance) [C4-S54]; MPPI's own modelling assumptions (control-affine dynamics, noise only in actuated channels, known dynamics) [C4-S49].

**What remains.** No source measured MPPI tracking error on a ground vehicle as a function of a constant linear-speed or turn-rate under-delivery fraction; no source compared fixed gain correction, online gain/disturbance estimation, learned residuals and adaptive inner loops on the same vehicle and gain error; the ground-robot gain test [C4-S50] covers turn rate only and uses gradient-based MPC. Williams et al. ICRA 2016 still has no open copy.
