# L5 — Time synchronization

| | |
|---|---|
| **Question** | How do robots know *when* each measurement was taken, on one common time base, and what happens to fusion and control when they get it wrong? |
| **Covers** | Clock models and time scales, the effect of timing error and latency, delayed and out-of-sequence measurements, NTP/chrony, PTP, GNSS PPS time, how sensors and drivers stamp data, temporal calibration, time in ROS 2, and how timing is measured and checked, plus the time features of specific devices. |
| **Not covered** | Sensor fusion configuration (L4) and command-pipeline latency budgets (C3). |
| **Status** | Verified |
| **Last updated** | 2026-09-28 |

Page numbers in citations are PDF page numbers unless a printed page is given ("p. 1284"). For text and code files the citation gives a section, a symbol or a line number.

## Summary
- A timing error turns directly into a geometric error: for a moving robot the error is about speed × time error, plus (distance to the object) × turn rate × time error. A robot turning at 90 deg/s and looking at an object 10 m away gets a 15.7 cm projection error from a 10 ms timing error. [L5-S14, p. 3, eq. 10] [L5-S11, p. 1]
- Stamping data "on arrival" at the host is the weakest method. Buffering and scheduling on a loaded non-real-time OS can add jitter of hundreds of milliseconds, and FTDI USB-serial chips hold small packets for up to 16 ms by default. Hardware timestamping (timers, trigger lines, PPS) is the only way to get deterministic timing. [L5-S11, p. 1] [L5-S14, pp. 1–2] [L5-S39, p. 7]
- Typical accuracy of each clock-alignment method, as reported by its source:
  - NTP: "a few milliseconds" over the 1991 Internet, and "a few hundred microseconds" for clients on fast LANs (NTPv4).
  - chrony: "sub-microsecond" accuracy "might be possible" with NIC hardware timestamping.
  - PTP: about 100 ns with simple hardware.
  - GNSS timing receiver time pulse: 5 ns (1-sigma).
  [L5-S01, p. 1] [L5-S02, section 1] [L5-S24, FAQ 2.7] [L5-S05, p. 73] [L5-S31, section 1.2]
- When a device cannot be synchronized, its clock can still be mapped to host time from arrival or round-trip timing, with the clock skew modelled. Olson's passive method uses arrival times only and keeps the lowest-latency samples; TICSync and VersaVIS clock translation use two-way (request/response) exchanges. Accuracy is then sub-millisecond, or about ±0.2 ms over USB. [L5-S11, pp. 3–5] [L5-S17, pp. 1, 7] [L5-S20, pp. 6, 11]
- Two remaining sources of error:
  - A constant offset between sensors can be estimated from motion data: to within ±0.2 ms in Kalibr's batch method, and online in VIO. [L5-S12, p. 6] [L5-S18, p. 7]
  - A measurement that arrives late must be fused at its true time, by re-processing from a stored history, by extrapolation, or with an exact OOSM (out-of-sequence measurement) update. Otherwise it corrupts the estimate. [L5-S09, p. 2] [L5-S15, p. 2] [L5-S36, `smooth_lagged_data`]
- In ROS 2, `header.stamp` should be the acquisition time of the data. tf2 keeps 10 s of transforms by default, interpolates between stored stamps, and refuses to extrapolate beyond them. ROS time follows the system clock unless `use_sim_time` is set. [L5-S35, `Image.msg`, `LaserScan.msg`] [L5-S32, `buffer_core.hpp` l. 73, `cache.cpp` `findClosest`] [L5-S13, section "ROS Time"]

## Foundational references
| ID | Reference | Why it is foundational |
|---|---|---|
| L5-S01 | Mills, "Internet time synchronization: the Network Time Protocol", IEEE Trans. Commun., 1991 | The seminal NTP paper: four-timestamp offset and delay estimation, minimum-delay filtering, and the clock discipline loop. |
| L5-S02 | Mills et al., RFC 5905 (NTPv4), IETF, 2010 | The current normative NTP standard. |
| L5-S03 | Mills, "Adaptive hybrid clock discipline algorithm for NTP", IEEE/ACM ToN, 1998 | Introduced the hybrid PLL/FLL clock discipline and the Allan-intercept argument. |
| L5-S04 | Lamport, "Time, clocks, and the ordering of events in a distributed system", CACM 21(7), 1978 | Defines event ordering and physical clock synchronization bounds in distributed systems. |
| L5-S49 | Cristian, "Probabilistic clock synchronization", Distributed Computing, 1989 (not downloaded; described via L5-S11, p. 2) | Remote clock reading by round-trip timing. Olson describes most clock synchronization schemes as variants of Cristian's algorithm. [L5-S11, p. 2] |
| L5-S05 / L5-S50 | IEEE Std 1588-2019 and Eidson's 2006 book (not downloaded); Eidson's IEEE 1588 tutorial (NIST-hosted, 2005) used as the open proxy | The PTP standard. The tutorial presents its offset/delay mechanism, clock types and practical accuracy. |
| L5-S06 | Mogul et al., RFC 2783 PPS API, IETF, 2000 | The kernel interface through which PPS edges are timestamped. |
| L5-S07 | Sullivan, Allan, Howe, Walls (eds.), NIST TN 1337, 1990 | Standard reference on clock and oscillator characterization (Allan variance, drift). |
| L5-S51 | Bar-Shalom 2002, and Bar-Shalom, Chen, Mallick 2004, IEEE TAES (not downloaded) | The exact and multistep out-of-sequence-measurement (OOSM) Kalman updates. |
| L5-S09 | Larsen et al., "Incorporation of time delayed measurements in a discrete-time Kalman filter", IEEE CDC, 1998 | The standard comparison of delayed-measurement methods, plus the extrapolation method. |
| L5-S52 / L5-S10 | Skog & Händel, "Time synchronization errors in loosely coupled GPS-aided INS", IEEE T-ITS, 2011 (full text not downloaded); the paper's abstract was read in Skog's 2007 licentiate thesis summary | The reference analysis of GNSS–IMU time offset in vehicle navigation. |
| L5-S11 | Olson, "A passive solution to the sensor synchronization problem", IROS, 2010 | The standard robotics method for mapping device clocks to host time from arrival times. |
| L5-S12 | Furgale, Rehder, Siegwart, "Unified temporal and spatial calibration for multi-sensor systems", IROS, 2013 | Continuous-time batch estimation of time offset together with extrinsics (Kalibr). |
| L5-S53 / L5-S54 | Li & Mourikis, IJRR 2014 (author copy downloaded in the gap check); Kelly & Sukhatme, ISER 2010 (not downloaded) | Online filter-based time-offset estimation with an identifiability analysis, and general curve-registration temporal calibration. |
| L5-S13 (with L5-S32, L5-S33, L5-S34) | Open Robotics, "Clock and Time" ROS 2 design article; with the tf2 source and documentation and the message_filters documentation (Humble) | The specification of ROS 2 time sources and `use_sim_time`, and of how tf2 and message_filters use message stamps. |
| L5-S55 | Vig, "Quartz Crystal Resonators and Oscillators for Frequency Control and Timing Applications — A Tutorial", Rev. 8.5.3.6, 2007 | The standard tutorial reference on crystal oscillator types (XO, TCXO, OCXO) and their stability, aging and temperature behaviour. |
| L5-S56 | IS-GPS-200N, GPS space segment / user segment interface specification, 2022 | The primary specification of GPS time, its relation to UTC, leap seconds and the broadcast week number. |

## Findings

### 1. Clock models and time scales
- A clock's time error grows from four terms: the initial synchronization error To, the frequency offset (Δf/f)·t, half the frequency drift ½D·t², and a noise term σx(t). This is written as ΔT = To + (Δf/f)·t + ½D·t² + σx(t). [L5-S08, PDF p. 29 (printed p. 19), eq. 16]
- Without periodic re-syntonization (frequency recalibration), frequency aging can become the largest contributor to clock error for many frequency sources, for example quartz crystal and rubidium oscillators. [L5-S08, PDF p. 29 (printed p. 19)]
- Linear frequency drift eventually dominates all time uncertainties in clock models, even after correction, and drift matters particularly for quartz oscillators. [L5-S07, PDF p. 15 (p. TN-4)]
- The Allan variance is half the mean squared difference of consecutive fractional-frequency averages: σy²(τ) = 1/(2(M−1)) Σ[yi+1 − yi]². It is the standard time-domain stability measure. [L5-S08, PDF p. 24 (printed p. 14), eq. 6]
- The Allan intercept is the averaging time at which the stability curve turns. Below it, more averaging improves accuracy; above it, more averaging degrades accuracy. NTP's time constants are designed around it. [L5-S03, p. 8]
- Common computer oscillators may be frequency-accurate to only 0.01 percent and vary by several ppm with normal room-temperature changes. [L5-S01, p. 11]
- A battery-backed room-temperature quartz clock "may drift as much as a second per day". [L5-S01, p. 1]
- A SICK LMS151 laser scanner lost 92 seconds against GMT over a 6-day office test. After removing the first-order skew, temperature-correlated offset swings of over 30 ms remained. [L5-S17, p. 1, Fig. 1]
- Olson models a sensor clock as running fast or slow relative to the host by rate errors α1, α2. For a typical sensor these are "quite small: a few percent". Inexpensive microcontrollers with RC oscillators can reach much larger values. [L5-S11, pp. 3–4]
- The Xsens MTi-600 internal clock has "an accuracy of about 10 ppm". [L5-S40, p. 37, "Clock Bias Estimation function"]
- Crystal oscillators come in three main categories, by how they handle the crystal's frequency-vs-temperature curve:
  - XO (plain crystal oscillator): no temperature correction.
  - TCXO (temperature-compensated): a temperature sensor drives a correction. Analog TCXOs give "about a 20X improvement" over the bare crystal.
  - OCXO (oven-controlled): the crystal is held in an oven at its zero-slope temperature, giving a ">1000X improvement".
  [L5-S55, PDF p. 31 (slide 2-6)]
- Vig's accuracy hierarchy, including environmental effects (e.g. −40 °C to +75 °C) and one year of aging: XO 10⁻⁵ to 10⁻⁴ (typical use "computer timing"), TCXO 10⁻⁶, OCXO 10⁻⁸. [L5-S55, PDF p. 33 (slide 2-8)]
- Vig's comparison table gives temperature stability over −55 °C to +85 °C of 5 × 10⁻⁷ for a TCXO and 1 × 10⁻⁹ for an OCXO, and aging of 5 × 10⁻⁷ and 5 × 10⁻⁹ per year. [L5-S55, PDF p. 237 (slide 7-1)]
- GPS time is a continuous time scale with its zero at midnight of 5/6 January 1980 (UTC(USNO)). UTC is corrected with integer leap seconds, so the two scales differ. [L5-S56, section 3.3.4, PDF p. 56]
- The GPS control segment keeps GPS time within one microsecond of UTC (modulo one second). The broadcast data relate GPS time to UTC(USNO) within 20 ns (one sigma). [L5-S56, section 3.3.4, PDF p. 56]
- The legacy navigation message sends only the ten least significant bits of the week number, "a modulo 1024 binary representation" of the GPS week. [L5-S56, section 20.3.3.3.1.1, PDF p. 112]
- When a leap second is added, "unconventional time values of the form 23:59:60.xxx are encountered". Some equipment approximates UTC by decrementing its running count within several seconds after the event, and user equipment "must consistently implement carries or borrows into any year/week/day counts". [L5-S56, section 20.3.3.5.2.4 b, PDF p. 147]
- Leap smearing is the alternative to stepping: a chrony server can suppress the leap status and slew the served time slowly, so clients never see a leap second. In chrony's recommended example the smear takes 62500 s (about 17.36 h) and the frequency offset peaks at 32 ppm. Clients must use only servers that smear in exactly the same way. [L5-S26, `leapsecmode`]
- Linux clocks:
  - `CLOCK_REALTIME` is settable wall-clock time. It jumps when the time is set, is slewed by NTP, and counts seconds since 1970-01-01 UTC ignoring leap seconds. [L5-S29, DESCRIPTION]
  - `CLOCK_MONOTONIC` is not settable and never goes backwards, but it is affected by frequency adjustments. [L5-S29, DESCRIPTION]
  - `CLOCK_MONOTONIC_RAW` is not subject to frequency adjustments. `CLOCK_BOOTTIME` also counts suspend time. `CLOCK_TAI` counts leap seconds. [L5-S29, DESCRIPTION]
- Unix time has no 23:59:60. When a leap second is inserted, the system clock ends up one second ahead of UTC and must be stepped or slewed back. chrony's default, where the system supports leap seconds, is for the kernel to step the clock back one second at 00:00:00 UTC. [L5-S26, `leapsecmode`]
- A GNSS timing receiver's time pulse can be aligned to either GPS time or UTC. [L5-S30, section 1]
- In distributed systems, "happened before" is only a partial ordering of events. Lamport gives an algorithm to extend it with logical clocks, and a bound on how far synchronized physical clocks can drift apart. [L5-S04, p. 558 (PDF p. 1), abstract]

### 2. Effects of timing error and latency on fusion and control
- Syncline model: the error caused by a synchronization error μsync is approximately ‖v‖·μsync + d·‖ω‖·μsync. Here v is the robot's linear velocity, ω its angular velocity and d the distance to the object measured. [L5-S14, p. 3, eq. 10]
- If there is no relative motion between robot and object, there is no synchronization-induced error. [L5-S14, p. 3]
- The first term of the Syncline error is the distance travelled between the true sample time and the stamp. The second is the orientation change multiplied by the object distance, a small-angle approximation. [L5-S14, p. 3]
- Syncline's point: fusion accuracy is limited either by sensor noise or by synchronization quality. For slow platforms (surface vessels) sensor noise dominates. For fast platforms (UAVs), a better sensor does not help when synchronization is only good to hundreds of milliseconds. (Domain: UAV, USV, AUV and car georeferencing.) [L5-S14, p. 1]
- Olson's worked example: rotating at 90 deg/s while observing an object 10 m away, a 10 ms synchronization error gives a 15.7 cm projection error. [L5-S11, p. 1]
- A GNSS–IMU timing error that the filter does not model causes an increased error covariance and a bias in the estimated forward acceleration. [L5-S10, PDF p. 15, Paper E summary]
- If the timing error is added to the estimation problem, simulation gives "almost perfect time synchronization", and the timing error is observable for all trajectories that include turns or non-zero accelerations. [L5-S10, PDF p. 15]
- Latency causes three filter failures:
  - A latent measurement handled naively in a highly dynamic setting can be rejected by residual editing (outlier gating) even though it is accurate.
  - Latent measurements processed repeatedly accumulate error, which can lead to filter divergence.
  - Measurements processed in an order that differs from their sampling times are out-of-sequence measurements (OOSMs).
  [L5-S15, p. 2]
- In a lunar-landing simulation, vehicle state uncertainty was most affected by camera timing jitter during periods of little dynamic activity. The authors conclude that accurate timing hardware and characterization matter more than better processing strategies. (Domain: planetary lander.) [L5-S15, p. 27]
- In feedback control, a pure time delay τ has unit gain but adds a phase lag of ωτ that grows linearly with frequency. [L5-S23, PDF p. 21 (p. 10-20)]
- A delay τc = (π + θ0)/ω0 makes the loop transfer function reach −1 at the crossover frequency ω0, so signals return in phase and an oscillation may result. [L5-S23, PDF p. 5 (p. 10-4)]
- The delay margin is the smallest time delay that makes a feedback system unstable. For loops whose gain has several high-frequency peaks, it is a more relevant measure than the phase margin. [L5-S23, PDF p. 18 (p. 10-17)]

### 3. Delayed and out-of-sequence measurements in estimators
- Larsen et al. compare four ways to fuse a delayed measurement in a Kalman filter:
  - Augment the state with past states: optimal, but only practical for delays of a few samples.
  - Recalculate the filter through the delay period: optimal, but costly.
  - Alexander's correction term, added when the measurement arrives.
  - Extrapolate the measurement to the present time and compute a gain for it (their new method).
  [L5-S09, PDF p. 2 (p. 3972)]
- In their simulations, no method fully compensated for the delay: all normalized error variances were above one. The optimal recalculation and the modified Alexander method gave identical results. [L5-S09, PDF p. 6 (p. 3976)]
- "In-sequence processing" keeps a history of measurements, states and covariances, and chronologically re-processes everything after an OOSM arrives. [L5-S15, p. 2]
- Bar-Shalom (2002) gave an exact update for an OOSM within the current sampling interval. Extending it to older measurements needs a smoothing-like re-processing. [L5-S15, p. 2]
- Fixed-lag-smoother and reprocessing approaches produce an optimal estimate that lags behind real time and carry significant computational overhead. [L5-S15, pp. 2–3]
- Brouk and DeMars instead treat the latency as a filter state, and add a "temporal measurement update" that accounts for uncertainty in the acquisition time. [L5-S15, pp. 1, 3]
- OOSMs arise even on deterministic, time-triggered networks when sensors have different cycle times. Buffering until all older measurements have arrived can compete with advanced OOSM algorithms when the cycle times are integer multiples (a = n·b) and the measurements can be synchronized. (Domain: automotive driver assistance, Volkswagen.) [L5-S16, p. 10]
- robot_localization offers `smooth_lagged_data`: on receiving data older than the last filter update, it reverts to the last state before that measurement and re-processes all measurements up to the current time. [L5-S36, `state_estimation_nodes.rst` `~smooth_lagged_data`]
- `history_length` sets how many seconds of state and measurement history are kept. The documentation says it should be at least as large as the lag. [L5-S36, `~history_length`]
- With `smooth_lagged_data` off, robot_localization skips the prediction step for a measurement older than the last one (Δt ≤ 0) and applies it as a correction to the current state. [L5-S36, `filter_base.cpp` `processMeasurement`]
- If the history is too short to revert to a lagged measurement's time, robot_localization logs a debug error and continues. [L5-S36, `ros_filter.cpp` `integrateMeasurements`]
- Factor-graph smoothing, the main alternative to filtering: Indelman et al. formulate aided inertial navigation as a factor graph in which multi-rate, asynchronous and possibly delayed measurements are added "in a natural way". Their incremental smoother decides how many past states to recompute at each step, and so acts as an adaptive fixed-lag smoother. [L5-S57, p. 1]
- In their formulation a delayed GPS measurement taken at time tl but received at tk > tl becomes a factor attached only to the navigation node at tl. Delayed measurements "can be incorporated into factor graph as easily as any other measurement", whereas they need special care in filters. [L5-S57, pp. 2, 4]
- Indelman et al. note that keeping a buffer of past navigation solutions in a filter "produces only an approximated solution". Their own results come from simulation (IMU, GPS and stereo vision), compared with full batch optimisation and an EKF. [L5-S57, p. 1]

### 4. Network clock synchronization: NTP and chrony
- NTP offset and delay from four timestamps (client send T1, server receive T2, server send T3, client receive T4):
  - offset θ = ½[(T2−T1)+(T3−T4)]
  - round-trip delay δ = (T4−T1)−(T3−T2)
  [L5-S02, section 8] [L5-S01, p. 5]
- The true offset lies within θ ± δ/2, so the half round-trip delay bounds the error. [L5-S01, p. 5]
- Asymmetric network delay can make a clock appear stable to nanoseconds while being off by milliseconds. Half the root delay is the maximum error from asymmetry. [L5-S24, FAQ 7.2]
- NTP keeps the last eight (offset, delay) samples and uses the one with the lowest delay, because the best offset samples occur at the lowest delays. [L5-S01, pp. 9–10]
- A phase-lock loop (PLL) usually works better when network jitter dominates, and a frequency-lock loop (FLL) better when oscillator wander dominates. NTPv4 combines the two. [L5-S03, p. 2]
- With a 1024 s loop time constant, the NTP PLL has a 53-minute rise time for a time step and a 5% overshoot. [L5-S03, p. 3]
- Reported NTP accuracy:
  - Internet (1991): "a few milliseconds" throughout most of the Internet. [L5-S01, p. 1]
  - LAN and Internet (1998): better than a millisecond in LANs and better than a few tens of milliseconds over the Internet. [L5-S03, p. 1]
  - NTPv4: primary servers within a few tens of microseconds; secondary servers and clients on fast LANs within a few hundred microseconds. [L5-S02, section 1]
- RFC 5905 defines a step threshold STEPT of 125 ms. Larger offsets step the clock and reset all associations. Offsets above the panic threshold PANICT of 1000 s should make the program exit. [L5-S02, sections 11.2.3, 11.3]
- A step is applied only after the offset has exceeded STEPT for the stepout interval WATCH (900 s). This resists clock steps during extreme network congestion. [L5-S02, section 11.3]
- chrony slews the clock by default. `makestep` allows steps, e.g. "makestep 0.1 3" steps when the offset exceeds 0.1 s during the first three updates only. [L5-S26, `makestep`] [L5-S24, FAQ 3.4]
- chrony's documentation says it is usually desirable to step only at boot, "before starting programs that rely on time advancing monotonically forwards". [L5-S26, `makestep`]
- In chrony's simulated client tests (stable clock, 10 μs network jitter), accuracy was 35 ± 8 μs for chrony, 234 ± 46 μs for ntp and 857 ± 226 μs for openntpd. At 1.0 ms jitter, chrony and ntp were similar: 475 ± 93 μs vs 454 ± 94 μs. [L5-S25, "Performance", Test 1]
- With a network connection available only 30 minutes per day, chrony's mean error was about 7–26 ms, while ntp's was about 0.6–1.1 s. [L5-S25, Test 3]
- `hwtimestamp` makes the network card (NIC) stamp NTP packets with its own clock, avoiding kernel and driver queueing delays. It needs Linux 3.19 or newer and NIC support, checked with `ethtool -T`. [L5-S26, `hwtimestamp`]
- With local hardware timestamping, good switches, interleaved mode and short polling, chrony says "a sub-microsecond accuracy and stability of a few tens of nanoseconds might be possible". [L5-S24, FAQ 2.7]
- For best stability, chrony advises disabling CPU frequency scaling and Energy-Efficient Ethernet, and prioritizing NTP packets in switches. [L5-S24, FAQ 2.7]
- Harrison and Newman argue that NTP is undesirable for robots: it can take many hours to synchronize, and its frequency adjustments cause timestamp inconsistencies. [L5-S17, p. 1]

### 5. Precision Time Protocol (IEEE 1588) and Linux hardware timestamping
- IEEE 1588 aims for sub-microsecond synchronization of clocks in localized networked measurement and control systems. [L5-S05, p. 10]
- PTP offset and delay: offset = (MS_difference − SM_difference)/2 and one-way delay = (MS_difference + SM_difference)/2. MS is the master-to-slave Sync difference and SM the slave-to-master Delay_Req difference. [L5-S05, p. 23]
- These formulas assume a symmetric path. The tutorial's asymmetric example yields 55 minutes instead of the true 60. [L5-S05, pp. 23–24]
- Hardware-assisted timestamping at the MAC/PHY is "potentially the most accurate". Boundary clocks reduce timing fluctuations inside network components. [L5-S05, pp. 70–72]
- In practice, about 100 ns accuracy is achievable with 2 s updates, inexpensive oscillators, compact lightly loaded networks and simple PI servos. Below 20 ns requires some combination of faster sampling, better oscillators, boundary clocks, more sophisticated statistics and servo algorithms, and careful control of the environment, especially temperature. [L5-S05, p. 73]
- The slave clock servo is typically a PI controller acting on the computed offset. [L5-S05, p. 79]
- linuxptp tools:
  - `ptp4l` implements the PTP Boundary, Ordinary and Transparent Clock for Linux. [L5-S27, `ptp4l.8` NAME/DESCRIPTION]
  - `ptp4l` defaults to the delay request-response (E2E) mechanism and to hardware timestamping. [L5-S27, `ptp4l.8` options `-E`, `-H`, `time_stamping`]
  - `ptp4l` has a `delayAsymmetry` setting for known path asymmetry. [L5-S27, `ptp4l.8` `delayAsymmetry`]
  - `phc2sys` synchronizes two or more clocks, typically the system clock to a PTP hardware clock (PHC) that ptp4l keeps synchronized. [L5-S27, `phc2sys.8` DESCRIPTION]
  - `ts2phc` synchronizes PHCs to external timestamp signals such as a 1-PPS. It can take time of day from a GPS's NMEA RMC sentence. [L5-S27, `ts2phc.8` DESCRIPTION, `-s`]
  - ts2phc's `ts2phc.nmea_delay` (default 0 ns) gives the minimum expected delay of the NMEA RMC messages. It needs to be set when the maximum NMEA delay can exceed 1 s (or the pulse width, when both PPS edges are timestamped), so that NMEA timestamps are assigned to the correct PPS pulse. [L5-S27, `ts2phc.8` `ts2phc.nmea_delay`]
- Linux socket timestamps:
  - `SOF_TIMESTAMPING_RX_HARDWARE` takes receive timestamps from the network adapter. [L5-S28, `timestamping.rst` 1.3.1]
  - `SOF_TIMESTAMPING_RX_SOFTWARE` takes them "just after a device driver hands a packet to the kernel receive stack". [L5-S28, `timestamping.rst` 1.3.1]
  - `SO_TIMESTAMP` reports "not necessarily monotonic" system time. [L5-S28, `timestamping.rst` section 1]
- chrony does not implement PTP. It can use a NIC's PHC as a reference clock or for NTP hardware timestamping, and can run NTP over PTP transport. [L5-S24, FAQ 2.14]
- chrony's view: PTP's advantage is hardware support in switches and NICs, and "if [NTP] had the same support as PTP, it could perform equally well". [L5-S24, FAQ 2.14]
- ptp4l includes gPTP (IEEE 802.1AS-2011) options, such as `neighborPropDelayThresh` and 802.1AS-capable checks. [L5-S27, `ptp4l.8`]
- ptp4l's PI servo gains are derived from the sync interval. Unless set, the scale constants are 0.7 (proportional) and 0.3 (integral) with hardware timestamping, and 0.1 and 0.001 with software timestamping. [L5-S27, `ptp4l.8` `pi_proportional_scale`, `pi_integral_scale`]
- gPTP (IEEE 802.1AS) is a profile, or subset, of IEEE 1588. It allows only the peer-to-peer delay mechanism, and no transparent clocks, only ordinary and boundary clocks. In the peer-to-peer case every node runs its own servo. [L5-S58, PDF p. 5, section 1] [L5-S59, pp. 1–2]
- Frankó and Hollósi model each gPTP hop as another PI loop in series. Each added hop between grandmaster and slave increases overshoot and "significantly" decreases synchronization accuracy, and PI tuning trades lower overshoot for longer settling time without removing the error. They confirmed this with measurements on embedded devices. [L5-S59, pp. 1, 3]
- AUTOSAR's time synchronization over Ethernet uses 802.1AS mechanisms with automotive restrictions. Because ECU roles are known in advance and the network is static, choosing the time master during operation with the best master clock algorithm (BMCA) is not required, and the protocol supports neither BMCA nor Announce and Signaling messages. For clock accuracy it refers to IEEE 802.1AS Annex B.1.2. [L5-S58, PDF pp. 5–6, sections 1, 1.2.2, 1.2.3]

### 6. GNSS time and PPS disciplining
- The PPS API captures an "assert" or "clear" timestamp as soon as possible after a signal edge, usually in an interrupt. Software compares it with the received time code to find the offset between the system clock and the time source. [L5-S06, section 1]
- The PPS API allows fixed offsets to be applied to captured timestamps. [L5-S06, section 3.2]
- A PPS signal is typically wired to a serial port's DCD pin or to a GPIO. The Linux PPS subsystem timestamps each pulse for user space. Combined with NTPD it gives "sub-millisecond synchronisation to UTC". [L5-S28, `pps.rst` Overview]
- A PPS reference clock does not carry the full time. A second source (NMEA or NTP) must identify which second each pulse marks. When that source is another refclock, chrony requires the offset between the two to be below 0.4 s. [L5-S24, FAQ 3.9] [L5-S26, `refclock PPS`]
- NMEA sentences commonly arrive with a large delay relative to the pulse. chrony's FAQ example shows an NMEA source offset of +504 ms that must be corrected with the `offset` option. [L5-S24, FAQ 3.9]
- Timing from serial message arrival alone "is accurate to milliseconds". The PPS-based source is "much more accurate". [L5-S24, FAQ 2.13]
- gpsd's socket refclock can beat a raw PPS refclock if gpsd applies the receiver-reported "sawtooth" (quantization) correction. [L5-S24, FAQ 2.13]
- A GNSS time pulse has three error parts:
  - a constant delay from antenna cable and receiver, removed by a cable-delay setting;
  - pulse-to-pulse quantization, removed with the UBX-TIM-TP quantization-error message;
  - multipath and ionospheric error.
  [L5-S30, section 2.1]
- The u-blox 6 time pulse is derived from a 48 MHz clock, which causes jitter. [L5-S30, section 1.2.2]
- For the u-blox LEA-6T, u-blox measured a 6.7 ns pulse deviation. The datasheet gives an RMS of 30 ns without and 15 ns with quantization compensation. [L5-S30, sections 2.1, pp. 5–7]
- The ZED-F9T-10B timing module specifies 5 ns (1-sigma) time-pulse accuracy in absolute mode, 2.5 ns in differential mode and ±4 ns jitter. u-blox advises measuring and compensating the whole antenna-to-output propagation delay. [L5-S31, section 1.2]
- For holdover, u-blox suggests an oven-controlled oscillator (OCXO) instead of a temperature-compensated one (TCXO). [L5-S30, section 2.3.2]
- A GNSS receiver whose GPRMC status is Void (no fix) may keep producing PPS from its internal clock. [L5-S43, p. 52]

### 7. How sensors and drivers stamp data: device time vs host arrival time
- Common synchronization primitives on commercial sensors:
  - 1PPS input with the sensor's own timestamps;
  - network synchronization (PTP or NTP);
  - a time-of-validity output pulse that the host timestamps;
  - an external trigger input.
  [L5-S14, pp. 1–2]
- Timestamping methods compared:
  - Hardware timer timestamping is the only deterministic method, limited by the timer clock's frequency and drift.
  - GPIO interrupt timestamping is not deterministic on Linux, because interrupts can be disabled during system calls.
  - Timestamp-on-arrival is the fallback when there is nothing better.
  [L5-S14, p. 2]
- Arrival stamping suffers from deep buffers and buffer-flushing logic in USB-serial converters, data acquisition boards, and non-real-time operating systems, which give no upper bound on buffering time. [L5-S11, p. 1]
- FTDI chips send data to the PC when the 64-byte buffer is full, when an RS232 status line changes, when an enabled event character arrives, or when a latency timer expires. The timer's default is 16 ms, so a single character takes 16 ms to arrive, on top of the serial transfer time. [L5-S39, p. 7]
- The FTDI latency timer can be set from 1 to 255 ms in 1 ms steps. [L5-S39, p. 8]
- On Linux the FTDI latency timer can be reduced to 1 ms. [L5-S11, p. 1, footnote]
- Olson's passive method estimates the sensor-to-host clock offset as the maximum of (sensor time − host arrival time). Latency is always positive, so the lowest-latency sample gives the tightest bound. [L5-S11, p. 3, eqs. 3–4]
- With clock drift, each observation's bound is loosened by the maximum rate error. A two-pass algorithm computes the result in O(N) time, and the causal (online) variant costs O(1) per observation. [L5-S11, pp. 4–5]
- Olson's method is proven never to be worse than naive arrival stamping. [L5-S11, p. 5, Claim 2]
- In synthetic tests with up to 0.5 s random latency, naive stamping averaged 0.25 s error, which Olson's method substantially reduced. [L5-S11, p. 6, Fig. 6]
- Olson's method was used on MIT's DARPA Urban Challenge vehicle to put 12 SICK lidars, a Velodyne HDL-64E, 15 radars, an Applanix IMU/GPS and embedded microcontrollers on one time base. (Domain: road vehicle.) [L5-S11, p. 2]
- TICSync learns the offset and skew between two clocks from two-way timing. Each update costs O(1), and it gives probabilistic error bounds. [L5-S17, p. 1]
- TICSync typically reaches better-than-millisecond accuracy within a few seconds. Over a 162 ms round-trip path the offset error settled near 35 μs, limited by clock granularity. [L5-S17, pp. 1, 7]
- VersaVIS translates microcontroller time to host time with an onboard EKF that estimates clock skew and offset from periodic request/response exchanges, assuming symmetric delay. [L5-S20, p. 6, eqs. 1–2]
- In VersaVIS tests, the raw residual oscillated at ±5 ms from USB jitter, and the EKF smoothed it to ±0.2 ms clock-translation accuracy after about 60 s. [L5-S20, p. 11]
- Scanning sensors: a LaserScan's header stamp is the acquisition time of the first ray, and `time_increment` gives the time between measurements so that points can be placed along a moving robot's path. [L5-S35, `LaserScan.msg`]
- KITTI gives three Velodyne timestamps per spin (start, end, and facing forward), because the scanner has a "rolling shutter". [L5-S21, p. 2]
- Hardware triggering, KITTI: a reed contact on the spinning Velodyne triggers the cameras when the lidar faces forward. [L5-S21, p. 4]
- Hardware triggering, VersaVIS: a microcontroller triggers cameras and IMU and records hardware-timer timestamps. It triggers each camera half an exposure early so that the mid-exposure times line up. [L5-S20, pp. 4–5]
- The middle of the exposure is the ideal point to timestamp an image. The estimated camera–IMU offset grew with exposure time at slope 0.498, against the theoretical 0.5. [L5-S12, pp. 5–6]
- Nikolic et al. call mid-exposure stamping "an established fact in photogrammetry". Their FPGA sensor unit shifts each camera trigger to account for exposure time, so that mid-exposure lines up with IMU sampling. [L5-S66, p. 3]
- They treat IMU delay (communication, filter and logic delays) as "in general fixed", and compensate it the same way, by moving the moment the IMU is polled. [L5-S66, p. 3]
- With both compensations the average estimated camera–IMU delay was "only about 7 µs". Plain periodic triggering, with the stamp at the trigger time, gave an exposure-dependent delay. [L5-S66, p. 6, Fig. 6]
- A system that stamps images at the start of exposure has "a varying, exposure dependent offset" to IMU data. [L5-S66, p. 2]
- Scanning lidar motion distortion: because the points of one sweep are received at different times, a moving lidar produces a distorted cloud. When the scan rate is high compared with the motion, the distortion can often be neglected. When scanning is slow it "can be severe". [L5-S67, pp. 1–2]
- LOAM corrects this distortion by assuming constant angular and linear velocity during a sweep. It linearly interpolates the pose for each point from that point's own timestamp, and reprojects the points to one time. [L5-S67, pp. 4–5, eq. 4]
- LOAM's tests placed the lidar on a cart indoors and on a ground vehicle outdoors, all at 0.5 m/s. (Domain: ground-vehicle lidar mapping.) [L5-S67, p. 7]
- Even with FPGA timestamping, logic and filter delays inside sensors and polling delays remain. [L5-S12, p. 5]
- In VersaVIS, the camera–IMU offset depended strongly on the IMU's internal filter setting, so the authors say such delay should be compensated in the driver or the estimator. [L5-S20, p. 10]

### 8. Temporal calibration: estimating offsets between sensors
- Furgale et al. estimate the time offset jointly with the spatial transform, in continuous-time maximum-likelihood batch estimation, with time-varying states represented as B-splines. [L5-S12, pp. 1, 3]
- Across 40 real datasets, Furgale et al.'s offset estimates stayed within ±0.2 ms of the line of best fit, "just 4% of the shortest measurement period". [L5-S12, p. 6, Fig. 5]
- Their calibration motion averaged about 55°/s absolute angular velocity and 1.1 m/s² acceleration, to make all quantities observable. [L5-S12, p. 5]
- Kalibr turns temporal calibration on by default. Its wiki reports good results with a 20 Hz camera and a 200 Hz IMU. [L5-S38, "2) Collect images", "3) Running the calibration"]
- Kalibr advises exciting all IMU axes and ensuring "low jitter timestamps in same clock". It advises inspecting IMU timestamp intervals, citing a case of bursts at 1 ms spacing with 6 ms gaps caused by buffering. [L5-S38, "2) Collect images" Tips; "4) The output"]
- Qin and Shen estimate the camera–IMU offset online in an optimization-based VIO, by shifting each feature observation along its estimated image-plane velocity. [L5-S18, p. 2]
- On EuRoC V101 with added 5, 15 and 30 ms offsets, their estimates converged within a few seconds. [L5-S18, pp. 6–7]
- Qin and Shen summarize earlier methods (Mair and Kelly & Sukhatme were not read here):
  - Mair: cross-correlation or phase congruency;
  - Kelly & Sukhatme: ICP alignment of rotation curves;
  - Kalibr: offline batch estimation with a calibration pattern;
  - Li & Mourikis: an online offset state in an MSCKF.
  [L5-S18, p. 2]
- Li and Mourikis add the camera–IMU time offset td to the EKF state, together with IMU pose, velocity, biases, the camera-to-IMU transform and feature positions. They apply it to map-based localization, SLAM and visual-inertial odometry. [L5-S53, PDF p. 1, abstract]
- They show td is locally identifiable except in a small set of degenerate motions. The two that can occur in practice are zero rotation and constant rotational velocity, and these already cause loss of observability even when td is known. [L5-S53, PDF pp. 9–10]
- In their indoor experiment (Xsens MTi-G at 100 Hz, camera at 20 Hz), the estimate converged within the first few seconds and its final standard deviation was 0.40 ms. [L5-S53, PDF pp. 10–11]
- In simulation, a td drifting linearly from 20 ms to 520 ms over 500 s ("a severe clock drift") was tracked with consistent estimates. [L5-S53, PDF p. 15]
- Kelly, Grebe and Giamou show structural problems when a delay is added to an EKF state vector, including in GPS/INS (Nilsson, Skog, Händel) and VIO (Li & Mourikis). They conclude that such filters are prone to bias and inconsistency, and are sensitive to initial conditions. [L5-S19, pp. 1, 8]
- As a remedy, Kelly et al. suggest a sliding-window estimator large enough to cover any feasible delay. [L5-S19, p. 8]
- VersaVIS notes that constant offsets can be calibrated, but changing offsets (OS scheduling, separate device clocks, uncompensated exposure changes) are the critical ones to avoid. [L5-S20, p. 9]

### 9. Time in ROS 2
- ROS 2 defines three time abstractions: `SystemTime` (tied to the system clock), `SteadyTime` (monotonic) and `ROSTime`. [L5-S13, "Time Abstractions"]
- `ROSTime` equals `SystemTime` unless the `use_sim_time` parameter is set. Then it follows the `/clock` topic. [L5-S13, "ROS Time", "Default Time Source"]
- A ROS time of zero means "uninitialized". [L5-S13, "Default Time Source"]
- `SteadyTime` is typed so that it cannot be compared with `SystemTime` or `ROSTime`. [L5-S13, "Implementation"]
- Time jumps, especially backwards (log playback), must be handled with jump callbacks, which the API provides. [L5-S13, "Challenges in using abstracted time", "Public API"]
- The design assumes nodes have a synchronized system clock. It recommends bringing an external source such as GPS in through standard NTP integration with the system clock, rather than using it directly as ROS time. [L5-S13, "Background", "Custom Time Source"]
- `std_msgs/Header` carries a seconds + nanoseconds stamp and a frame_id. [L5-S35, `Header.msg`]
- `sensor_msgs/Image` says the stamp "should be acquisition time of image". [L5-S35, `Image.msg`]
- `sensor_msgs/TimeReference` pairs a system-time stamp with the corresponding time from an external source "not actively synchronized with the system clock". [L5-S35, `TimeReference.msg`]
- tf2 buffers:
  - Default cache: 10 s of transforms. [L5-S32, `buffer_core.hpp` `BUFFER_CORE_DEFAULT_CACHE_TIME`; `time_cache.hpp` `TIMECACHE_DEFAULT_MAX_STORAGE_TIME`] [L5-S33, `Learning-About-Tf2-And-Time-Cpp.rst` Background]
  - Data older than the newest stamp minus the cache length is rejected, and tf2 warns `TF_OLD_DATA`. [L5-S32, `cache.cpp` `insertData`; `buffer_core.cpp` l. 295]
- tf2 lookups:
  - Between two stored stamps, tf2 interpolates translation linearly and rotation by slerp. [L5-S32, `cache.cpp` `interpolate`]
  - A request later than the newest stored stamp fails with "extrapolation into the future"; earlier than the oldest, with "extrapolation into the past". [L5-S32, `cache.cpp` `findClosest`]
  - Time 0 means "the latest available" transform. For a chain of frames, tf2 uses the latest time common to all links. [L5-S33, `Learning-About-Tf2-And-Time-Cpp.rst` §1] [L5-S32, `buffer_core.cpp` `getLatestCommonTime`]
- A transform takes "usually a couple of milliseconds" to reach a listener's buffer. `lookupTransform` therefore takes an optional timeout to wait for it. [L5-S33, `Learning-About-Tf2-And-Time-Cpp.rst` §§1–2]
- `tf2_ros::MessageFilter` caches stamped messages until they can be transformed into the target frame. [L5-S33, `Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.rst`]
- `tf2_ros::MessageFilter` drops messages for reasons including `OutTheBack` (older than all cached transforms) and `QueueFull`. [L5-S32, `message_filter.hpp` `FilterFailureReason`]
- robot_localization's `transform_timeout` sets how long to wait for a transform. The default of 0 takes the latest available transform. A non-zero value can make the filter miss its output rate. [L5-S36, `~transform_timeout`]
- message_filters synchronization policies:
  - `ExactTime` needs identical stamps. [L5-S34, `index.rst` 6.2]
  - `ApproximateEpsilonTime` accepts stamps within an epsilon. [L5-S34, `index.rst` 6.3]
  - `ApproximateTime` uses an adaptive matching algorithm. [L5-S34, `index.rst` 6.4]
- `ApproximateTime` details:
  - Without a header, Python's `allow_headerless=True` substitutes current ROS time. [L5-S34, `index.rst` 6.4]
  - It warns once if a topic's messages arrive out of order. [L5-S34, `approximate_time.h` l. 193]
  - Defaults: age penalty 0.1 and an effectively unlimited maximum interval. A queue size of 1 "will tend to drop many messages", and at least 2 is recommended. [L5-S34, `approximate_time.h` ll. 121–127]
- `TimeSequencer` holds messages for a set delay, releases them in stamp order, and discards any message older than one already released. [L5-S34, `index.rst` §4 "Time Sequencer"]
- ROS 2 middleware adds latency: Kronauer et al. found end-to-end communication overhead up to 50% compared with using DDS directly (as reported by Bédard et al.). [L5-S22, p. 2]
- In Kronauer et al.'s own profiling, the up-to-50% overhead of ROS 2 over raw DDS occurs for small (128 B) messages. The largest shares come from the DDS middleware and the "rclcpp notification delay", the time between DDS signalling new data and ROS 2 actually taking it. [L5-S60, pp. 4–5]
- Their rules of thumb: no middleware was fastest in all cases; above the UDP fragmentation size (64 KB) latency grows with payload; higher publishing frequency gave lower latency; and latency "highly depends on energy saving features of the OS and the hardware". For their tests they changed kernel settings such as the CPU scaling governor to reduce this noise. [L5-S60, pp. 4, 6–7]
- Every received ROS 2 message carries middleware metadata with a `source_timestamp` (when published) and a `received_timestamp` (when received). The exact point where each is taken is not specified, but it should be the same every time. Where the rmw supports it, a per-publisher `publication_sequence_number` shows how many messages the publisher sent between two received ones; those were either lost or taken by other `rmw_take` calls. [L5-S61, `rmw_message_info_s`]
- Nav2 (Humble) time handling:
  - `nav2_util::getTransform` looks up the latest transform (`tf2::TimePointZero`) and uses `transform_tolerance` as the wait timeout. [L5-S63, `robot_utils.cpp` `getTransform`]
  - The costmap `ObservationBuffer` transforms each sensor cloud into the global frame at the cloud's own header stamp. [L5-S63, `observation_buffer.cpp` `bufferCloud`]
  - Observations older than `observation_keep_time` (now − stamp) are purged. With a keep time of 0, only the newest observation is kept. [L5-S63, `observation_buffer.cpp` `purgeStaleObservations`]
  - `isCurrent()` warns when a buffer has not been updated within `expected_update_rate`. A value of 0 disables the check. [L5-S63, `observation_buffer.cpp` `isCurrent`]
  - Current (rolling) Nav2 documentation lists defaults of `transform_tolerance` 0.3 s for the costmap, and 0.0 for `observation_persistence` and `expected_update_rate`. These defaults are from the rolling documentation and were not checked against the Humble source. [L5-S64, costmap_2d `index.md`; `obstacle.md`]

### 10. Measuring and verifying timing on a robot
- NTP gives a checkable error bound: root distance = root dispersion + half the root delay. The system clock's maximum error adds the remaining correction shown as `System time` in `chronyc tracking`. [L5-S24, FAQ 7.2]
- chrony's default root-dispersion growth rate (`maxclockerror`) is 1 ppm. [L5-S24, FAQ 7.2]
- A reference clock's offset can be measured by marking it `noselect` and comparing it against an NTP server in `chronyc sourcestats`. [L5-S24, FAQ 3.9]
- u-blox measures time-pulse accuracy against a reference GPS receiver synchronized to a rubidium clock, averaging 1 s samples over 6 h. [L5-S30, section 2.1]
- VersaVIS verified camera-to-camera sync with an LED timing board whose count advances on every trigger. Identical counts in 400 image pairs showed better than 0.5 ms. [L5-S20, pp. 8–9]
- Offset consistency across repeated Kalibr calibrations is used to verify camera–IMU sync, e.g. offsets consistent to below 0.05 ms for VersaVIS. [L5-S20, p. 10]
- An Intel RealSense T265 gave inconsistent offsets with a bimodal distribution, the modes about 15 ms apart (which the authors relate to half the inter-frame time). The authors read this as a sign that some datasets had images shifted by one frame. [L5-S20, p. 10]
- ros2_tracing instruments ROS 2 with the LTTng tracer to record message publication and reception times across processes. Its average added end-to-end latency with all instrumentation on was 0.0033 ms. [L5-S22, pp. 1, 5]
- `diagnostic_updater::TimeStampStatus` compares each message stamp with the current clock:
  - It reports "Timestamps too far in future seen" if the delay is below `min_acceptable` (default −1 s).
  - It reports "too far in past" if the delay is above `max_acceptable` (default 5 s).
  - It flags zero timestamps.
  [L5-S37, `update_functions.hpp` `TimeStampStatusParam`, `TimeStampStatus::run`]
- The Velodyne ROS 2 driver uses `TimeStampStatus` on its scan stamps. [L5-S44, `driver.cpp` ll. 169, 282]
- Kalibr's wiki advises inspecting IMU timestamp differences ("DTs") to detect buffered or bursty stamping. [L5-S38, "4) The output"]
- Clock offset between two devices can be measured directly by comparing their PPS outputs. Frankó and Hollósi captured both PPS edges with a microcontroller timer input capture at about 5.9 ns resolution. The PC network cards had no easily available PPS output, so embedded boards were used as grandmaster and slave. [L5-S59, pp. 4–5]
- End-to-end camera latency ("glass-to-glass" delay) can be measured with an LED in the camera's view and a phototransistor on the display. Bachhuber and Steinbach sample the phototransistor at 2 kHz and report 0.5 ms measurement precision, limited mainly by the sampling rate. [L5-S65, pp. 1, 3]
- The LED switches in negligible time and the phototransistor has a 10 μs rise and fall time, both small compared with the delay measured. [L5-S65, p. 2]
- Because the event happens at a random point in the camera's frame period, the camera's share of the delay is uniformly distributed. So the result is a distribution, not one number: in their 50 Hz example the delay ranged from 19.1 ms to 52.4 ms, with a standard deviation of 6.9 ms. [L5-S65, pp. 3–4]
- An earlier method that filmed moving LEDs with a camera recording at up to 200 Hz had "an average imprecision of 5 milliseconds". [L5-S65, p. 2]

### 11. How other domains do it
- Automotive, DARPA Urban Challenge: MIT's vehicle synchronized all its sensors passively to one time base. [L5-S11, p. 2]
- Automotive, KITTI dataset: KITTI triggered its cameras from the lidar. It took the nearest 100 Hz GPS/IMU sample, giving a worst-case 5 ms mismatch, and recorded all stamps with the host system clock. [L5-S21, p. 4]
- Automotive, DRIVE AGX Orin (not Jetson): NVIDIA DRIVE OS runs `ptp4l` in an automotive profile to lock a Tegra PHC to an external grandmaster. It aligns the CPU timestamp counter (TSC) to the PHC using a 1 Hz PPS, which also aligns camera frame-sync signals to the PTP second. [L5-S46, "Orin Time Sync", "AVNU PTP for Development"]
- Driver assistance: fusion of lidar, radar and camera with different preprocessing times produces OOSMs, even on time-triggered buses (FlexRay, TTCAN). [L5-S16, pp. 1–2]
- Marine and aerial georeferencing: Syncline curves compare UAV, USV, AUV and surface vessels. Fast platforms need much tighter synchronization for the same georeferencing error. [L5-S14, pp. 1, 4]
- Planetary landing: latent, jittery terrain-camera measurements are handled by modelling the latency, with an error budget of timing effects. [L5-S15, pp. 1, 27]
- Visual-inertial sensor suites: hardware-triggered, mid-exposure-stamped systems reach sub-millisecond camera–IMU timing. [L5-S20, p. 1] [L5-S12, p. 6]
- Industrial measurement and control: this is IEEE 1588's original target. [L5-S05, p. 10]
- Automotive, AUTOSAR: the AUTOSAR time synchronization protocol, a restricted gPTP profile, targets "time-critical and safety-related automotive applications such as airbag systems and braking systems". [L5-S58, PDF p. 5, section 1.2]
- Industrial time-sensitive networking (TSN): gPTP accuracy falls as boundary-clock hops are added between grandmaster and slave, when each hop runs a PI servo. [L5-S59, p. 1]
- Navigation, GNSS-aided INS: factor-graph smoothers take delayed GPS fixes at their true time, which filters need special methods for. [L5-S57, pp. 2, 4]

### 12. Product specifics: Xsens MTi-680G, Velodyne VLP-16, ZED X, USB-serial links, Jetson Orin
**Xsens MTi-600 series (MTi-680G)**
- `SampleTimeFine` is the sample time in 10 kHz clock ticks. `SampleTimeCoarse` is in seconds. The two combine into a long-range timestamp. [L5-S41, pp. 47–48]
- `UtcTime` gives nanoseconds, date, time and validity flags. [L5-S41, p. 47]
- `GnssPvtPulse` is in the same clock domain as `SampleTimeFine`, and relates IMU samples to GNSS PVT samples. [L5-S41, p. 52]
- Clock bias estimation:
  - The internal clock (about 10 ppm) can be disciplined to an external reference. On the MTi-670G/680G it is always referenced to the internal GNSS receiver, and this is not user-configurable. [L5-S40, p. 37, "Clock Bias Estimation function"]
  - With ClockSync the output stream follows the external reference, but timestamps are still defined on the unadjusted internal sampling clock. [L5-S40, p. 38]
  - The 1PPS time-pulse function uses the GNSS 1PPS to synchronize the MTi. It is always enabled on the 680G. [L5-S40, p. 38]
- SyncIn and SyncOut:
  - The MTi-600 has two SyncIn lines and one SyncOut. [L5-S40, p. 35]
  - SyncIn functions include TriggerIndication, which outputs a message timestamped at the trigger; SendLatest; StartSampling; and ClockSync. [L5-S40, pp. 36–38]
  - SyncOut's Interval Transition Measurement gives a pulse from the internal 400 Hz sampling clock, and a 1 Hz output synchronized to the GNSS 1PPS is possible. [L5-S40, pp. 35, 38]
- In NMEA-input mode (external receivers, MTi-670/680), the MTi also synchronizes its internal clock to the UTC time in the sentences. [L5-S40, p. 30]
- Xsens ROS 2 driver (commit e145fb5), `time_option` modes:
  - `0` uses the MTi's UTC time (the default, "recommended for accurate time synchronization"). [L5-S42, `xsens_mti_node.yaml`]
  - `1` uses time integrated from `SampleTimeFine`. [L5-S42, `xsens_mti_node.yaml`]
  - `2` uses host time. [L5-S42, `xsens_mti_node.yaml`]
- In `SampleTimeFine` mode, the driver stamps the first packet with host `now()`. It then adds tick differences × 100 μs (`timeDiff * 1e5` ns), handling wraparound. [L5-S42, `xsens_time_handler.cpp`]
- If the requested field is missing, the Xsens driver falls back in two steps: with `time_option` 0 and no UTC time in the packet it uses `SampleTimeFine` if present, and only when neither applies does it use host `now()`. [L5-S42, `xsens_time_handler.cpp` `convertUtcTimeToRosTime`]

**Velodyne VLP-16 (manual 63-9243 Rev. F, 2022; ROS 2 driver commit 56fc178)**
- The sensor counts microseconds since the top of the hour (TOH, 0 to 3,599,999,999 μs) on an internal oscillator, and sends the count in every packet. [L5-S43, pp. 133–134]
- A valid PPS rising edge realigns the sub-second counter. A GPRMC or GPGGA sentence sets minutes and seconds. [L5-S43, pp. 43, 61, 134]
- PPS lock is shown by status byte 0x02. [L5-S43, p. 134]
- If the PPS becomes unstable, the sensor free-runs. The PPS-qualification `Delay` defaults to 5 s. [L5-S43, p. 135]
- PPS and NMEA must alternate: at least 50 ms from the end of the PPS to the NMEA start, and at least 300 ms from the NMEA end to the next PPS. The pulse width is not critical (typically 10 μs to 200 ms). [L5-S43, p. 44]
- The packet timestamp is the time of the packet's first data point. Each point's time = timestamp + 55.296 μs × sequence index + 2.304 μs × data-point index. [L5-S43, p. 68]
- By default (`gps_time: false`), the ROS 2 driver stamps each packet with the average of host times read just before and after `recvfrom`, plus `time_offset`. [L5-S44, `input.cpp` `InputSocket::getPacket`]
- With `gps_time: true`, the ROS 2 driver combines the TOH microseconds with the hour from the host clock. It shifts by one hour if host and sensor disagree by more than half an hour. [L5-S44, `time_conversion.hpp`]
- The ROS 2 driver's scan stamp is the last packet's stamp, unless `timestamp_first_packet` is true. [L5-S44, `driver.cpp` ll. 274–276]

**Stereolabs ZED X**
- ZED camera sensors share "a common and low-drift reference clock". Incoming packets are "timestamped upon reception by the host machine" in Epoch time with nanosecond resolution. [L5-S45, "Sensors Time Synchronization"]
- `TIME_REFERENCE::IMAGE` gives data at the frame's time. `TIME_REFERENCE::CURRENT` gives data at the time of the call. [L5-S45, time-sync page; API `TIME_REFERENCE`]
- The `IMAGE` stamp is anchored at the start of sensor readout, which is the end of integration. `IMAGE_CENTER_OF_EXPOSURE` is half an exposure earlier. [L5-S45, API `TIME_REFERENCE`]
- `IMAGE_CENTER_OF_EXPOSURE` is exact on global-shutter ZED X, available only through `getTimestamp()`, and returns 0 on USB cameras. [L5-S45, API `TIME_REFERENCE`]
- The ZED ROS 2 wrapper (v5.2.2), with a live camera, stamps each grabbed frame with `getTimestamp(sl::TIME_REFERENCE::IMAGE)`, not `IMAGE_CENTER_OF_EXPOSURE`. [L5-S62, `zed_camera_component_main.cpp` ll. 5086–5103]
- With `debug.use_pub_timestamps: true` (default false), the wrapper stamps messages with the ROS clock's `now()` at publication instead. The config comment says this "is useful to test data communication latency". [L5-S62, `common_stereo.yaml` l. 200; `zed_camera_component_main.cpp` l. 744]
- ZED IMU, barometer and magnetometer messages are stamped with the SDK's own sensor timestamps, unless `sensors.sensors_image_sync` (default false) is set. That option publishes the sensor data with the stamp of the video/depth message just published. [L5-S62, `zed_camera_component_main.cpp` ll. 5329–5366; `zed_camera_component_video_depth.cpp` ll. 2784–2795; `common_stereo.yaml` l. 61]
- In SVO file playback without `use_svo_timestamps`, the wrapper uses `TIME_REFERENCE::CURRENT`, the time of the call, instead of the recorded frame time. [L5-S62, `zed_camera_component_main.cpp` ll. 5086–5092]

**USB-serial links**
- FTDI latency timer: 16 ms default, adjustable 1–255 ms. [L5-S39, pp. 7–8]
- Syncline's authors state FTDI chips are "known to introduce up to 16ms buffering delay". [L5-S14, p. 2]

**Jetson Orin**
- Jetson Linux r36.4.4 lists "IEEE 1588-2008 (PTP)" among the EQOS Ethernet controller features for the Orin series. [L5-S47, EQOS Ethernet feature table]
- From JetPack 6.0, the Generic Timestamp Engine (GTE) driver for Jetson AGX Orin, Orin NX and Orin Nano is deprecated and replaced by the upstream kernel Hardware Timestamp Engine (HTE). [L5-S48]

## Recommended practice
1. Stamp at acquisition, not arrival. Use hardware timers, triggers, PPS or the sensor's own synchronized clock where available. [L5-S14, p. 2] [L5-S12, p. 1]
2. Where only arrival times exist, map the device clock with a lowest-latency, skew-aware estimator rather than raw arrival times. [L5-S11, p. 5] [L5-S17, p. 1] [L5-S20, p. 6]
3. Stamp images at mid-exposure, or trigger cameras half an exposure early so mid-exposure times are periodic. [L5-S12, p. 5] [L5-S20, pp. 4–5] [L5-S45, API `TIME_REFERENCE`]
4. Use NIC hardware timestamping for NTP or PTP on a LAN, and several time sources. [L5-S26, `hwtimestamp`] [L5-S24, FAQ 2.7]
5. Allow clock steps only at boot, before time-dependent programs start, and slew afterwards. [L5-S26, `makestep`]
6. Pair each PPS pulse with a time-of-day source, and set the NMEA delay or offset correctly. [L5-S24, FAQ 3.9] [L5-S27, `ts2phc.nmea_delay`]
7. Measure and compensate the fixed delays (cable, receiver, driver) as constants. [L5-S31, section 1.2] [L5-S30, section 2.1]
8. Calibrate the remaining constant sensor-to-sensor offset with rich motion, and check it for consistency across repeated runs. [L5-S12, p. 5] [L5-S38, "2) Collect images" Tips] [L5-S20, p. 10]
9. Fuse lagged data at its true time, by re-processing from a history at least as long as the lag, or with a delay-aware update. [L5-S36, `~smooth_lagged_data`, `~history_length`] [L5-S09, p. 2]
10. Monitor stamp age and plausibility at runtime. [L5-S37, `TimeStampStatus`]
11. Reduce USB-serial buffering (FTDI latency timer). [L5-S39, p. 8]
12. Compensate fixed sensor delays (exposure, IMU filter and communication delays) in the acquisition itself, e.g. by shifting trigger or polling times. [L5-S66, p. 3]
13. For scanning lidars, use per-point times to remove motion distortion when the motion during a sweep is not negligible. [L5-S67, pp. 1–2, 4–5]
14. When measuring latency, report a distribution (minimum, mean, maximum), because frame-based sampling spreads the delay over a frame period. [L5-S65, pp. 3–4]

## Key numbers
| Quantity | Value | Conditions | Source |
|---|---|---|---|
| Projection error from timing error | 15.7 cm | 90 deg/s rotation, object at 10 m, 10 ms error | L5-S11, p. 1 |
| Host scheduling jitter | "hundreds of milliseconds" | Loaded non-real-time OS | L5-S11, p. 1 |
| FTDI latency timer | 16 ms default; 1–255 ms adjustable | FT232R/FT245R/FT2232C/BM chips | L5-S39, pp. 7–8 |
| NTP accuracy | "a few milliseconds" | Internet, 1991 | L5-S01, p. 1 |
| NTPv4 accuracy | tens of μs (primary); a few hundred μs (LAN clients) | Modern workstations | L5-S02, section 1 |
| chrony client accuracy | 35 ± 8 μs | Simulation, 10 μs network jitter, stable clock | L5-S25, Test 1 |
| chrony with HW timestamping | sub-μs, stability of a few tens of ns "might be possible" | Local HW timestamping, good switches, short polling | L5-S24, FAQ 2.7 |
| NTP step / panic thresholds | 125 ms / 1000 s | RFC 5905 | L5-S02, section 11.3 |
| PTP practical accuracy | ~100 ns; <20 ns needs better hardware | 2 s updates, inexpensive oscillators, PI servo | L5-S05, p. 73 |
| GNSS time pulse | 5 ns (1σ) absolute, 2.5 ns differential, ±4 ns jitter | ZED-F9T-10B | L5-S31, section 1.2 |
| GNSS time pulse (older) | 30 ns RMS uncompensated, 15 ns compensated | LEA-6T datasheet via app note | L5-S30, pp. 6–7 |
| PPS-to-NMEA pairing limit | offset < 0.4 s (< 0.2 s before chrony 4.1) | chrony PPS refclock | L5-S24, FAQ 3.9 |
| Quartz frequency error | 0.01 % accuracy; several ppm with room temperature | Common computers | L5-S01, p. 11 |
| Xsens MTi-600 internal clock | about 10 ppm | Before external clock sync | L5-S40, p. 37 |
| Sensor clock drift example | 92 s lost in 6 days; >30 ms temperature swings | SICK LMS151 in an office | L5-S17, p. 1 |
| Two-way clock mapping (TICSync) | ~35 μs offset error | 162 ms round-trip path | L5-S17, p. 7 |
| USB clock translation (VersaVIS) | ±5 ms raw → ±0.2 ms after EKF | 1 s updates, ~60 s convergence | L5-S20, p. 11 |
| Batch temporal calibration | within ±0.2 ms (4% of shortest period) | Camera–IMU, 40 datasets | L5-S12, p. 6 |
| KITTI GPS/IMU-to-frame mismatch | 5 ms worst case | 100 Hz INS, nearest sample | L5-S21, p. 4 |
| tf2 default cache | 10 s | ROS 2 Humble | L5-S32, `buffer_core.hpp` l. 73 |
| VLP-16 point timing | 55.296 μs per firing sequence, 2.304 μs per point | Single return | L5-S43, p. 68 |
| Xsens SampleTimeFine | 10 kHz ticks | MTi-600 | L5-S41, p. 47 |
| ros2_tracing overhead | 0.0033 ms average | All instrumentation enabled | L5-S22, p. 1 |
| Oscillator accuracy (incl. environment, 1 year aging) | XO 10⁻⁵–10⁻⁴; TCXO 10⁻⁶; OCXO 10⁻⁸ | Vig hierarchy | L5-S55, PDF p. 33 |
| Temperature stability, −55 to +85 °C | TCXO 5 × 10⁻⁷; OCXO 1 × 10⁻⁹ | Vig comparison table | L5-S55, PDF p. 237 |
| GPS time vs UTC | within 1 μs (modulo 1 s); GPS–UTC data within 20 ns (1σ) | Control segment requirement | L5-S56, section 3.3.4 |
| GPS legacy week number | 10 bits, modulo 1024 weeks | LNAV message | L5-S56, section 20.3.3.3.1.1 |
| chrony leap smear (recommended example) | 62500 s (about 17.36 h); max 32 ppm | `leapsecmode slew` + `smoothtime` | L5-S26, `leapsecmode` |
| ptp4l PI scale constants | kp 0.7, ki 0.3 (HW); 0.1, 0.001 (SW) | Defaults when not set | L5-S27, `ptp4l.8` |
| ROS 2 overhead vs raw DDS | up to 50 % | 128 B messages | L5-S60, p. 5 |
| Online camera–IMU offset (EKF) | 0.40 ms final std. dev. | Xsens MTi-G 100 Hz + camera 20 Hz | L5-S53, PDF p. 11 |
| Hardware-synchronized camera–IMU | about 7 µs average delay | FPGA unit, exposure- and delay-compensated triggering | L5-S66, p. 6 |
| Glass-to-glass measurement precision | 0.5 ms | LED + phototransistor sampled at 2 kHz | L5-S65, p. 1 |
| PPS comparison resolution | about 5.9 ns | 195.5 MHz timer input capture | L5-S59, p. 5 |

## How it is tested
| Test | What it measures | Pass criterion used in the source | Source |
|---|---|---|---|
| Time pulse vs rubidium-disciplined GPS reference | Absolute PPS accuracy | Reported std. dev. (6.7 ns) and mean (−2.35 ns, removable as user delay) | L5-S30, section 2.1 |
| NTP root distance / `chronyc tracking` | Upper bound on system clock error | Root dispersion + ½ root delay + remaining correction | L5-S24, FAQ 7.2 |
| `noselect` reference compared with NTP in `sourcestats` | Offset of an NMEA or PPS refclock | Offset corrected until below the pairing limit (0.4 s) | L5-S24, FAQ 3.9 |
| LED counter board seen by triggered cameras | Camera-to-camera sync | Same LED count in all 400 pairs → better than 0.5 ms | L5-S20, pp. 8–9 |
| Repeated Kalibr calibrations | Camera–IMU offset consistency | Low std. dev.; consistent offsets (< 0.05 ms) | L5-S20, p. 10 |
| Offset vs exposure time regression | Whether image stamps sit at mid-exposure | Slope near 0.5 (measured 0.498); residual within ±0.2 ms | L5-S12, p. 6 |
| Online offset estimation with injected offsets | Temporal calibration convergence | Converges within a few seconds for 5/15/30 ms | L5-S18, pp. 6–7 |
| Clock translation residual and innovation | Host–MCU clock mapping | Zero-mean innovation ±0.2 ms after convergence | L5-S20, p. 11 |
| `TimeStampStatus` diagnostic | Stamp age vs current clock | Delay within [−1 s, 5 s] by default; no zero stamps | L5-S37, `update_functions.hpp` |
| IMU timestamp interval plot | Buffered or bursty stamping | Regular intervals; bursts (1 ms spacing with 6 ms gaps) flag a problem | L5-S38, "4) The output" |
| ros2_tracing message-flow trace | Publication and reception times per message | Not a pass/fail test; used to find latency bottlenecks | L5-S22, p. 5 |
| PPS output comparison with a timer input capture | Offset between two synchronized clocks | Step response (overshoot, settling time) compared with a model | L5-S59, pp. 4–5 |
| LED + phototransistor glass-to-glass test | Camera-to-display latency distribution | Measured min/max match the frame-period model (19.1–52.4 ms at 50 Hz) | L5-S65, pp. 2–4 |
| Offset vs exposure time with periodic vs compensated triggering | Camera–IMU delay after hardware sync | Average delay about 7 µs with compensation | L5-S66, p. 6 |
| Middleware source vs received timestamps | Per-message transport latency and losses | Not a pass/fail test; a sequence-number gap counts messages sent but not received by this take (lost or taken elsewhere), where the rmw supports it | L5-S61, `rmw_message_info_s` |

## Common mistakes
- Stamping data on arrival at a loaded non-real-time host, which adds up to hundreds of milliseconds of jitter. [L5-S11, p. 1]
- Leaving the FTDI latency timer at 16 ms, which delays small messages by 16 ms. [L5-S39, p. 7]
- Ignoring the FTDI worst case: 62 bytes arriving within 16 ms do not trigger the timeout, so each 64-byte packet is sent only every 16 ms. [L5-S39, p. 7]
- Assuming a symmetric network path when it is not, which biases the offset (the PTP tutorial example gives 55 instead of 60 minutes). [L5-S05, p. 24] [L5-S24, FAQ 7.2]
- Letting the clock step while programs run. Steps reset NTP associations, and chrony recommends stepping only at boot. [L5-S02, section 11.2.3] [L5-S26, `makestep`]
- Pairing PPS pulses with an NMEA source whose delay is not corrected, so pulses are assigned to the wrong second. [L5-S24, FAQ 3.9] [L5-S27, `ts2phc.nmea_delay`]
- Estimating clock offset without skew, so the offset drifts after convergence. [L5-S20, pp. 10–11]
- Stamping images at readout (end of exposure) instead of mid-exposure when exposure varies, which shifts the offset with exposure time. [L5-S12, p. 6] [L5-S45, API `TIME_REFERENCE`]
- Requesting a tf2 transform at "now" without a timeout: it fails with "extrapolation into the future" because the newest transform is a few milliseconds old. [L5-S33, `Learning-About-Tf2-And-Time-Cpp.rst` §1]
- Feeding lagged measurements to robot_localization without `smooth_lagged_data`: they are applied to the current state with no prediction back to their time. [L5-S36, `filter_base.cpp` `processMeasurement`]
- Using `ApproximateTime` with a queue size of 1, which "will tend to drop many messages". [L5-S34, `approximate_time.h` l. 127]
- Ignoring a GNSS receiver's Void status while it keeps outputting PPS from its internal clock. [L5-S43, p. 52]
- Stamping images at the start of exposure, which gives an offset to IMU data that changes with exposure. [L5-S66, p. 2]
- Handling a leap second inconsistently: equipment must carry or borrow consistently into year/week/day counts when 23:59:60 appears. [L5-S56, section 20.3.3.5.2.4 b]
- Mixing leap-smearing and non-smearing NTP servers, or servers that smear differently. [L5-S26, `leapsecmode`]
- Treating a lidar sweep as one instant when the robot moves fast relative to the scan rate. [L5-S67, pp. 1–2]
- Measuring latency with power-saving features left on, which adds noise to the result. [L5-S60, pp. 4, 7]

## Disagreements between sources
- **Is NTP suitable on robots?**
  - Harrison and Newman argue it is undesirable: synchronization can take many hours, and frequency adjustments make timestamps inconsistent. [L5-S17, p. 1]
  - The ROS 2 design recommends integrating external time (e.g. GPS) through standard NTP with the system clock. [L5-S13, "Custom Time Source"]
  - chrony reports fast synchronization and, with hardware timestamping, sub-microsecond accuracy. [L5-S25, "Summary"] [L5-S24, FAQ 2.7]
  - Inference (this review, not stated by the sources): the sources seem to differ mainly in which NTP implementation and configuration they assume.
- **Can a filter estimate its own time offset?**
  - Skog's GNSS-aided INS work reports "almost perfect time synchronization" when the timing error is added to the estimator. [L5-S10, PDF p. 15]
  - Qin and Shen describe Li and Mourikis's filter-based (MSCKF) online calibration as having a significant advantage in computational complexity, but less accurate than their own optimization-based method. [L5-S18, p. 2]
  - Kelly, Grebe and Giamou argue that putting the delay in an EKF state is structurally flawed, prone to bias and inconsistency, and sensitive to initial conditions. [L5-S19, pp. 1, 8]
  - As a possible remedy, Kelly et al. themselves suggest a sliding-window estimator large enough to cover any feasible delay. [L5-S19, p. 8] That batch and optimization methods (L5-S12, L5-S18) escape this criticism is an inference of this review; no source states it.
- **Buffer or use an OOSM algorithm?**
  - Mauthner et al. suggest ("hinted at the possibility") that simple buffering can compete with advanced OOSM algorithms under certain cycle-time conditions. [L5-S16, p. 10]
  - Brouk and DeMars consider reprocessing and smoothing approaches computationally burdensome and lagging, and prefer a latency state. [L5-S15, pp. 2–3]
- **How much does processing strategy matter?**
  - Brouk and DeMars conclude that better timing hardware and characterization matter more than better processing strategies. [L5-S15, p. 27]
  - Larsen et al. show real differences between delay-handling methods. [L5-S09, p. 6]
- **Filter, buffer, or smoother for delayed data?**
  - Indelman et al. say buffering past solutions in a filter gives only an approximate solution, and that a factor-graph smoother handles delayed data as easily as any other. [L5-S57, pp. 1–2]
  - Brouk and DeMars consider fixed-lag smoothing computationally heavy and lagging behind real time. [L5-S15, pp. 2–3]
  - Mauthner et al. suggest buffering can be competitive under certain cycle-time conditions. [L5-S16, p. 10]
- **How should a leap second reach the system clock?**
  - The GPS specification expects 23:59:60 to appear, and allows equipment to decrement its count within several seconds afterwards. [L5-S56, section 20.3.3.5.2.4 b]
  - chrony's default steps the clock back one second at midnight UTC through the kernel. [L5-S26, `leapsecmode`]
  - chrony can instead smear the leap second by slewing over about 17 hours, which requires all servers to smear identically. [L5-S26, `leapsecmode`]
- **Is an EKF time-offset state reliable?** Li and Mourikis report consistent estimates, even for a drifting offset. [L5-S53, PDF pp. 11, 15] Kelly et al. argue such filters are prone to bias and inconsistency. [L5-S19, pp. 1, 8]

## Open questions
- Foundational works read only through secondary descriptions:
  - Cristian 1989 (L5-S49);
  - the IEEE 1588-2019 text (L5-S50);
  - Bar-Shalom's exact and multistep OOSM equations (L5-S51);
  - Skog & Händel's full error-covariance derivation (L5-S52);
  - Kelly & Sukhatme's TD-ICP method (L5-S54).
  Their findings here come only from abstracts or other authors' summaries.
- Per-degree temperature coefficients for specific XO/TCXO parts (as opposed to Vig's range-level stabilities, L5-S55) were not found in a manufacturer datasheet.
- How specific receivers (u-blox, Xsens) report the GPS–UTC leap-second count and handle week-number rollover was not checked; only the GPS specification (L5-S56) was read.
- No numeric accuracy requirement for gPTP (802.1AS) or AUTOSAR time sync was found in an open source. AUTOSAR refers to IEEE 802.1AS Annex B.1.2 (L5-S58), and the 802.1AS text is paywalled.
- Factor-graph insertion of late measurements is covered by one simulation study (L5-S57). No real-vehicle comparison of smoother vs filter handling of the same delayed data was found.
- Which Jetson Orin modules (AGX vs NX/Nano) expose a usable PHC on their Ethernet port, and what PPS-on-GPIO timestamping accuracy Jetson achieves, were not confirmed from NVIDIA documentation.
- Xsens MTi-680G latency from sampling to serial output, and the latency of non-FTDI USB-CDC links, were not found.
- VLP-16 packet latency from firing to host arrival was not found in the manual or driver source. (The ZED wrapper's stamping is now covered from source, L5-S62.)
- LED/photodiode latency rigs are documented for video (L5-S65, 0.5 ms precision). No equivalent published rig for lidar-to-actuator or IMU-to-actuator latency on a robot was found.
- A primary-source accuracy figure for Linux ROS 2 per-message `source_timestamp`/`received_timestamp` across machines (which depends on clock sync between hosts) was not found.

## Sources
| ID | Citation | Link | Accessed | File | Level |
|---|---|---|---|---|---|
| L5-S01 | D. L. Mills, "Internet time synchronization: the Network Time Protocol", *IEEE Trans. Communications* 39(10):1482–1493, 1991. DOI 10.1109/26.103043 (author reprint). | https://www.eecis.udel.edu/~mills/database/papers/trans.pdf | 2026-09-28 | sources/mills_1991_ntp_internet_time_sync.pdf | A |
| L5-S02 | D. Mills, J. Martin, J. Burbank, W. Kasch, "Network Time Protocol Version 4: Protocol and Algorithms Specification", IETF RFC 5905, 2010. | https://www.rfc-editor.org/rfc/rfc5905.txt | 2026-09-28 | sources/ietf_2010_rfc5905_ntpv4.txt | A |
| L5-S03 | D. L. Mills, "Adaptive hybrid clock discipline algorithm for the Network Time Protocol", *IEEE/ACM Trans. Networking* 6(5):505–514, 1998. DOI 10.1109/90.731182 (author copy). | https://www.eecis.udel.edu/~mills/database/papers/allan.pdf | 2026-09-28 | sources/mills_1998_hybrid_clock_discipline.pdf | A |
| L5-S04 | L. Lamport, "Time, clocks, and the ordering of events in a distributed system", *Comm. ACM* 21(7):558–565, 1978. DOI 10.1145/359545.359563 (author-hosted copy; PDF p. 1 = printed p. 558). | https://lamport.azurewebsites.net/pubs/time-clocks.pdf | 2026-09-28 | sources/lamport_1978_time_clocks_ordering.pdf | A |
| L5-S05 | J. C. Eidson, "IEEE-1588 Standard for a Precision Clock Synchronization Protocol for Networked Measurement and Control Systems — A Tutorial", NIST/Agilent, Oct. 2005 (slides). | https://www.nist.gov/document/tutorial-basicpdf | 2026-09-28 | sources/eidson_2005_ieee1588_tutorial.pdf | B |
| L5-S06 | J. Mogul, D. Mills, J. Brittenson, J. Stone, U. Windl, "Pulse-Per-Second API for UNIX-like Operating Systems, Version 1.0", IETF RFC 2783, 2000. | https://www.rfc-editor.org/rfc/rfc2783 | 2026-09-28 | sources/ietf_2000_rfc2783_pps_api.txt | A |
| L5-S07 | D. B. Sullivan, D. W. Allan, D. A. Howe, F. L. Walls (eds.), *Characterization of Clocks and Oscillators*, NIST Technical Note 1337, 1990. | https://www.nist.gov/system/files/documents/calibrations/tn1337.pdf | 2026-09-28 | sources/nist_1990_tn1337_clocks_oscillators.pdf | A |
| L5-S08 | W. J. Riley, *Handbook of Frequency Stability Analysis*, NIST Special Publication 1065, 2008. | https://tf.nist.gov/general/pdf/2220.pdf | 2026-09-28 | sources/riley_2008_nist_sp1065_frequency_stability.pdf | A |
| L5-S09 | T. D. Larsen, N. A. Andersen, O. Ravn, N. K. Poulsen, "Incorporation of time delayed measurements in a discrete-time Kalman filter", *Proc. 37th IEEE CDC*, pp. 3972–3977, 1998. DOI 10.1109/CDC.1998.761918 (DTU Orbit copy with cover page). | https://backend.orbit.dtu.dk/ws/files/4363506/Larsen.pdf | 2026-09-28 | sources/larsen_1998_delayed_measurements_kf.pdf | A |
| L5-S10 | I. Skog, *GNSS-aided INS for land vehicle positioning and navigation*, Licentiate thesis, KTH, TRITA-EE 2007:066, 2007 (DIVA file contains the thesis summary only, including the abstract of Paper E, published as Skog & Händel, IEEE T-ITS 12(4), 2011). | https://www.diva-portal.org/smash/get/diva2:12840/FULLTEXT01.pdf | 2026-09-28 | sources/skog_2007_gnss_ins_licentiate_summary.pdf | A |
| L5-S11 | E. Olson, "A passive solution to the sensor synchronization problem", *Proc. IEEE/RSJ IROS*, pp. 1059–1064, 2010. DOI 10.1109/IROS.2010.5650579. | https://april.eecs.umich.edu/pdfs/olson2010.pdf | 2026-09-28 | sources/olson_2010_passive_sensor_sync.pdf | A |
| L5-S12 | P. Furgale, J. Rehder, R. Siegwart, "Unified temporal and spatial calibration for multi-sensor systems", *Proc. IEEE/RSJ IROS*, pp. 1280–1286, 2013. DOI 10.1109/IROS.2013.6696514. | http://vigir.missouri.edu/~gdesouza/Research/Conference_CDs/IEEE_IROS_2013/media/files/0240.pdf | 2026-09-28 | sources/furgale_2013_unified_temporal_spatial_calib.pdf | A |
| L5-S13 | T. Foote (Open Robotics), "Clock and Time", ROS 2 design article. | https://design.ros2.org/articles/clock_and_time.html | 2026-09-28 | sources/ros2_design_clock_and_time.md | B |
| L5-S14 | E. R. Jellum, T. H. Bryne, T. A. Johansen, M. Orlandić, "The Syncline model — analyzing the impact of time synchronization in sensor fusion", *Proc. IEEE CCTA*, 2022 (IEEE Xplore 9966179); open copy arXiv:2209.01136v2. | https://arxiv.org/pdf/2209.01136 | 2026-09-28 | sources/jellum_2022_syncline.pdf | A |
| L5-S15 | J. D. Brouk, K. J. DeMars, "Kalman filtering with uncertain and asynchronous measurement epochs", *NAVIGATION* 71(3), 2024. DOI 10.33012/navi.652 (CC-BY). | https://doi.org/10.33012/navi.652 | 2026-09-28 | sources/brouk_2024_kf_asynchronous_epochs.pdf | A |
| L5-S16 | M. Mauthner, W. Elmenreich, A. Kirchner, D. Boesel, "Out-of-sequence measurements treatment in sensor fusion applications: buffering versus advanced algorithms", *Workshop Fahrerassistenzsysteme (FAS)*, Löwenstein/Hößlinsülz, pp. 20–30, 2006 (author version; peer-review status of this national workshop not stated, so graded C). | https://mobile.aau.at/~welmenre/papers/mauthner-fas06.pdf | 2026-09-28 | sources/mauthner_2006_oosm_buffering_vs_algorithms.pdf | C |
| L5-S17 | A. Harrison, P. Newman, "TICSync: Knowing when things happened", *Proc. IEEE ICRA*, pp. 356–363, 2011. DOI 10.1109/ICRA.2011.5980112. | https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.648.532&rep=rep1&type=pdf | 2026-09-28 | sources/harrison_2011_ticsync.pdf | A |
| L5-S18 | T. Qin, S. Shen, "Online temporal calibration for monocular visual-inertial systems", *Proc. IEEE/RSJ IROS*, pp. 3662–3669, 2018; open copy arXiv:1808.00692. | https://arxiv.org/pdf/1808.00692 | 2026-09-28 | sources/qin_2018_online_temporal_calib_vio.pdf | A |
| L5-S19 | J. Kelly, C. Grebe, M. Giamou, "A question of time: revisiting the use of recursive filtering for temporal calibration of multisensor systems", *Proc. IEEE MFI*, 2021 (IEEE Xplore 9591176); open copy arXiv:2106.00391v3. | https://arxiv.org/pdf/2106.00391 | 2026-09-28 | sources/kelly_2021_question_of_time_temporal_calib.pdf | A |
| L5-S20 | F. Tschopp, M. Riner, M. Fehr, L. Bernreiter, F. Furrer, T. Novkovic, A. Pfrunder, C. Cadena, R. Siegwart, J. Nieto, "VersaVIS—An open versatile multi-camera visual-inertial sensor suite", *Sensors* 20(5):1439, 2020. DOI 10.3390/s20051439 (published open-access version, CC-BY; page numbers refer to this 18-page PDF). | https://www.mdpi.com/1424-8220/20/5/1439/pdf (bot check blocks scripted download; file fetched from MDPI's CDN: https://mdpi-res.com/d_attachment/sensors/sensors-20-01439/article_deploy/sensors-20-01439.pdf) | 2026-09-28 | sources/tschopp_2020_versavis.pdf | A |
| L5-S21 | A. Geiger, P. Lenz, C. Stiller, R. Urtasun, "Vision meets robotics: The KITTI dataset", *Int. J. Robotics Research* 32(11):1231–1237, 2013 (author copy). | https://www.cvlibs.net/publications/Geiger2013IJRR.pdf | 2026-09-28 | sources/geiger_2013_kitti_dataset.pdf | A |
| L5-S22 | C. Bédard, I. Lütkebohle, M. Dagenais, "ros2_tracing: Multipurpose low-overhead framework for real-time tracing of ROS 2", *IEEE Robotics and Automation Letters* 7(3):6511–6518, 2022. DOI 10.1109/LRA.2022.3174346 (checked via Crossref); open copy arXiv:2201.00393v4. | https://arxiv.org/pdf/2201.00393 | 2026-09-28 | sources/bedard_2022_ros2_tracing.pdf | A |
| L5-S23 | K. J. Åström, R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, 2nd ed., Princeton University Press; online version 24 Jul 2020, Chapter 10 "Frequency Domain Analysis". | http://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-loopanal_24Jul2020.pdf | 2026-09-28 | sources/astrom_2020_feedback_systems_ch10_loop_analysis.pdf | A |
| L5-S24 | chrony project, "Frequently Asked Questions" (chrony 4.x). | https://chrony-project.org/faq.html | 2026-09-28 | sources/chrony_2025_faq.md | B |
| L5-S25 | chrony project, "Comparison of NTP implementations" (chrony 4.9, ntp 4.2.8p18, ntpsec 1.2.5, openntpd 7.9p1). | https://chrony-project.org/comparison.html | 2026-09-28 | sources/chrony_2025_comparison.md | B |
| L5-S26 | chrony project, "chrony.conf(5) Manual Page", version 4.6. | https://chrony-project.org/doc/4.6/chrony.conf.html | 2026-09-28 | sources/chrony_2024_chrony_conf_4.6.md | B |
| L5-S27 | linuxptp project, man pages ptp4l(8), phc2sys(8), ts2phc(8), tag v4.4 (March 2024). | https://github.com/richardcochran/linuxptp/tree/v4.4 | 2026-09-28 | sources/linuxptp_v4.4_ptp4l.8; linuxptp_v4.4_phc2sys.8; linuxptp_v4.4_ts2phc.8 | B |
| L5-S28 | Linux kernel documentation, "Timestamping" (networking/timestamping.rst) and "PPS - Pulse Per Second" (driver-api/pps.rst), tag v6.10. | https://github.com/torvalds/linux/tree/v6.10/Documentation | 2026-09-28 | sources/linux_v6.10_networking_timestamping.rst; linux_v6.10_pps.rst | B |
| L5-S29 | Linux man-pages project, clock_getres(2) / clock_gettime(2), man-pages-6.9. | https://git.kernel.org/pub/scm/docs/man-pages/man-pages.git/tree/man/man2/clock_getres.2?h=man-pages-6.9 | 2026-09-28 | sources/linux_manpages_6.9_clock_getres.2 | B |
| L5-S30 | u-blox AG, "GPS-based Timing: Considerations with u-blox 6 GPS receivers", Application Note GPS.G6-X-11007, 2011. Covers the older u-blox 6 generation; used here for general time-pulse principles only. | https://content.u-blox.com/sites/default/files/products/documents/Timing_AppNote_%28GPS.G6-X-11007%29.pdf | 2026-09-28 | sources/ublox_2011_gps_based_timing_appnote.pdf | B |
| L5-S31 | u-blox AG, "ZED-F9T-10B High accuracy timing module — Data sheet", UBX-20033635 R09, 2024. | https://content.u-blox.com/sites/default/files/ZED-F9T-10B_DataSheet_UBX-20033635.pdf | 2026-09-28 | sources/ublox_2024_zed_f9t_10b_datasheet.pdf | A |
| L5-S32 | ROS 2 geometry2 (tf2, tf2_ros) source: `buffer_core.cpp`, `buffer_core.hpp`, `cache.cpp`, `time_cache.hpp`, `message_filter.hpp`, branch humble at commit 404b7224d623d614f18fa9738dbf1716403d857e. | https://github.com/ros2/geometry2/tree/404b7224d623d614f18fa9738dbf1716403d857e | 2026-09-28 | sources/geometry2_humble_buffer_core.cpp; geometry2_humble_buffer_core.hpp; geometry2_humble_cache.cpp; geometry2_humble_time_cache.hpp; geometry2_humble_tf2_ros_message_filter.hpp | B |
| L5-S33 | ROS 2 documentation (Humble): "About tf2", "Learning about tf2 and time (C++)", "Traveling in time (C++)", "Using stamped datatypes with tf2_ros::MessageFilter", ros2_documentation commit 35b00f1f3c1ab7c14bf85e35fa895f9f580ea279. | https://github.com/ros2/ros2_documentation/tree/35b00f1f3c1ab7c14bf85e35fa895f9f580ea279/source | 2026-09-28 | sources/ros2doc_humble_about_tf2.rst; ros2doc_humble_learning_tf2_and_time_cpp.rst; ros2doc_humble_time_travel_tf2_cpp.rst; ros2doc_humble_tf2_message_filter.rst | B |
| L5-S34 | ROS 2 message_filters (Humble): `doc/index.rst`, `sync_policies/approximate_time.h`, `exact_time.h`, commit 14ffd199d23cdfcd7ed102de8328081a5ec8f5e6. | https://github.com/ros2/message_filters/tree/14ffd199d23cdfcd7ed102de8328081a5ec8f5e6 | 2026-09-28 | sources/message_filters_humble_index.rst; message_filters_humble_approximate_time.h; message_filters_humble_exact_time.h | B |
| L5-S35 | ROS 2 common_interfaces (Humble): `std_msgs/Header`, `sensor_msgs/Image`, `Imu`, `LaserScan`, `TimeReference`, commit 08434490d3e4abd37ccc5462cdde69b88853b5b8. | https://github.com/ros2/common_interfaces/tree/08434490d3e4abd37ccc5462cdde69b88853b5b8 | 2026-09-28 | sources/common_interfaces_humble_Header.msg; common_interfaces_humble_Image.msg; common_interfaces_humble_Imu.msg; common_interfaces_humble_LaserScan.msg; common_interfaces_humble_TimeReference.msg | B |
| L5-S36 | robot_localization (humble-devel): `doc/state_estimation_nodes.rst`, `src/ros_filter.cpp`, `src/filter_base.cpp`, commit 8696ee5a9e4f959fcaae37835dcf2ed12ead581b. | https://github.com/cra-ros-pkg/robot_localization/tree/8696ee5a9e4f959fcaae37835dcf2ed12ead581b | 2026-09-28 | sources/robot_localization_humble_state_estimation_nodes.rst; robot_localization_humble_ros_filter.cpp; robot_localization_humble_filter_base.cpp | B |
| L5-S37 | ros/diagnostics (ros2-humble): `diagnostic_updater/update_functions.hpp`, commit de779cfd3bff7975f158971c58bedf0581148f9a. | https://github.com/ros/diagnostics/blob/de779cfd3bff7975f158971c58bedf0581148f9a/diagnostic_updater/include/diagnostic_updater/update_functions.hpp | 2026-09-28 | sources/diagnostics_humble_update_functions.hpp | B |
| L5-S38 | ETH Zurich ASL, Kalibr wiki "Camera IMU calibration", wiki commit 73a2ba7e134dded9d3472e053c51c6f2d29133a4 (2024-08-12). | https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration | 2026-09-28 | sources/ethzasl_2024_kalibr_wiki_camera_imu_calibration.md | B |
| L5-S39 | Future Technology Devices International, "AN232B-04 Data Throughput, Latency and Handshaking", 2006. | https://www.ftdichip.com/Documents/AppNotes/AN232B-04_DataLatencyFlow.pdf | 2026-09-28 | sources/ftdi_2006_an232b04_latency.pdf | B |
| L5-S40 | Xsens (Movella), *MTi 600-series User Manual*, created 31 Oct 2023. | https://mtidocs.movella.com/mti-600-series-user-manual | 2026-09-28 | sources/xsens_2023_mti600_user_manual.pdf | A |
| L5-S41 | Xsens, *MT Low Level Communication Protocol Documentation*, MT0101P, Revision 2020.A, June 2020. | https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf | 2026-09-28 | sources/xsens_2020_mt_low_level_protocol.pdf | A |
| L5-S42 | Xsens, Xsens_MTi_ROS_Driver_and_Ntrip_Client (ros2 branch): `xsens_time_handler.cpp`, `xdacallback.cpp`, `param/xsens_mti_node.yaml`, commit e145fb5051447374925a656d7fd637ff07085efe. | https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/e145fb5051447374925a656d7fd637ff07085efe/src/xsens_mti_ros2_driver | 2026-09-28 | sources/xsens_ros2driver_e145fb5_xsens_time_handler.cpp; xsens_ros2driver_e145fb5_xdacallback.cpp; xsens_ros2driver_e145fb5_xsens_mti_node.yaml | B |
| L5-S43 | Velodyne LiDAR, *VLP-16 User Manual*, 63-9243 Rev. F (title page marked "DRAFT"), last updated 2022-03-07; hosted by Ouster. | https://data.ouster.io/downloads/velodyne/user-manual/vlp-16-user-manual-revf.pdf | 2026-09-28 | sources/velodyne_2022_vlp16_user_manual_revf.pdf | A |
| L5-S44 | ros-drivers/velodyne (ros2 branch), velodyne_driver: `driver.cpp`, `input.cpp`, `time_conversion.hpp`, `README.md`, commit 56fc178d2dad4b6d38c6a69aeb2435ff75503e52. | https://github.com/ros-drivers/velodyne/tree/56fc178d2dad4b6d38c6a69aeb2435ff75503e52/velodyne_driver | 2026-09-28 | sources/velodyne_ros2_driver.cpp; velodyne_ros2_input.cpp; velodyne_ros2_time_conversion.hpp; velodyne_ros2_driver_README.md | B |
| L5-S45 | Stereolabs, ZED SDK documentation (live pages for ZED SDK 5.x; the pages carry no version number): "Sensors Time Synchronization", "Using the Sensors API", and API reference "Video module" (`TIME_REFERENCE`). | https://docs.stereolabs.com/docs/development/zed-sdk/modules/sensors/time-synchronization ; https://docs.stereolabs.com/docs/development/zed-sdk/modules/sensors/using-the-api ; https://www.stereolabs.com/docs/api/group__Video__group.html | 2026-09-28 | sources/stereolabs_2026_zed_sensors_time_sync.md; stereolabs_2026_zed_sensors_api.md; stereolabs_2026_zed_api_video_module.md | B |
| L5-S46 | NVIDIA, "Orin Time Sync", DRIVE OS 6.0.6 Linux SDK Developer Guide (DRIVE AGX Orin), published 2023-01-31. | https://developer.nvidia.com/docs/drive/drive-os/6.0.6/public/drive-os-linux-sdk/common/topics/network_stub/time_sync_details.html | 2026-09-28 | sources/nvidia_2023_driveos606_orin_time_sync.md | B |
| L5-S47 | NVIDIA, "Jetson Orin Series" (software features), Jetson Linux Developer Guide r36.4.4, last updated 2026-01-16. | https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SO/JetsonOrinSeries.html | 2026-09-28 | sources/nvidia_2026_jetson_r36.4.4_orin_series_features.md | B |
| L5-S48 | NVIDIA, "Generic Timestamp Engine", Jetson Linux Developer Guide r36.3, 2024. | https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/SD/Kernel/GenericTimestampEngine.html | 2026-09-28 | sources/nvidia_2024_jetson_r36.3_generic_timestamp_engine.md | B |
| L5-S49 | F. Cristian, "Probabilistic clock synchronization", *Distributed Computing* 3:146–158, 1989. DOI 10.1007/BF01784024. | https://doi.org/10.1007/BF01784024 | 2026-09-28 | not downloaded (no open copy) | A |
| L5-S50 | IEEE Std 1588-2019, *IEEE Standard for a Precision Clock Synchronization Protocol for Networked Measurement and Control Systems*, IEEE, 2020; and J. C. Eidson, *Measurement, Control, and Communication Using IEEE 1588*, Springer, 2006. | https://standards.ieee.org/ieee/1588/6825/ | 2026-09-28 | not downloaded (no open copy; L5-S05 used as proxy) | A |
| L5-S51 | Y. Bar-Shalom, "Update with out-of-sequence measurements in tracking: exact solution", *IEEE TAES* 38(3):769–777, 2002; Y. Bar-Shalom, H. Chen, M. Mallick, "One-step solution for the multistep out-of-sequence-measurement problem in tracking", *IEEE TAES* 40(1):27–37, 2004. | https://doi.org/10.1109/TAES.2002.1039398 | 2026-09-28 | not downloaded (no open copy; described via L5-S15) | A |
| L5-S52 | I. Skog, P. Händel, "Time synchronization errors in loosely coupled GPS-aided inertial navigation systems", *IEEE Trans. ITS* 12(4):1014–1023, 2011. DOI 10.1109/TITS.2011.2126569. | https://doi.org/10.1109/TITS.2011.2126569 | 2026-09-28 | not downloaded (ResearchGate 403; abstract read in L5-S10) | A |
| L5-S53 | M. Li, A. I. Mourikis, "Online temporal calibration for camera–IMU systems: theory and algorithms", *Int. J. Robotics Research* 33(7):947–964, 2014. DOI 10.1177/0278364913515286. | https://intra.ece.ucr.edu/~mourikis/papers/Li2014IJRR_timing.pdf (author copy; live URL returned 401, retrieved from the Internet Archive: https://web.archive.org/web/2020id_/https://intra.ece.ucr.edu/~mourikis/papers/Li2014IJRR_timing.pdf) | 2026-09-28 | sources/li_2014_online_temporal_calib_camera_imu.pdf | A |
| L5-S54 | J. Kelly, G. S. Sukhatme, "A general framework for temporal calibration of multiple proprioceptive and exteroceptive sensors", *Experimental Robotics (ISER 2010)*, STAR 79, pp. 195–209, Springer, 2014. | https://doi.org/10.1007/978-3-642-28572-1_14 | 2026-09-28 | not downloaded (no open copy; described via L5-S18) | A |
| L5-S55 | J. R. Vig, "Quartz Crystal Resonators and Oscillators for Frequency Control and Timing Applications — A Tutorial", Rev. 8.5.3.6, January 2007 (mostly prepared at the US Army Communications-Electronics RD&E Center; approved for public release). Tutorial, not peer-reviewed; third-party mirror (the IEEE UFFC-hosted copy could not be reached). | https://www.rfseminar.nl/cms/wp-content/uploads/2019/03/Vig-tutorial_Jan_2007.pdf | 2026-09-28 | sources/vig_2007_quartz_oscillator_tutorial.pdf | B |
| L5-S56 | US Space Force / GPS Directorate, *NAVSTAR GPS Space Segment / Navigation User Segment Interfaces*, IS-GPS-200 Revision N, 1 Aug 2022. | https://www.navcen.uscg.gov/sites/default/files/pdf/gps/IS-GPS-200N.pdf | 2026-09-28 | sources/usspaceforce_2022_is_gps_200n.pdf | A |
| L5-S57 | V. Indelman, S. Williams, M. Kaess, F. Dellaert, "Factor graph based incremental smoothing in inertial navigation systems", *Proc. 15th Int. Conf. Information Fusion (FUSION)*, IEEE, 2012 (IEEE Xplore 6290565); extended version in *Robotics and Autonomous Systems* 61(8):721–738, 2013. | https://dellaert.github.io/files/Indelman12fusion.pdf | 2026-09-28 | sources/indelman_2012_factor_graph_incremental_smoothing_ins.pdf | A |
| L5-S58 | AUTOSAR, "Time Synchronization Protocol Specification", Document ID 897, AUTOSAR FO R22-11, 2022. | https://www.autosar.org/fileadmin/standards/R22-11/FO/AUTOSAR_PRS_TimeSyncProtocol.pdf | 2026-09-28 | sources/autosar_2022_prs_time_sync_protocol.pdf | A |
| L5-S59 | A. Frankó, G. Hollósi, "Settling issues in IEEE 802.1AS networks in PI based clock servos", *3rd Int. Workshop on Analytics for Service and Application Management (AnServApp 2023)*, at IFIP/IEEE CNSM 2023, IFIP. | https://dl.ifip.org/db/conf/cnsm/cnsm2023/1570947595.pdf | 2026-09-28 | sources/franko_2023_8021as_multihop_pi_servo.pdf | A |
| L5-S60 | T. Kronauer, J. Pohlmann, M. Matthé, T. Smejkal, G. Fettweis, "Latency analysis of ROS2 multi-node systems", *Proc. IEEE Int. Conf. Multisensor Fusion and Integration for Intelligent Systems (MFI)*, 2021; open copy arXiv:2101.02074v3. | https://arxiv.org/pdf/2101.02074 | 2026-09-28 | sources/kronauer_2021_ros2_latency_analysis.pdf | A |
| L5-S61 | ROS 2 rmw (humble branch): `rmw/include/rmw/types.h`, commit 566d17a97e87bc5336ce33fb025ce8e952a3326a. | https://github.com/ros2/rmw/blob/566d17a97e87bc5336ce33fb025ce8e952a3326a/rmw/include/rmw/types.h | 2026-09-28 | sources/rmw_humble_types.h | B |
| L5-S62 | Stereolabs, zed-ros2-wrapper tag v5.2.2 (commit b47c72779add6d58a4dba9c888b9ea529f6ae448): `zed_camera_component_main.cpp`, `zed_camera_component_video_depth.cpp`, `zed_wrapper/config/common_stereo.yaml`. | https://github.com/stereolabs/zed-ros2-wrapper/tree/v5.2.2 | 2026-09-28 | sources/zed_ros2_wrapper_v5.2.2_main.cpp; zed_ros2_wrapper_v5.2.2_video_depth.cpp; zed_ros2_wrapper_v5.2.2_common_stereo.yaml | B |
| L5-S63 | ros-navigation/navigation2 (humble branch): `nav2_util/src/robot_utils.cpp`, `nav2_costmap_2d/src/observation_buffer.cpp`, commit 3c3db59d6969d8ecee8e68468693d006397f4a0c. | https://github.com/ros-navigation/navigation2/tree/3c3db59d6969d8ecee8e68468693d006397f4a0c | 2026-09-28 | sources/nav2_humble_robot_utils.cpp; nav2_humble_observation_buffer.cpp | B |
| L5-S64 | Nav2 documentation (docs.nav2.org, current/rolling; the robot runs Humble, so defaults quoted from it are labelled rolling-only): costmap_2d `index.md` and `costmap_plugins/obstacle.md`, commit 588d37415e87eb083500d6c79aaed92ee1285f52. | https://github.com/ros-navigation/docs.nav2.org/tree/588d37415e87eb083500d6c79aaed92ee1285f52 | 2026-09-28 | sources/nav2docs_588d374_costmap_2d_index.md; nav2docs_588d374_obstacle_layer.md | B |
| L5-S65 | C. Bachhuber, E. Steinbach, "A system for high precision glass-to-glass delay measurements in video communication", *Proc. IEEE ICIP*, pp. 2132–2136, 2016; open copy arXiv:1510.01134. | https://arxiv.org/pdf/1510.01134 | 2026-09-28 | sources/bachhuber_2016_glass_to_glass_delay.pdf | A |
| L5-S66 | J. Nikolic, J. Rehder, M. Burri, P. Gohl, S. Leutenegger, P. T. Furgale, R. Siegwart, "A synchronized visual-inertial sensor system with FPGA pre-processing for accurate real-time SLAM", *Proc. IEEE ICRA*, pp. 431–437, 2014. DOI 10.1109/ICRA.2014.6906892 (author copy via the Internet Archive). | https://web.archive.org/web/2020id_/https://furgalep.github.io/bib/nikolic_icra14.pdf | 2026-09-28 | sources/nikolic_2014_synchronized_vi_sensor_fpga.pdf | A |
| L5-S67 | J. Zhang, S. Singh, "LOAM: Lidar odometry and mapping in real-time", *Robotics: Science and Systems (RSS)*, 2014. | https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf | 2026-09-28 | sources/zhang_2014_loam.pdf | A |
