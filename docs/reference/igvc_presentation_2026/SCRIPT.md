# IGVC 2026 AutoNav — Autonomy + Cyber Security · Speaker Script
**Speaker:** Parsa (autonomy/software lead) · **5 slides**
**Measured length:** ~375 spoken words ≈ **2:53 @130 wpm (2:41 rehearsed @140)** — well under the 3.5-min budget after the fluff trim.

> Engineer's register: lead with the decision, state the rationale, close with the measured number. No hype, no anthropomorphizing.
> Spine: **SEE → DECIDE → PROVE → LOCATE → SECURE**

> *Optional lead-in if you open the autonomy section (drop if the master deck already framed the course):*
> *"MARVIN runs the course with no prior map — it has to perceive and decide in real time. I'll cover perception, the driving logic, how we validated it, localization, and cyber security."*

---
## ① PERCEPTION *(~33 s)* — segmentation + sensors
"Two sensors fuse into one Nav2 costmap. The Velodyne VLP-16 LiDAR gives 3-D obstacle geometry at ten hertz, independent of color and lighting. The ZED X camera segments three classes: lane lines, barrels, and solid-white potholes. Potholes and lane lines are both white, so we separate them at the pixel level — **potholes avoided, lanes tracked**. Startup calibration adapts the vision thresholds to ambient lighting. Required obstacle detection: two meters; measured, **fifteen**, at full camera rate."

## ② DRIVING LOGIC *(~40 s)* — Nav2 costmap
"Planning and control are separate. Global planner: Navfn, a Dijkstra planner — over Smac-Hybrid, whose **two-point-three-meter turning radius won't fit a ten-foot lane**; the tracked chassis turns in place, so curvature constraints add nothing. Controller: MPPI, a sampling-based MPC at twenty hertz — it **re-plans around newly placed barrels** and rejects any rollout that enters a pothole. A dual EKF supplies pose; a recovery behavior tree under a forty-five-second watchdog handles dead-ends and switchbacks. Required obstacle reaction: two hundred fifty milliseconds; measured, **forty-seven**."

## ③ VALIDATION *(~28 s)* — Webots + KPI table
"Two-tier validation: a **Webots** simulation runs the full Nav2 pipeline; the **hardware** chassis confirms the numbers. Each KPI maps to an IGVC rule, target versus measured. Obstacle reaction: **forty-seven milliseconds against a two-hundred-fifty target**. May twenty-first field test, person stepping into the path: **seven recovery behaviors, zero collisions**."

## ④ LOCALIZATION *(~37 s)* — GPS / odom
"A dual EKF at thirty hertz: the local filter fuses IMU, wheel odometry, and ZED yaw; the global filter adds GPS. **No RTK** — IGVC rule I-point-two forbids positioning base stations — so GPS is **unaided two-to-five-meter SBAS**, fused **differentially behind a six-sigma outlier gate**. Per-step GPS deltas, not absolute fixes, keep position noise out of the costmap. **Safety-critical control runs in the drift-free odometry frame**; GPS only biases the long-range waypoints."

## ⑤ CYBER SECURITY *(~48 s)* — cyber table
"A **NIST risk assessment** — three top vulnerabilities.

**First, the network.** ROS 2's default DDS has no authentication, so any host on the Wi-Fi could publish a drive command. We restrict the network to team hardware; the **judges' wireless E-stop cuts motor power independently of software**.

**Second**, the web operator interface has no login — a **five-hundred-millisecond watchdog** zeros the drive when commands stop, with manual override.

**Third, GPS spoofing.** The filter takes GPS only as a bounded per-step correction and rejects physically impossible jumps.

Layered defense: **two hardware stops — the physical E-stop and the judges' wireless relay — plus a software interlock. No single software compromise can move the vehicle.**"

### Plain-English backing (for Q&A confidence)
- **Network / DDS:** ROS 2 nodes talk over the network with *no authentication by default* — an open channel; anyone on the same Wi-Fi can publish `/cmd_vel`. We don't re-architect DDS for the competition — we shrink the attack surface (closed network) and keep the hardware E-stop as the backstop. *Production:* DDS security (per-node certs, encrypted topics) on its own VLAN.
- **Web UI:** the operator controller is a web page with *no password* — convenient, but anyone on the LAN could take over. The 0.5 s watchdog, manual-override priority, and E-stop contain it. *Production:* TLS + per-operator certs, Foxglove bound to localhost.
- **GPS spoofing:** a cheap radio can broadcast a *stronger fake GPS signal*. We never trust GPS as absolute position — only a per-step correction behind a 6σ gate — so a spoof can't yank us off the lane. *Production:* multi-band receiver with RAIM, authenticated RTK, dead-reckoning cross-check.
- **The wireless E-stop is HARDWARE** — a relay held by the judges that physically cuts motor power. Never call it software.

---
## Delivery
- **Verbatim anchors:** "potholes avoided, lanes tracked" (S1); "re-plans around newly placed barrels" (S2); "control runs in the drift-free odometry frame" (S4); "two hardware stops … plus a software interlock — no single software compromise can move the vehicle" (S5).
- On S5, **slow down on each "First / Second / Third"** and point to the matching table row; let the judges read the production column while you talk.
- Cadence: this register reads a touch slower than conversational — land each measured number ("fifteen," "forty-seven," "six-sigma") rather than rushing past it.

## Timing options (≈2:53 @130 / 2:41 @140)
- **As written:** ~30–40 s of headroom under 3.5 min — room for the optional lead-in or a Q&A buffer.
- **To fill toward 3.5 if needed:** add the optional course-overview lead-in, and restore the S5 production-hardening detail verbally (it's on the slide).
