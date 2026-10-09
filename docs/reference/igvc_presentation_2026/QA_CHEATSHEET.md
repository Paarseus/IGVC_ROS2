# Q&A Cheat Sheet — Autonomy (IGVC 2026 Preliminary Oral)
Judges score **"Response to questions" (100 pts)** = *short answers that address only the question.* Lead with the direct answer, then one sentence of evidence. Never bluff — every answer below is grounded in our code/docs.

## Most likely hard questions

**Q: How do you tell a 2-ft white pothole from a white lane line?**
A: By shape and context, in the camera segmentation. Lane lines are long, thin (~3-in) boundary stripes; a pothole is a compact solid white disc inside the drivable lane. We classify them differently — a pothole becomes a lethal cell to route around (lane-change), a lane is a soft boundary we follow. LiDAR sees neither (both flat), so this is camera-only — which is exactly why the camera layer matters.

**Q: What frame is your navigation actually running in right now — odom or the GPS map frame?**
A: The local obstacle-avoidance costmap already runs in **odom** (drift-free), which keeps MPPI's rollouts accurate. `bt_navigator` and the global costmap are still in map today; collapsing them to **map ≡ odom** is our committed, in-progress next step. We diagnosed that map drift breaks map-frame goals, so we're deliberately moving global control into odom — a design decision, not a patch.

**Q: Has your camera lane detection ever driven an avoidance, or was that LiDAR?**
A: The May 21 avoidance demo was **LiDAR-only**; the ZED semantic lane layer joined the live loop on May 29. The camera outputs a single-class non-asphalt mask into a Nav2 semantic layer (lanes + potholes); LiDAR is the always-on fallback so we're never blind on obstacles.

**Q: How big is the map-frame drift, and where's the number from?**
A: Measured, not rounded: ~2.7 cm/s at rest, peaking ~55 cm/s mid-turn, netting **~2.5 m vector error over a 70 s / 11 m run**, with map-EKF path excess of 13–37 m vs the local odom path across three bags. That's why control lives in odom.

**Q: You fuse unaided SBAS GPS that under-reports its covariance — how do you avoid snapping to a bad fix? Differential or absolute?**
A: GPS is **advisory only**, fused **differentially** into the map EKF behind a Mahalanobis rejection gate. Only the local EKF (IMU + wheel odom) drives real-time control, so GPS never moves the robot directly. Going to absolute fusion would first need a ~5 m covariance floor at the NavSatFix source — a documented prerequisite we have not applied, which is why control stays in odom.

**Q: The rubric wants simulation testing — show me your sim.**
A: We run a **Webots** sim (`avros_sim`, `cpp_campus` world) with the **full Nav2 stack at topic-level parity** to hardware — it validates the perception → planning → behaviour pipeline. The Webots model is Ackermann and our robot is a tracked diff-drive, so **controller and skid-steer tuning are validated on hardware** via field tests and bag replay. Sim proves the pipeline; the chassis proves the kinematics.

## Quick-fire
- **Why Navfn, not Smac/Hybrid?** Smac's ~2.3 m turning circle can't fit a 10-ft IGVC lane / 5-ft switchback; our tracked chassis has a 0 m turning radius — a holonomic Dijkstra planner is the right tool.
- **Why only 13–16 Hz, not 20?** Deliberate trade for full semantic-layer behaviour; max gap 0.146 s, well under the actuator's 500 ms timeout.
- **MPPI crashed early — why stable now?** Root cause was CPU starvation from RViz on the Jetson (load 10.75→3.5 when killed); moved visualization to a laptop and cut batch_size 1000→500.
- **Stuck against a barrel?** 3× escalating recovery (clear → wait → back up → crawl), 45 s watchdog (15 s margin vs IGVC's 60 s hold-up rule); we omit in-place spin (footprint sweep exceeds half a lane).
- **Why no RTK/NTRIP?** IGVC §I.2 forbids positioning base stations and NTRIP legality is unresolved — so we don't depend on it.

## Image provenance (if a judge asks "is that your screenshot?")
- **Ours / our data:** Webots sim frame (slide 4), the KPI table (slide 4), the field-test result. The ZED stereo camera and Velodyne VLP-16 are the actual COTS sensors on the vehicle.
- **Reference visualizations of the stack we run** (not our own captures): the segmentation overlay (slide 2, the kiwicampus semantic-layer we deploy), the Nav2 RViz costmap (slide 3), and the GPS mapviz view (slide 5). Say so plainly if asked — "that's a Nav2 reference; our live costmap looks the same."

## Cyber security (slide 6)
**Q: What's your single biggest cyber risk and what stops it today?**
A: Our default ROS 2 DDS config — a laptop on our Wi-Fi could publish commands. Today we limit Wi-Fi to team laptops and the judges hold a hardware wireless E-stop that cuts motor power independent of software. For production we'd enable DDS security (per-node certs, encryption) on a segmented VLAN.
**Q: If your software is compromised, can the robot still be stopped?**
A: Yes — three independent stops: a physical E-stop button, the judges' wireless relay, and an actuator software interlock that zeros drive on stale input. The hardware relay must be re-enabled by a human once tripped, so no software state can override it.
**Q: Why is GPS spoofing only "low" risk?**
A: We treat GPS as a per-step correction behind a 6σ EKF gate, never as absolute position — a slow-drift spoof can't move us far before the wheel/IMU dead-reckoning disagrees and the filter rejects it.

## ⚠️ Report-vs-live divergences — KNOW THESE before you present
The design report (submitted 2026-05-15) describes the *target* system; the robot has evolved. The slides follow the **report** (what judges read). Be ready for:
- **Perception pipeline.** Report + slides describe the **3-class HSV** pipeline (lane / barrel / pothole) that *separates* potholes from lanes. The robot's current **default is `sooner25`** (inverted-asphalt), switched in for cleaner detection on bright asphalt — it marks *all* non-asphalt (lanes AND potholes) as lethal, so both are still avoided, but it doesn't split them into classes. Both pipelines are built and **switchable via `ros2 param set`**. **Decide which you'll run at competition.** If you run sooner25, rephrase S2 to "we mark lanes and potholes as no-go regions" rather than "separate" them. Honest Q&A line: *"We have a swappable pipeline; HSV gives explicit pothole/lane classes, sooner25 is cleaner on bright asphalt — we pick per surface."*
- **MPPI rollouts.** Report says **2000 rollouts / 56 steps**; live config is **500 / 25** (cut 2026-05-28 for Jetson control-loop headroom). The slide deliberately says "trajectories at 20 Hz" (no count). If asked: *"500 live — we reduced from the report's 2000 for loop headroom on the Jetson."*
- **Cameras.** Report says **3× ZED X**; only the **front** ZED X is wired today (left/right are Phase 5). Slide says "ZED X" without a count.
- **MARVIN** is the robot's name per the report — use it.
