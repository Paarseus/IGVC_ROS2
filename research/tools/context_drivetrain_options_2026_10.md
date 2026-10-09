# Research context: drivetrain architecture alternatives

Used by `research_topics.workflow.js` through `contextFile`. It sets the scope and the background for judging relevance. Agents never write about this system.

**Scope:** this is a hardware-architecture selection question, not a tuning question. For each drivetrain type, cover general kinematics, control and precision principles first (how the architecture produces motion, how odometry is derived, what limits precision and repeatability), then lessons from how other vehicles and robots use it (competition robotics, agricultural and field robots, automotive, planetary rovers, industrial AGVs — say what vehicle type a finding comes from), then specific off-the-shelf products/kits relevant to a mid-size (roughly 0.8 m radius, well under 100 kg) outdoor ground vehicle. Named candidate products are starting points, not a ceiling — actively look for other vendors and designs in the same category.

**Why this research exists:** the project's current drivetrain is a tracked skid-steer chassis, and a large amount of engineering effort has already gone into making it deliver precise, predictable motion to the autonomy stack (empirical skid/slip correction, motor-controller PID retuning across several field sessions, heading-hold logic, etc.). The team is deciding whether a different drivetrain architecture would reach precise, repeatable motion with substantially less of that custom correction and tuning burden, while still suiting outdoor grass/dirt/gravel terrain and integrating cleanly with a standard ROS 2 Nav2 autonomy stack. This phase is research only — comparing the findings against the current system and making a recommendation is a later phase.

**Background (for relevance only):** an IGVC AutoNav-class autonomous ground robot with:
- currently: a tracked skid-steer chassis (AndyMark Raptor), REV SPARK MAX controllers and NEO brushless motors driven through a CAN bridge
- ROS 2 Humble, with the Nav2 MPPI controller
- robot_localization (dual EKF + navsat_transform), wheel/track odometry, an IMU, and GNSS/RTK
- a Jetson-class onboard computer
- competition terrain: outdoor grass, dirt and gravel, lanes roughly 2-3 m wide with obstacles, requiring tight maneuvering including near-zero-radius turns
- small student engineering team, limited budget and build/maintenance time, needs to arrive at competition with a drivetrain whose control behavior is well-understood and reliable

**What "good" looks like for this research:** findings that let a later synthesis compare architectures on: motion/odometry precision and repeatability, how much custom kinematic correction or control-loop work each typically needs versus standard ROS 2 / Nav2 support, outdoor terrain traction and durability (grass, mud, dirt, water, debris), turning/maneuvering capability relevant to tight lanes and obstacle avoidance, cost and mechanical/electrical complexity, and maintainability for a small team.
