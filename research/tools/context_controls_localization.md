# Research context: controls and localization

Used by `research_topics.workflow.js` through `contextFile`. It sets the scope and the background for judging relevance. Agents never write about this system.

**Scope:** cover general principles, methods and lessons first: control and estimation theory, calibration, testing, and how other robots and vehicles do it (field, agricultural and planetary rovers, automotive, AGVs). These are often the most useful. Then add material specific to the hardware and software below, as one sub-section.

**Background (for relevance only):** an IGVC AutoNav ground robot with:
- a tracked skid-steer chassis (AndyMark Raptor)
- REV SPARK MAX controllers and NEO brushless motors, driven through a Teensy CAN bridge
- ROS 2 Humble, with the Nav2 MPPI controller
- robot_localization (dual EKF + navsat_transform)
- an Xsens MTi-680G GNSS/INS with RTK over NTRIP
- a Jetson Orin
