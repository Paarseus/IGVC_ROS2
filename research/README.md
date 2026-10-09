# Controls and Localization Research

An evidence base for making the robot's control and localization correct and precise, built from published research, manufacturer documentation and open-source robot stacks. Also covers a separate question: whether a different drivetrain architecture (D1-D5) would need less custom correction than the current tracked skid-steer chassis.

**This phase:** research only. Comparing it with our code, and any drivetrain recommendation, comes next.

## Contents

| Path | What it is |
|---|---|
| [RESEARCH_PLAN.md](RESEARCH_PLAN.md) | Topics, research questions, method and workflow schedule |
| [STANDARDS.md](STANDARDS.md) | Rules for sources, citations, evidence levels and verification |
| [TEMPLATE_TOPIC.md](TEMPLATE_TOPIC.md) | Structure every topic README follows |
| `topics/` | One folder per topic: findings, verification and downloaded sources |
| `references/` | Master reference list across all topics (built at the end) |
| `evidence/` | Our own measurements and earlier reviews (kept separate from the research) |
| `tools/` | Reusable research workflow (map → research → gap check → verify claims and sources → correct), its guide (`tools/README.md`) and the context file for these topics |

## Topics

| ID | Topic | Status |
|---|---|---|
| C1 | [Motor velocity control](topics/C1_motor_velocity_control/) | Verified (W1 + extension + full re-check W3b) |
| C2 | [Drive kinematics](topics/C2_drive_kinematics/) | Verified (W1 + extension + full re-check W3b) |
| C3 | [Command pipeline](topics/C3_command_pipeline/) | Verified (W2) |
| C4 | [Controller–vehicle interface](topics/C4_controller_vehicle_interface/) | Verified (W2 + targeted additions, re-checked W3b) |
| L1 | [Wheel odometry](topics/L1_wheel_odometry/) | Verified (W3) |
| L2 | [IMU and heading](topics/L2_imu_heading/) | Verified (W3) |
| L3 | [GNSS / RTK](topics/L3_gnss_rtk/) | Verified (W4) |
| L4 | [Sensor fusion](topics/L4_sensor_fusion/) | Verified (W4) |
| L5 | [Time synchronization](topics/L5_time_sync/) | Verified (W4) |
| X1 | [Reference stacks](topics/X1_reference_stacks/) | in progress (W5) |
| X2 | [Test and validation methods](topics/X2_test_validation_methods/) | in progress (W5) |
| D1 | [Swerve drive modules](topics/D1_swerve_drive/) | Verified (W7) — 153 findings, 43 sources |
| D2 | [Wheeled skid-steer / differential drive](topics/D2_wheeled_skid_steer/) | Verified (W7) — 134 findings, 38 sources |
| D3 | [Ackermann / car-like steering](topics/D3_ackermann_steering/) | Verified (W7) — 134 findings, 45 sources |
| D4 | [Omnidirectional (mecanum/omni) drive](topics/D4_omnidirectional_drive/) | Verified (W8) — 136 findings, 39 sources |
| D5 | [Tracked drive alternatives and fixes](topics/D5_tracked_alternatives/) | Verified (W8) — 97 findings, 27 sources |

**Drivetrain architecture decision:** all five topics (D1-D5) are now Verified — see [`docs/drivetrain_architecture_analysis_2026_10_06.md`](../docs/drivetrain_architecture_analysis_2026_10_06.md) at the repo root for the comparative synthesis and recommendation.

## Evidence

| Folder | Content |
|---|---|
| `evidence/field_2026_09_27/` | Field measurements from 2026-09-27: RTK, IMU heading and filter profiles, straight drives, odometry check, scripts and recordings |
| `evidence/desk_review_2026_09/` | Earlier code and configuration reviews of our stack (2026-09-25 and 2026-09-27) |
