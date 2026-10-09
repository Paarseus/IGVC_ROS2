# docs/ index

Dated analysis/investigation docs, grouped by month (one folder per `YYYY-MM`, newest first below). Non-dated living/reference material is under `reference/` instead — those get updated in place, not superseded by a newer dated doc.

Reorganized 2026-10-09 (see `2026-10/repo_sync_audit_and_cleanup_plan_2026_10_09.md`) — previously ~70 files/dirs sat flat under `docs/`. Add new dated work to the current month's folder (create a new one if the month doesn't exist yet); add new living docs (protocols, standards, recipes) to `reference/`.

## 2026-10

- `drivetrain_architecture_analysis_2026_10_06.md` — drivetrain architecture deep-dive
- `obstacle_waypoint_test_plan_2026_10_02.md`, `nav_test_2026_10_02/` — waypoint/obstacle nav test plan + results
- `avros_lidar_layout_and_standard_2026_10_09.md` — avros_lidar vs. STVL comparison, package-layout recommendation
- `repo_sync_audit_and_cleanup_plan_2026_10_09.md` — this cleanup effort's own audit + plan

## 2026-09

- `imu_usb_latency_2026_09_30.md` — Xsens FTDI latency-timer fix
- `drive_tuning_2026_09_28/` — SPARK MAX FW26 / Teensy v2d tuning campaign (bench + ground), firmware finalization, root-cause analyses

## 2026-06

- `igvc_autonav_tuning_2026_06_01.md` — competition config tuning (freeze-behind-barrel fix)
- `cv_sooner25_tuning_2026_06_01/`, `cv_yolopv2_2026_06_01/`, `cv_canny_pipeline_2026_06_01/` — perception pipeline evaluations (adaptive/sooner25 kept, YOLOPv2 marginal, Canny kept off)

## 2026-05

- `skid_steer_kinematics_findings_2026_05_18.md` — Mandow skid-steer correction findings
- `next_session_test_plan_2026_05_19.md`, `session_2026_05_19_mppi_planner_costmap_gps.md` — MPPI/costmap/GPS session planning
- `lidar_obstacle_avoidance_test_2026_05_21.md` — lidar obstacle avoidance test
- `phase2_session_2026_05_26.md` — phase 2 session notes
- `session_2026_05_27_yaw_diag_field_plan.md`, `yaw_diag_analysis_strategy.md`, `yaw_diag_decision_thresholds.md`, `yaw_diag_session_2026_05_27/`, `yaw_diag_session_2026_05_28/`, `yaw_diag_session_LOG.md`, `yaw_diag_patches/` — yaw-drift diagnosis investigation (full cluster)
- `fresh_audit_2026_05_29.md`, `cv_costmap_deep_analysis_2026_05_29.md`, `lidar_rotation_phantom_diagnosis_2026_05_29.md`, `lidar_rotation_phantom_fix_plan_2026_05_29.md`, `phase0_field_test_2026_05_29.md`, `phase0_field_test_2026_05_29_scripts/` — 2026-05-29 field test day
- `autonav_bt_mission_review_2026_05_30.md`, `costmap_obstacle_persistence_analysis_2026_05_30.md`, `deskew_sensorframe_verification_2026_05_30.md`, `obstacle_stall_rca_2026_05_30.md`, `rtk_integration_change_plan_2026_05_30.md` — 2026-05-30 field test day
- `camera_costmap_flicker_analysis_2026_05_31.md`, `cv_adaptive_debug_2026_05_31/`, `cv_onnx_research_2026_05_31/`, `launch_files_analysis_2026_05_31.md`, `multicam_grayscale_analysis_2026_05_31.md`, `perception_framedrop_rca_2026_05_31.md`, `semantic_layer_height_gate_2026_05_31.md` (+ `.patch`), `xsens_mag_profile_2026_05_31_patches/` — 2026-05-31 perception/camera investigation day
- `local_costmap_rate_finding_2026-05-12.md` — local costmap update-rate finding
- `CODE_REVIEW_2026-05-01.md` — code review
- `motor_sync_logs/`, `review_2026-05/` — supporting logs/review material

## 2026-04

- `CHANGELOG_2026-04-23.md` ... `CHANGELOG_2026-04-29.md` — early-phase daily changelogs

## reference/ (not dated — living docs, updated in place)

- `NEXT_PR.md` — running log of unreleased changes; becomes the next PR description
- `igvc_rules/`, `igvc_2026_gps_waypoints.md`, `mdot_cors_msrn_port_scheme_20210415.txt` — competition rules, surveyed waypoints, NTRIP caster reference
- `winners_research/` — past IGVC AutoNav winners' writeups + data (merged 2026-10-09 from the root-level `igvc_winners_research/`, which had drifted into a second copy)
- `architecture_decision_global_frame.md`, `gps_datum_history.md`, `zed_vio_frame_analysis.md` — standing architecture decisions/derivations
- `field_test_plan.md`, `V_vehicle_integration_test_plan.md`, `W_vehicle_integration_test_plan.md` (W supersedes V), `phase_telemetry_recipe.md`, `PLAN_perception_phase4_phase5.md` — test plans
- `F_firmware_characterization_report.md`, `nav2_bringup_troubleshooting.tex`, `nav2_testing_troubleshooting.tex`, `nav2_route_integration_report.{tex,pdf}`, `perception_latency_investigation.{tex,pdf}` — standing reports
- `xsens_mt_manager_setup.md`, `foxglove_layout.json`, `avros_nav_demo.gif`, `igvc_presentation_2026/` — setup reference / media
