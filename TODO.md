# IGVC_ROS2 — TODO

Source of truth for outstanding work. Organized by priority. Each item should be specific enough that a fresh agent (or you in two weeks) can act on it without re-deriving the context.

If you check off items, **delete them** rather than striking through, to keep the file scannable.

---

## High priority — do soon

These are blockers, have measurable impact on field-test accuracy, or are required before nav2 can be trusted in real drives.

### Hardware / power

- [ ] **Dedicated 48 V → 19 V buck for Jetson** (separate from motor rail). Currently both Jetson and SparkMAXes share one 48 V→12 V buck; motor inrush sags the rail and brown-outs the Jetson. **Blocks safe field testing.** Persistent journald is enabled for post-crash forensics in the meantime.

### Localization / kinematics calibration

- [ ] **5 m straight-line `m_per_rev` calibration.** Mark a 5 m line with tape, drive forward, read EKF `pose.position.x` at start vs end. If reported is 5.05 m, `m_per_rev` is 1 % too small — adjust by ratio. Current value `0.01994 m` derived from spec (π × 80.85 mm pulley ÷ 12.75:1 gearbox), untested. Largest unmeasured contributor to position drift. *(360° spin done 2026-04-28: wheels +0.7 % vs IMU; effective track ≈ 0.7416 m; EKF result 358.34° / acceptable. Translational accuracy still untested.)*
- [ ] **Measure ZED front camera mount offset in URDF.** Currently placeholder `1.0 0 0.9` in `avros.urdf.xacro`. Tape-measure from `base_link` (geometric center of robot at ground level) to camera optical sensor — three numbers (x forward, y left, z up), 5 mm precision is fine. Every camera-derived costmap cell is offset by mount error; most visible during tight obstacle maneuvers.
- [ ] **Calibrate GNSS antenna lever arm in `xsens.yaml`.** Currently `[0.0, 0.0, 0.0]` — assumes antenna is at the IMU. Tape-measure from IMU center to GPS antenna's phase center (marked on housing) in the IMU body frame. Wrong lever arm injects yaw rate as fake translation in GPS-fused pose; visible as the global pose drawing a circle when the robot spins in place.
- [ ] **Measure imu / velodyne mount positions in URDF.** Currently approximate. IMU position has small impact on local nav (most contributions zeroed in `imu0_config`); larger impact when GPS lever arm is tightened, since these are referenced together. Velodyne position matters when LiDAR is reintroduced into local_costmap.

### Sensors / GPS

- [ ] **Verify RTK FIXED/FLOAT status outdoors.** NTRIP creds are configured (`ntrip.earthscope.org:2101`, mountpoint `PSDM_RTCM3P3`) and `/rtcm` is streaming at ~5 Hz, but indoors `/gnss` `status.status: -1` (no satellites). Outside, watch the status climb from -1 → 0 (single) → 1 (DGPS) → 2 (RTK FLOAT) → 4 (RTK FIXED) over a few minutes with sky view. Required before relying on GPS-anchored map frame.

### Perception accuracy

- [ ] **Tighten HSV `pothole_*` thresholds in `pipelines/hsv.py`.** Concrete walkway is being misclassified as `pothole` at ~14 % of frame in outdoor field tests (verified 2026-04-28 by overlay snapshot). Pipeline supports live tuning via dynamic params per commit `ace533e`. Tighten H/S/V ranges to exclude medium-gray concrete shadows. Without this, `nav2`'s costmap has large false-positive obstacle blobs in front of the robot.
- [ ] **Re-tighten `max_obstacle_distance` back to 5–8 m** in semantic_layer config (currently 100 m for bench-test). Far observations create huge bounds rectangles that update slowly and sometimes contain stale data.

---

## Medium priority — improvements + cleanup

Tasks that aren't blocking field testing but improve correctness, simplicity, or developer experience.

### Architectural fixes (vs workarounds)

- [ ] **Write `kiwicampus_align_purge_clocks.patch`** — proper fix for the dual-clock decay bug we worked around 2026-04-28 by bumping `tile_map_decay_time: 1.5 → 5.0`. The bug: `bufferSegmentation` purges with `cloud_time_seconds` (sensor stamp) but `updateBounds` purges with `node->now()` (wall clock); the same observation gets evaluated against two different "now"s. Fix: change `semantic_segmentation_layer.cpp:339` to use the buffer's last cloud stamp instead of `node->now()`. Once applied, decay reverts to kiwicampus default 1.5 in `perception_test_params.yaml` and `nav2_params_humble.yaml` (×3). Add to `scripts/apply_kiwicampus_patches.sh` after `kiwicampus_pr2_mutex.patch`.
- [ ] **Re-evaluate `twist0: /filter/twist` removal from `ekf.yaml`.** Removed 2026-04-28 on inference (CLI `ros2 topic echo` refused the topic due to dual-publisher type conflict). On reflection that was a CLI limitation, not a confirmed runtime fault — EKF was bound to the correct `TwistWithCovarianceStamped` publisher. Re-add the block, A/B test EKF position accuracy with vs without it, decide based on data not inference. Compare `/odometry/filtered` to `/odometry/global` (GPS-anchored) over a fixed drive path.
- [ ] **Investigate `/filter/twist` dual-publisher origin.** `ros2 topic info /filter/twist --verbose` should name both nodes. Tied to the previous TODO — once we know which publisher is which, removal/keep decision becomes obvious. Even if we keep it removed, fixing the source-side duplicate is correct.

### Drivetrain calibration / tuning

- [ ] **`track_width_m` fine-tune.** 360° spin on 2026-04-28 showed effective track ≈ 0.7416 m vs configured 0.7366 m (~0.7 % wider). Bump `actuator_params.yaml` value to 0.7416 if you want wheel-derived yaw to match IMU more closely. Low practical impact — EKF weights IMU more for yaw — but easy improvement.
- [ ] **Phase 7 BURN persistence verification.** Manual power-cycle readback of SparkMAX gains after `BURN` command. Confirms PID gains survive reboot.
- [ ] **Tune `heading_kp` / `yaw_rate_kp` if drift or wobble appears under real driving.** Default values are bench-tuned; field driving may need adjustment.
- [ ] **Loosen right track (optional).** Phase 3 testing showed 2× stiction asymmetry vs left.

### Nav2 / full-stack

- [ ] **Test full Nav2 navigation stack after power rail is fixed.** `navigation.launch.py` brings up everything. Need the buck fix first (motor commands during planning could brown-out Jetson).
- [ ] **Commit SSL cert paths for Jetson** in `webui_params.yaml` (currently only set locally on Jetson, not in source).
- [ ] **Run `vcs import src < avros.repos` on Jetson** to standardize source dependencies. Some packages may currently be cloned outside the manifest.
- [ ] **Update Jetson git auth** — currently configured for a teammate (`Ryan-Simpson`), so `git push` from the Jetson fails 403. Either set up `gh auth login` on Jetson with a Paarseus PAT, or switch the remote to SSH and add the Jetson's SSH key to the Paarseus account.

### Localization stack verification

- [ ] **Test localization stack (EKF + navsat) end-to-end.** Drive a known closed loop outdoors, verify `/odometry/global` returns to start within the GPS uncertainty bound. EKF #1 alone was verified 2026-04-28 (14 m drive, position tracks); EKF #2 + navsat full pipeline still untested.

---

## Low priority — future / docs

Useful but not on any critical path.

- [ ] **Document kiwicampus `tile_map` `frame_id="map"` mislabel.** The `/local_costmap/<source>/tile_map` viz topic is published with hardcoded `frame_id: "map"` regardless of the buffer's actual `global_frame_` (which is `odom` for a local_costmap). Coordinates *in* the message are correct; the label is wrong. Add to `patches/README.md` (or create one) so the next person debugging spatial issues in Foxglove/RViz doesn't waste time on this.
- [ ] **Phase 5: scale perception to 3 cameras (front + left + right)** with kiwicampus semantic_layer. Decide whether to drop RealSense from VoxelLayer once ZED coverage is full.
- [ ] **Replace forward-Euler pose integration upstream** if you ever encounter it in another diff-drive publisher. (We already fixed it in `actuator_node.py:_publish_odom` 2026-04-28.)

---

## Done in 2026-04-28 session (for context)

For the historical record — these were resolved in commits `a750e37` and `1587259`. Don't re-do.

- Wheel odom diagonal covariance + midpoint integration (`actuator_node.py`)
- `odom0: /wheel_odom` added to EKF #1, `twist0: /filter/twist` removed from `ekf.yaml`
- `tile_map_decay_time: 1.5 → 5.0` in `perception_test_params.yaml` and `nav2_params_humble.yaml`
- NoMachine GDM black-screen fix (mask `gdm.service`, `multi-user.target` default)
- Detailed changelog at `docs/CHANGELOG_2026-04-28.md`

---

*Last updated: 2026-04-28 by session covering wheel-odom EKF fusion, costmap dual-clock decay workaround, and NoMachine fix. See `docs/CHANGELOG_2026-04-28.md` for the full session writeup.*
