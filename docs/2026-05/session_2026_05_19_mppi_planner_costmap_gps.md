# Session Log — 2026-05-19: MPPI, Planner Hardening, Costmap Clearing & GPS Drift

Long live-on-Jetson session. Tested MPPI end-to-end, bulletproofed the planner,
diagnosed the global-costmap phantom-accumulation problem and applied fixes,
analyzed the close-range LiDAR clearing failure, and tuned the EKF against GPS drift.

All work done live on the Jetson (`ssh jetson`, workspace `~/IGVC`). Config edits
made in this repo and `scp`'d to the Jetson (install→build→src are symlinks, so no
rebuild needed for YAML).

> ## ⚠️ STATUS: WORK IN PROGRESS — costmap smearing NOT fully resolved
>
> The smearing/phantom-accumulation is **still being worked on.** The fixes in
> Sections 6–8 (global STVL decay, close-range min_range/frustum tuning, EKF
> smoothing) were validated **STATIONARY ONLY** and **only partially**:
> - The global-costmap accumulation test (§6) was a stationary 30 s measurement.
> - The close-range clearing fix (§7) self-detect check passed, but the
>   **close-vs-far clearing re-test was NOT run** (blocked by the Xsens USB drop).
> - The EKF smoothing (§8) was **NOT measured** (also blocked by the Xsens drop).
>
> **We have NOT tested MOVEMENT with any of these fixes.** The only motion tests
> (MPPI 3 m drive, drive-past, the rear-approach that got trapped) were run on the
> **OLD config, before** the global-STVL/min_range/EKF changes. Whether the
> smearing is actually gone *while driving* — and whether the new config holds up
> under motion + GPS drift — is **untested and unverified.** Treat §6–8 as
> "applied, plausibly better, not proven." Next session must re-test stationary
> clearing AND a moving drive-past before trusting these.

---

## 1. MPPI smoke test (PASS, with a tracking caveat)

Drove the chassis under full Nav2 MPPI control.

- **3 m forward goal** reached: traveled 3.40 m, ended 0.14 m from goal once
  `xy_goal_tolerance` was tightened 2.0 → 0.3 m (the loose 2.0 m had it stopping
  ~1.9 m short and still reporting SUCCEEDED).
- Speed obeyed the cap (≤ 0.4–0.5 m/s in the supervised runs).
- **Obstacle avoidance in motion works**: with a person 3 m ahead, MPPI detoured
  with **80 cm planned clearance / 75 cm maintained during motion**, 0 lethal-cell
  crossings.

**Caveat — yaw tracking:** on the open 3 m drive the chassis curved ~**5°/m**
(17.6° over 3 m) and MPPI fought it with sustained corrective `wz` (124 negative
vs 38 positive samples). Root: right-track friction asymmetry + Mandow multiplier
(1.19) calibrated indoors. Re-calibrate Mandow for the test surface; see
`docs/skid_steer_kinematics_findings_2026_05_18.md`.

**Also found:** `ax_max/ax_min/az_max` in `nav2_params_humble.yaml` are **silently
ignored** by Humble's MPPI (the controller's param list has no `FollowPath.a*`
entries). The 2026-05-19 accel-sync commit (7b8f782) has no effect; the actuator
slew limiter is what actually clamps accel. Left as-is (harmless), flagged here.

---

## 2. Planner bulletproof suite (PP1–PP8) — ALL PASS

Navfn (`nav2_navfn_planner`), all via `ComputePathToPose` (no motion):

| Phase | Result |
|---|---|
| PP1 goal-space sweep (8 dirs × 3 dist) | PASS — uniform, ratio ≤1.1 except sub-inflation-radius (1 m) goals |
| PP2 pathological inputs (NaN, huge, bad/empty frame, zero-quat, 200 m OOB) | PASS — clean ABORTED, **planner never crashed**, stayed active |
| PP3 stress (20 rapid, cancel, concurrent) | PASS — 18/20 rapid, RSS +1.5 MB (no leak) |
| PP4 performance | PASS — 30 m plans as fast as 1 m; the ~1.4 s is action round-trip overhead, real planning is sub-second |
| PP5 determinism | PASS — identical paths for identical start+goal |
| PP6 obstacle routing | PASS — 80 cm clearance, 156 cm detour, 0 crossings |
| PP7 no-path/trapped | PASS — ABORTED in 1.4 s (no hang), recovers immediately |
| PP8 replanning under change | PASS — different paths for clean vs cluttered map |

---

## 3. GPS drift — the root cause behind most costmap problems

Robot **stationary**, yet `/odometry/global` (map frame) drifted **8.2 cm/s**
early, settling to **3.8 cm/s**. NMEA fix quality = **1** (unaided GPS, no SBAS).
Local EKF (`/odometry/filtered`, odom frame) was rock-stable.

This drift is the source of: the costmap "rolling" in RViz, the obstacle "smear
toward the car," and the rear-approach navigation failure (a 36 s drive that
got trapped spinning at −1.74 rad/s after phantom obstacles boxed it in;
global lethal cells grew **5878 → 24567** in that run).

### Diagnosis chain (all isolated to single stages)
- D1 `/gnss` raw drift ≈ D2 `/odometry/gps` ≈ D3 `/odometry/global` ≈ D4 `map→odom`
  → the noise enters at the **GPS receiver**, not the EKF/navsat.

---

## 4. SBAS investigation (the real source fix — NOT yet applied)

SBAS (WAAS) would flip fix quality 1→2 and roughly halve the drift. It's
geostationary satellites, **IGVC-legal** (not a base station; RTK *is* forbidden).

**Hardware constraint discovered:** the MTi-680G + 12-pin→USB cable exposes a
**single** serial port (`/dev/ttyUSB0`, Xsens MTi Converter 2639:0301) speaking
Xbus. The official Movella SBAS procedure (BASE article + Supercharge MTAN001-A)
needs u-center connected to the u-blox on its **own UART**, which the bare module
doesn't expose. So u-center cannot be used on this hardware.

**The working path** (not yet executed): `ForwardGnssData` (Xbus 0xE2) transparent
mode — send the UBX-CFG-VALSET enabling SBAS to the internal u-blox *through* the
MTi via MT Manager's "Device Data View → Message" field, plus the article's Step 3
`SetGnssReceiverSettings` → "Generic u-blox Receiver" (`FA FF AC 0A 00 06 00 02 00
04 00 00 00 00 3F`) so the MTi stops overwriting the u-blox config at boot.
Generated hex (enable SBAS, persist to flash):
`FA FF E2 16 B5 62 06 8A 0E 00 00 07 00 00 20 00 31 10 01 05 00 31 10 01 4E DC 7A`
(keys `CFG-SIGNAL-SBAS_ENA 0x10310020`, `CFG-SIGNAL-SBAS_L1CA_ENA 0x10310005`).

**Tools installed on the laptop (x86_64; Jetson is aarch64 — MT Suite is x86_64-only):**
MT Software Suite 2026.0 (`/opt/movella/mtmanager-2026.0`, launcher `mtmanager`)
and u-center v25.06 under Wine (`u-center`). libicu66 sideloaded for MT Manager.

Also noted: the driver's NavSatStatus mapping (`gnsspublisher.h:80-95`) is wrong —
it sets `STATUS_SBAS_FIX` only on RTK_FLOAT, never on real SBAS. So verify SBAS via
raw NMEA `$GPGGA` quality field (1→2), not `/gnss.status`.

---

## 5. LiDAR health (PASS)

`/velodyne_points`: 20.5 Hz, ~18 k pts/scan, organized, fresh stamps, **full 360°
coverage**. (A single-scan check first looked like a 180° blind spot — that was an
artifact: the VLP-16 spins at 10 rev/s but publishes at 20 Hz, so each *cloud* is a
~180° half-sweep. Over consecutive scans, and in the costmap, coverage is full.)

---

## 6. Costmap clearing diagnosis + FIX (applied this session)

**The asymmetry:** local costmap = STVL (`SpatioTemporalVoxelLayer`) with
`voxel_decay` + `decay_acceleration` → cells **decay in ~1 s** even off-ray. Global
costmap = plain `ObstacleLayer`, **no decay**, raytrace-only → off-ray cells (from
GPS-drift map→odom wobble) **persist forever** → accumulation.

Measured live: after a person stepped away, **local ROI cleared in ~1 s; global
held 14–17 cells for 25 s** (never cleared).

### Fix applied (`nav2_params_humble.yaml`, global_costmap)
- `ObstacleLayer` → **STVL** (`stvl_layer`) with time decay (mirrors local)
- rolling window **100 → 40 m** (phantoms fall off the trailing edge)
- `update_frequency` 5 → 10 Hz, `publish_frequency` 0.5 → 2 Hz

**Result (STATIONARY ONLY):** stationary 30 s test, global lethal total went
**2858 → 2360 (bounded, decaying)** vs the old **5878 → 24567 (4× pile-up)**.
Accumulation looks fixed *while parked* — **not yet tested in motion** (a moving
drive smears differently as the rolling window + GPS drift interact). WIP.

---

## 7. Close-range clearing analysis + FIX (applied this session)

User observed the smear is **worst when standing close, in front**. Deep analysis
(STVL source + geometry + live data) found **four stacked near-zone effects**:

1. **Hard blind ring 1.0 m** — `velodyne.yaml min_range:1.0` drops all returns <1 m
   → can't mark *or* clear there.
2. **Vertical near-cone** — Velodyne at **0.71 m** above ground, lowest beam (−15°)
   reaches ground at **2.67 m**. Close, low voxels fall **below** the STVL frustum
   (`IsInside()` false) → excluded from fast frustum decay → only slow timeout.
   No `vertical_fov_padding` was set.
3. **Occlusion** — a close body blocks the background; the cloud has no points in
   that direction → no raytrace clearing endpoint.
4. **GPS drift** smears whatever survives on the slow timeout path (~40 cm at
   8 cm/s × 5 s).

### Fix applied
- `velodyne.yaml`: `min_range` **1.0 → 0.7 m**. (Tried 0.4 first; it **self-detected
  a robot mount/payload** at 0.4–0.6 m / ~0.6 m height — 45 phantom lethal cells.
  0.7 m clears that structure while still seeing 30 cm closer than 1.0 m.)
- both STVL layers (local + global): `vertical_fov_padding: 0.1` (widen near cone),
  `voxel_decay` 5 → **2 s**, `decay_acceleration` 5 → **10**.

**Verified:** with min_range 0.7, near-range self-detect = **0 points, 0 cells**.

---

## 8. EKF smoothing vs GPS drift (applied this session)

`ekf.yaml` `ekf_filter_node_map`:
- x/y `process_noise_covariance` **1.0 → 0.1** — high value made the map EKF chase
  every noisy GPS fix (the 8 cm/s jitter). Lower → leans on smooth prediction,
  follows GPS slowly → steadier `map→odom`.
- `odom0_pose_rejection_threshold` **6.0 → 4.0** — drop multipath spikes sooner.

Note: this reduces *jitter*, not the *net* GPS drift (source-limited; needs SBAS).
Combined with the STVL decay, a smoother frame means less smear per decay window.

---

## 9. Config changes committed

| File | Change |
|---|---|
| `nav2_params_humble.yaml` | global ObstacleLayer→STVL+decay, 100→40 m window, 10 Hz / 2 Hz; both STVL layers: voxel_decay 2 s, decay_acceleration 10, vertical_fov_padding 0.1 |
| `velodyne.yaml` | min_range 1.0 → 0.7 m |
| `ekf.yaml` | map EKF x/y process noise 1.0 → 0.1, GPS rejection 6.0 → 4.0 |

---

## 10. Open items / next session

> **Top priority: the smearing fix is WIP and movement is untested.** Re-validate
> §6–8 stationary, then run a moving drive-past, before relying on the new config.


- **Xsens USB drop:** the driver intermittently fails to connect ("No MTi device
  found") after rapid stack restarts even with the device present + port free.
  Recurs as the stuck-USB issue → **physical USB power-cycle of the Xsens** is the
  reliable fix (ROS-level restart doesn't always recover it). Stack has no IMU /
  no `map→odom` until then.
- **Validate EKF tune:** measure stationary drift after the tune (blocked above by
  the Xsens drop). Expect steadier `map→odom`, similar net drift.
- **Re-run close-vs-far clearing (check b):** confirm the near-zone smear is gone
  with the user standing close then stepping away (was blocked by the Xsens drop).
- **SBAS:** apply via MT Manager `ForwardGnssData` (Section 4) — the real drift fix.
- **Mandow re-calibration** for the test surface (yaw drift ~5°/m, Section 1).
- **Periodic global-costmap clear** as a belt-and-suspenders backstop (BT recovery
  node or timer calling `clear_entirely_global_costmap`).
