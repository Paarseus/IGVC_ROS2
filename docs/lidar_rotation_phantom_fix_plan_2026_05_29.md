# LiDAR Rotation-Phantom — Fix Plan (2026-05-29)

Companion to [`lidar_rotation_phantom_diagnosis_2026_05_29.md`](lidar_rotation_phantom_diagnosis_2026_05_29.md).
Triple-confirmed root cause (bag forensics + methodology writeup + docs-grounded adversarial workflow):

- **TRIGGER = scan-skew (motion distortion).** `velodyne_transform_node` has **no `fixed_frame`**, so the
  whole ~50–100 ms revolution is stamped at one time and STVL does **one** TF lookup for it. During
  in-place rotation, late-sweep points are mis-rotated into an **arc** in the odom costmap. Fingerprint:
  the phantom ring **scales with range** (far ring grows ~2× during rotation in the bag).
- **AMPLIFIER = STVL persistence.** `voxel_decay`/`decay_acceleration` decide how long each skew-arc
  lingers. **`voxel_decay` is NOT the trigger** — lowering it alone just trades the rotation ring for the
  **stationary flicker** it was raised to fix.
- **GATES (why it's intermittent):** high ω (only spinning skews), CPU-starvation (RViz on Jetson →
  bursty STVL updates), and P1 map-drift (made it spin in the first place).
- **NEAR ring (0–2 m) is a SEPARATE effect** — likely **min-range self-detection** of the robot's own
  structure during rotation (NOT skew, which is anti-correlated with proximity). `velodyne.yaml:18`
  records prior self-detection (45 phantom cells at 0.4 m). **Unresolved — needs the open-area test (3.1).**

**Nothing here is applied yet.** Config-only (Tier 1 + 2) → symlink-installed → a relaunch picks it up,
no rebuild.

---

## Change set — tiered, ordered by value/risk

### Tier 0 — Operational (no file change)
- **Never run RViz on the Jetson during nav — use laptop Foxglove** (`ws://100.93.121.3:8765`). Removes
  the starvation amplifier. (Already established; restate as policy.)
- **P1 localization fix (Phase 1/2: `differential:false` → `map≡odom`)** is the upstream driver of the
  excessive spinning. Out of scope for *this* fix, but it's what stops the robot rotating into the
  phantom regime in the first place.

### Tier 1 — STVL config, low-risk, reversible (`nav2_params_humble.yaml`)
Apply to the **local** costmap (primary — this is what MPPI avoids on) and **mirror to global** for consistency.

| # | Param | Local line | Global line | Now | → New | Why | What it could break |
|---|---|---|---|---|---|---|---|
| 1.1 | `mark_threshold` | 289 | 535 | `0` | **`1`** | **Highest value.** Requires ≥2 returns in a voxel to mark → single-hit skew ghosts **and** flicker never mark — *without* the decay/flicker tradeoff. | A genuinely thin/far obstacle hit by only **one** beam per scan won't mark. **Validate the barrel still marks head-on** after the change. |
| 1.2 | `vertical_fov_padding` | 325 | 550 | `0.1` | **`0.35`** | Lets close/low voxels count as in-frustum → they get fast frustum decay (attacks the near-frustum amplifier). | Mild. Slightly more voxels subject to accelerated decay; could marginally speed clearing of low real obstacles. |
| 1.3 | `expected_update_rate` | 373 | 581 | `0.0` | **`0.2`** | STVL warns when its cloud buffer goes stale (≥0.2 s) → surfaces starvation instead of silently bursting. | Low — only enables a staleness warning; no behavior change to marking. |

**HOLD in Tier 1 (flicker tradeoff — do NOT change yet):**
- **`decay_acceleration` (local 335 / global 552, `2.0`):** the workflow suggests 2→6, but **this is the
  exact param the team lowered 10→5→2 to kill flicker.** Raising it risks re-introducing the ~40%
  stationary sawtooth. **Defer.** Only bump (and modestly, e.g. 2→3–4) *if the rotation ring persists
  after 1.1 + Tier 2*, and re-check flicker each step.
- **`voxel_decay` (local 281 `8.0` / global `5.0`):** **do NOT lower yet.** Lowering re-opens flicker;
  it's only safe to lower *after* deskew (Tier 2) makes sweeps agree spatially. Sequence matters.

### Tier 2 — Structural cure (`velodyne.yaml`) — the real fix, needs on-device verification
| # | Change | File / line | Why | What it could break |
|---|---|---|---|---|
| 2.1 | Add **`fixed_frame: "odom"`** to `velodyne_transform_node` | `velodyne.yaml` (~line 16, in the `velodyne_transform_node` params) | Turns on **per-packet ego-motion compensation** (`computeTransformToFixed`) → removes skew **at the source** → both costmaps deskewed → ring gone even at high ω, *and* persistence can stay high (no flicker tradeoff). | **Changes the output cloud's `frame_id`** (velodyne→odom per upstream behavior). MUST verify STVL still marks (it transforms cloud→global_frame; should be fine since local global_frame is `odom`) and that **no consumer assumes the `velodyne` frame**. Requires `odom→velodyne` TF at packet time (we have it via robot_state_publisher + odom EKF). |

**After Tier 2 is validated working:** *then* safely **lower `voxel_decay` (local 8→~3)** and keep
`decay_acceleration` modest — deskewed sweeps leave no arc to accumulate, so you get ring-free **and**
flicker-free.

### Tier 3 — Investigations (no change; resolve open questions)
- **3.1 Open-area rotation test** — rotate in place at ~0.5 rad/s with **nothing around**. If a **0.7–1.0 m
  ring still appears** → it's **self-detection**, not skew/proximity → raise `velodyne.yaml` `min_range`
  `0.7 → 0.9`. (Strongly suspected given the documented 0.4 m self-detection.)
- **3.2 Resolve rpm vs rate** — `rpm: 600` implies 10 Hz scans, but measured `/velodyne_points` ≈ 19.7 Hz.
  Measure the true per-cloud timespan from the bag; it sets the actual skew magnitude (50 ms vs 100 ms sweep).
- **3.3 Verify on Jetson Humble** — both the writeup and the workflow read the **Jazzy** `datacontainerbase`
  source, asserting identical Humble behavior. Confirm `fixed_frame` deskew + output-frame behavior on the
  Jetson's installed `ros-humble-velodyne` before trusting Tier 2.

---

## Validation sequence (Jetson + **laptop Foxglove, NO RViz on Jetson**)
Run with the perception/REDUCED config from the Phase 0 work; record a light bag each step.

0. **Open-area rotate** 0.5 rad/s → near ring with nothing around? (resolves 3.1 self-detection).
1. **Baseline:** barrel ~1 m ahead, one in-place revolution. Log local-costmap **lethal-cell count +
   radial spread @2 Hz** and `ros2 topic hz /cmd_vel /local_costmap/costmap_raw` → record the trailing arc.
2. **Apply Tier 1**, repeat 1 → **pass = ring collapses <1 s after yawing past the object, and the barrel
   still marks head-on.**
3. **Park 30 s next to the barrel** → **pass = stationary sawtooth stays well under the historic ~40%**
   (proves Tier 1 didn't regress flicker).
4. **Apply Tier 2 (`fixed_frame: "odom"`)** → **pass = ring gone even at 1.0 rad/s AND STVL still marks**
   (explicitly verify the frame_id contract didn't break marking). *Then* lower `voxel_decay` and re-check.
- **Always check CPU load first** if the ring/flicker returns — it signals a starvation regression
  (RViz/heavy bag), not a decay-tuning need.

---

## What we are deliberately NOT changing (refuted by all three sources)
- **`transform_tolerance`** — not the cause (0 TF exceptions in the Phase 0 run).
- **`ekf.yaml` / EKF yaw** — odom EKF is healthy (gyro bias ~0.008°/s, 105–106% ω delivery per
  `yaw_diag_session_2026_05_27/VERDICT.md`); it fuses no GPS, so it's GPS-immune.
- **STVL → VoxelLayer/ObstacleLayer** — STVL's time-decay is the right tool for sparse VLP-16; the issue
  is skew + persistence tuning, not the layer.
- **`obstacle_range` 15→8** — optional/low value (bounds far smear, not the near trapping ring). Don't lead with it.

## Recommended execution order
1. **Tier 0** (RViz off) — always.
2. **Tier 1.1 `mark_threshold: 0→1`** + **1.2/1.3** — highest value, lowest risk, no flicker tradeoff. Validate (steps 1–3).
3. **Tier 2.1 `fixed_frame: "odom"`** — the cure; validate frame contract (step 4) + on-device (3.3).
4. **Only then:** lower `voxel_decay`; bump `decay_acceleration` only if still needed, with flicker re-check.
5. **Tier 3.1** open-area test in parallel to settle the near-ring (self-detection → `min_range` bump).

---

## UPDATE 2026-05-29 PM — field findings + what was applied

### NEW root-cause finding: the footprint was undersized (this was the clipping cause)
While testing barrel avoidance, the chassis kept **clipping the barrel**. Root cause turned out to be
**not inflation** but the **costmap footprint**:
- The configured footprint was a rectangle `[[±0.5, ±0.37]]` → circumscribed only **0.622 m** from
  `base_link` (inscribed 0.38 m).
- Measured **IMU(=base_link)-to-furthest-vehicle-point = 0.914 m**. (The IMU sits at `base_link`
  x=0/y=0; the Velodyne is 0.089 m forward, so the earlier 0.81 m LiDAR figure was off by the mount
  offset.)
- So the real robot extended **~0.3 m beyond** the modeled footprint → MPPI's `consider_footprint`
  check thought the corners were clear when they weren't → **clipping**, which no inflation value fully
  fixes.

**Applied (committed):** local + global costmap → **`robot_radius: 0.914`, `footprint: "[]"`** (circular;
Nav2 auto-generates a 16-pt circle that `consider_footprint` uses). Circular is correct for an
in-place-rotating tracked robot (circumscribed circle = true swept shape). *Verify on launch that
`published_footprint` is a circle (not center-only).* A tighter rectangle is a later efficiency option
but needs front/rear/half-width measured separately (robot may be asymmetric about base_link).

### `mark_threshold: 0 → 1` — validated by on-robot A/B, applied (committed)
Clean A/B on the Velodyne bag (in-place spin, barrel at 4 m): the **far-ring rotation excess dropped
from +49% to +11% (HI/LO 1.49 → 1.11)**, barrel still marked (~130 cells), **no flicker tradeoff**.
This is the Tier-1.1 lever; it attacks single-hit scan-skew ghosts at all ranges.

### Self-detection hypothesis — REFUTED by the open-area test
Open-area in-place spin (nothing within 3 m): **zero near ring (0–1 m = 0.0 lethal)** → `min_range: 0.7`
is adequate, the robot is **not** seeing itself. The far ring (9–15 m, distant structures) still grows
~1.7× while spinning, confirming the **scan-skew** mechanism. The near ring seen in *barrel* runs was
the **real barrel at close range (proximity)**, not self-detection.

### Still open / not yet applied
- **Inflation re-tune with the correct footprint.** Committed value is still `0.3` (corridor-tuned for
  the old undersized footprint). With the true 0.914 m circular footprint the clip is fixed by the
  footprint itself; inflation is now a separate corridor-vs-margin knob to re-tune (the
  "inflation < inscribed" warning will be loud with a circular footprint — advisory, since
  `consider_footprint` is the hard constraint).
- **Tier 2 deskew (`fixed_frame: "odom"`)** — still the structural cure for the residual ~11% skew ring;
  not applied (needs the frame-id verification on the Jetson Humble binary).
- **Validation pending:** re-run the 7 m barrel-avoidance goal on the **full stack** (perception ON, so
  the camera also marks the barrel — LiDAR-only under-marks it) with the corrected footprint; confirm
  no clip and `published_footprint` is a circle.
- **0.089 m LiDAR mount offset** is a URDF estimate — confirm on the real vehicle.
</content>
