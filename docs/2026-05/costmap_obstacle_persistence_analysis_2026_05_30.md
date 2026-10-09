# Costmap Obstacle Persistence Analysis — Keeping LiDAR/Camera Points After RTK GPS

**Date:** 2026-05-30
**Author:** Analysis subagent (for IGVC team lead)
**Scope:** AutoNav Challenge only (no Self-Drive)
**Question asked:** "Deep analysis of the costmap — can we keep LiDAR/camera points so they persist instead of disappearing — now that we added RTK GPS, and what do the IGVC 2026 rules allow."

---

## TL;DR

Yes, we *can* make obstacle points persist instead of decaying, and RTK GPS is the change that makes it *possible* in principle — but **persistence is conditionally safe, not unconditionally safe, and the most aggressive version of the plan is currently blocked by unresolved RTK and de-skew prerequisites.**

**Bottom line recommendation:** Adopt a **hybrid** — keep the **LOCAL** costmap in `odom` (drift-free, jump-immune, the reactive layer MPPI samples) and make the **GLOBAL** (`map`-frame) costmap the persistence layer, with a **finite-but-long** STVL decay (not infinite), re-enabled camera layer, and **RTK-fix-gated auto-clear** so an RTK dropout wipes the persisted map before it can smear. **Do NOT use `decay_model:-1` (PERSISTENT) or `mapping_mode:true`** — those remove the only escape valve, grow unbounded, serialize a course map to disk (an IGVC §I.2 violation), and do nothing for the documented footprint-freeze stall.

**Does RTK actually make persistence safe?** It makes the *position* half safe: RTK FIXED collapses `map→odom` position error from the old SBAS 6–12 m drift to ~1–3 cm, and the 2026-05-30 field run confirmed `map→odom` was stable enough to navigate. **But three honest caveats stand, and two are blockers:**

1. **The field proof is thin.** The validated run was a single ~26.7 m straight-in drive with no obstacles and no persistence enabled. It does NOT prove `map→odom` stays sub-cm-stable (especially rotationally — single-antenna MTi-680G, no mag) across a full 500 ft / 6 min switchback course. **A long-run TF log is a hard gate before raising decay.**
2. **RTK dropout (FIX→FLOAT/SBAS) snaps `map→odom` on the very next EKF cycle**, before any software dwell-and-clear can fire — and the rejection gate is currently effectively OFF (`odom0_pose_rejection_threshold: 13.8`). The covariance floor + real Mahalanobis gate (rtk_integration_change_plan ranks 4 and 6) **must ship and be bench-validated before any global decay increase.**
3. **Persistence does not fix the documented barrel-freeze stall** (footprint/inflation geometry) and several proposed bumps (longer decay, wider inflation, re-enabled semantic layer) risk *worsening* it, plus re-admit the rotation scan-skew ghost because LiDAR de-skew (Issue #21) is still OFF.

**The IGVC rules are not the blocker.** Within-run, live-sensor obstacle persistence is **explicitly allowed** (and the perception it requires is mandated). Only *cross-run* course memorization is forbidden (§I.2), which we satisfy by never serializing a map to disk and by full kill+relaunch between runs.

So: the *direction* is right and RTK is the right enabler, but **gate the rollout** — frame-stability over distance, covariance/gate prerequisites, and de-skew first; LiDAR persistence before camera persistence; finite decay derived from pass-by time, not a guessed 45 s.

---

## The Question

The current costmap **forgets** obstacles:

- LiDAR-marked voxels (Velodyne → STVL) **time-decay to zero in ~3 s local / ~5 s global**.
- Camera-detected lanes/barrels/potholes (ZED → semantic layer) **decay in ~0.3 s** or are instantly raytrace-cleared when the camera sees through them, and exist **only in the local costmap** — they never enter the global/map-frame plan at all.

The team lead wants obstacle points to **persist** ("remember the barrel you just drove past so the planner doesn't cut a chord back into it") instead of disappearing, and is asking whether the newly-added **RTK GPS** (absolute fusion, commit `82f5fd3`) makes that safe, and whether the **IGVC 2026 rules** permit it.

This document answers all three: *what clears points today*, *what RTK changes*, *what the rules allow*, and *the concrete, gated config plan* — with the adversarial review folded in honestly so the blockers are not hidden.

---

## How the Costmap Currently Handles Obstacle Persistence

There are two costmaps with deliberately different forgetting behavior, plus two distinct sensor→costmap paths (LiDAR and camera).

### Local costmap (odom frame) — fast, reactive, short memory

- `global_frame: odom`, `rolling_window: true`, 50×50 cells @ 0.2 m = **10×10 m**, 10 Hz (`nav2_params_humble.yaml:240-257`). Layer order `[stvl_layer, semantic_layer, inflation_layer]` (`:281`); STVL first so the semantic layer's LETHAL writes win via `updateWithMax` (`:267`, `combination_method:1`).
- This is the layer MPPI actually samples. Because it is odom-frame there is no GPS smear to accumulate, so persistence here is geometrically safe — but it is intentionally short so it stays *reactive*.

### Global costmap (map frame) — was the "forget fast" layer, built on the no-RTK premise

- **RESOLVED (verified against live YAML 2026-05-30):** the global costmap is **already in `map` frame** — `global_frame: map` (`nav2_params_humble.yaml:502`), as are `bt_navigator` (`:21`) and `route_server` (`:644`). The `local_costmap` stays `odom` (`:243`). This `odom→map` restore was committed 2026-05-30 the moment RTK FIXED landed. So the REP-105 frame split the persistence plan wants is **already in place** — the frame is not the blocker. (An earlier analysis pass cited a stale `:502` comment as still `odom`; that is wrong — the live value is `map`.)
- Historically this layer was the "forget fast" costmap, engineered around the assumption that the GPS `map` frame was unusable (the no-RTK premise below). RTK is what justified flipping it back to `map`.
- Only obstacle source is STVL on the Velodyne. The semantic (camera) layer was **deliberately removed** from the global plugins list — `plugins: ["stvl_layer", "inflation_layer"]` (`:528`) — with the block retained but `enabled: false` (`:569-571`). Removal rationale (`:518-527`): painted-lane perception is short-range, has no out-of-FOV clearing, and **"smears in the global costmap under unaided GPS."**

### What clears LiDAR points, and when (STVL)

STVL does **not** use classic raytrace-through-cell clearing — that is precisely why it replaced `VoxelLayer` on 2026-05-13 (`:268-280`): the VLP-16's 16-beam 2° ring spacing leaves angular gaps that a stationary robot freezes, and raytrace can never clear a cell with no return beyond it. Instead STVL clears two ways:

| Clearing path | Param | Local value | Global value | Effect |
|---|---|---|---|---|
| **Time-decay** | `voxel_decay` + `decay_model` | 3.0 s, model 0 (linear) (`:286,291`) | 5.0 s, model 0 (linear) (`:538,542`) | An un-reconfirmed voxel fades to zero over this many seconds. `decay_model: -1` = PERSISTENT (never decays). |
| **Frustum-accelerated** | `decay_acceleration` | 2.0 (`:340`) | 2.0 (`:562`) | A voxel **inside** the sensor FOV that gets no confirming return decays *faster* — this is how a removed barrel (laser now sees sky) clears on a stationary robot. The only "the obstacle is actually gone" path for sparse VLP-16. |

Marking gate: `mark_threshold: 1` means a voxel needs **≥2 returns** to be marked (kills single-hit ghosts) (`:294/:545`); height band `[0.4, 0.8]` m in odom frame (`:321-322`); `obstacle_range` 8.0 m local / 10.0 m global (`:323/:557`).

**STVL internals confirm:** `grid.cpp:320-331` — `decay_model 0` returns `voxel_decay - time_delta`; `decay_model -1/2` is PERSISTENT and never clears. The frustum path (`grid.cpp:176-198`) **still clears in-frustum voxels if `decay_acceleration > 0` even in PERSISTENT mode** — so "persist forever" requires *both* `decay_model:-1` *and* `decay_acceleration:0.0` (a documented trap). `mapping_mode:true` (`layer.cpp:770-803`) never clears AND **serializes the OpenVDB grid to disk every 60 s** — a literal saved course map.

### What clears camera points, and when (semantic layer)

Camera flow: ZED X → `perception_node` (sooner25 pipeline, **stateless single-frame** V>215 threshold, emits only `class_id_lane`) → 4-topic contract → kiwicampus `SemanticSegmentationLayer` in-process in `controller_server`. Each in-range mask pixel's paired 3D cloud point is TF'd to odom, binned to a 0.2 m tile, pushed onto a per-tile time-queue. On every `updateBounds`:

1. **Raytrace FREE_SPACE** along sensor→point rays for the current frame (`clearing: true`, `raytrace_max_range: 8.0`, `semantic_segmentation_layer.cpp:701-779`) — actively erases any cell the camera now sees through, even one marked LETHAL a moment ago.
2. **Purge** queue entries older than `tile_map_decay_time: 0.3 s` (`:370`; `segmentation_buffer.cpp:241`).
3. **Re-mark**: live tiles get **254 (LETHAL)** for danger classes (lane_white/barrel_orange/pothole, `samples_to_max_cost:1` — single sample marks). Decayed-empty tiles are **skipped** (`if (tile.second.empty()) continue;`, `:372-375`) — they are NOT reset to free.

**Net current behavior:**
- A LiDAR obstacle that leaves view is forgotten in ~3 s local / ~5 s global (faster if still in-frustum).
- A camera lane/barrel **in-FOV** clears in 0.3 s or instantly when raytraced through — the **least persistent** source.
- A subtle gap: a camera cell that goes **out-of-FOV** (e.g. a lane directly ahead after a 90° turn) is neither re-observed nor raytrace-cleared, so it **persists by accident** until the rolling window drops it (~25 m back). This is documented (`cv_costmap_deep_analysis_2026_05_29.md:194-197`).

---

## What RTK Changes

### The old premise: the GPS `map` frame was unusable

Before RTK, the global costmap and BT navigator were engineered to **forget fast** because the `map` frame was structurally hostile to persistence:

- Unaided SBAS GPS drifted `map→odom` **~6–12 m** AND rotated heading **~65–96°** (commit `5e6f279`; `project_nav_must_run_in_odom_not_gps_map`).
- Any world-anchored obstacle representation accumulated into phantom blobs: **5878 → 24567 lethal cells in a 40 s drive** that boxed the robot in (`nav2_params_humble.yaml:493-499`).
- So obstacles were anchored to `odom` (drift-free, built from IMU + wheel_odom + ZED-yaw, no GPS), decay was kept short, and the camera layer was excluded from the global costmap.

### What RTK FIXED changes

- RTK FIXED via MDOT CORS (`NS-IMAX-MSM4`) collapses the **position** half of `map→odom` to **~1–3 cm** (rtk_integration_change_plan §1).
- The map EKF now fuses GPS in **ABSOLUTE mode** (`ekf.yaml:186` `odom0_differential: false`, commit `82f5fd3`), pinning `map→odom` continuously instead of letting it walk off.
- A world-anchored obstacle now stays at its true (x,y) across a run instead of smearing — **which is exactly what is needed to "remember the points."**
- AutoNav obstacles are **100% static** (see rules section), so a persisted point is always a real, still-present obstacle. There is no "phantom where a moving object used to be" failure mode.

### The critical caveats (this is where it gets honest)

1. **The frame flip is already DONE — global obstacles live in `map` today.** Verified against the live YAML: `global_costmap`, `bt_navigator`, and `route_server` are all `global_frame: map` (`nav2_params_humble.yaml:502,21,644`); only `local_costmap` is `odom` (`:243`). So the persistence layer (global costmap) already lives in the RTK-pinned `map` frame — the architecture is correct. The remaining question is therefore **not** "should we promote to map" but **"is `map` stable *enough over a full run* to host long-lived marks"** — a map-stability question, not a frame-choice question. RTK pins `map→odom` to ~1–3 cm while FIXED, but that has only been proven over a 26.7 m straight drive (see caveat 1 in the TL;DR), and a FIX→FLOAT dropout still snaps the transform (caveat 3 below).

2. **Heading rotation persists even at cm-FIX.** The single-antenna MTi-680G on the no-mag General_RTK profile derives yaw from GNSS course-over-ground; RTK FIX fixes *position* but **not** the 65–96° heading-observability problem at low speed (rtk_integration_change_plan §2). If `map` rotates, every persisted cell rotates *about the robot* — the worst failure mode for long-lived marks. The 2026-05-30 field run did not exhibit this, but it was a 26.7 m straight drive at ~0.6 m/s — **not a long, slow, switchback course.**

3. **RTK dropout is instantaneous and currently un-gated.** FIX→FLOAT→standalone steps `map→odom` discretely (REP-105) by cm-to-meters. With absolute fusion ON and the rejection gate effectively OFF (`ekf.yaml:194` `odom0_pose_rejection_threshold: 13.8` — value is σ, squared in `filter_base.cpp` → ~190 → P(reject) ≈ 4e-42), a single degraded fix **snaps the transform on the very next EKF cycle** and propagates straight into `map→odom`. The real fix-status signal exists on `/gnss` (`gnsspublisher.h:87-95` decodes carrSoln → NavSatStatus) but **nothing consumes it** — robot_localization ignores NavSatStatus, and no `rtk_health_node` exists yet. MDOT CORS iMAX/VRS drops FIX whenever the GGA uplink lapses > 30 s, so FLOAT excursions are routine, not exceptional.

**Summary:** RTK makes the position anchor good enough that persistence is *worth attempting in the global map frame* — but only behind a fix-status gate, a real rejection gate, and a covariance floor that **do not yet exist**.

---

## What the IGVC 2026 Rules Allow

### The mapping/memorization prohibition (§I.2, verbatim)

> "Vehicles must be unmanned and autonomous. They must compete based on their ability to perceive the course environment and avoid obstacles. ... All computational power, sensing and control equipment must be carried on board the vehicle. **No base stations allowed for positioning accuracy Mapping or course position memorization is not allowed. Judges will adjust course between runs to nullify any mapping/memorization**" — §I.2 VEHICLE CONFIGURATION (lines 139-143)

Duplicate cross-run framing in the Self-Drive appendix (which this team does NOT enter):

> "No course mapping is allowed. The obstacles positions may vary each run." — III.7 Appendix C (line 1418)

### Within-run vs between-run — the decisive distinction

The enforcement clause is the interpretive key: **"Judges will adjust course between runs to nullify any mapping/memorization."** Re-arranging obstacles between runs only defeats a map carried *over* from a previous run. It has no effect on a costmap rebuilt from live sensors *this* run. Therefore:

- **ALLOWED:** Within-run obstacle persistence in a live, sensor-built costmap. This is the required "perceive the course environment and avoid obstacles" capability, not prohibited mapping. Standard Nav2 rolling-costmap accumulation of *this run's* observations is compliant.
- **PROHIBITED:** Course-position memorization carried *between* runs — saving/reloading a map of the specific course layout to pre-plan or shortcut.

Positioning is not merely allowed but **required**: §I.4 (lines 230-231) requires the vehicle to "find a path to a single two meter navigation waypoint," and the design report (lines 1620-1624) requires teams to "Describe how the system uses GPS for waypoint navigation and localization."

### AutoNav obstacles are all static (verbatim)

> "Obstacles on the course will consist of various colors ... of construction barrels/drums ... Natural obstacles such as trees or shrubs and manmade obstacles such as light posts or street signs could also appear ... Simulated potholes of 2 foot diameter, solid white circles may be inserted." — II.2 (lines 288-294)

> "The Course will be primarily sinusoidal curves with series of repetitive barrel obstacles." — II.2 (lines 299-302)

> "There will be a minimum of five feet clearance, minimum passage width, between the line and the obstacles." — II.2 (lines 295-298)

**There are zero moving obstacles and zero pedestrians in AutoNav** (mannequins/dynamic-pedestrian tests are Self-Drive-only). So a persisted point is always a real, still-present obstacle — and on narrow ~10 ft sinusoidal lanes with 5 ft turning radius, persistence is *beneficial*: it stops Navfn (holonomic) from cutting a chord back into a barrel the robot just steered around once it leaves the forward FOV.

### Scoring asymmetry favors retaining obstacles

> Penalty table (II.4): Crash/Obstacle Displacement = **−10 ft + E-Stop end-of-run**; Leave the Course = **−10 ft + E-Stop end-of-run**; Crossing internal lines = E-Stop end-of-run; Sideswipe/Obstacle Touch = −5 ft (NO E-Stop); Blocking Traffic (>1 min stationary) = −5 ft + E-Stop.

Run-ending events freeze your adjusted distance at the violation point. Keeping a barrel LETHAL after it exits FOV prevents the catastrophic run-ending crash; the worst it risks is a survivable brief slow-down (or, if a *false* persisted cell boxes the robot >1 min, a −5 ft Blocking-Traffic E-Stop). **Retaining barrels is the correct bias** — but stale/false cells must clear before the 1-minute Blocking threshold and before they force a boundary excursion.

### Compliance bottom line

| Requirement | Mechanism |
|---|---|
| No map files on disk | `mapping_mode: false` on BOTH costmaps (`:297/:548`) — OpenVDB grid never serialized. |
| Reset every run | Full kill + fresh `ros2 launch` per run zeroes both in-memory costmaps + EKF/odom (lifecycle configure clears the grid). **This is the mandatory §I.2 mechanism — not a node's startup clear.** |
| No cross-run obstacle memory | No SLAM map reload, no hand-seeded obstacles, no recorded prior path. |
| Single qualified stack | Persistence must live in the *same* autonomous stack used at qualification (§I.4) — an RTK-gated "persist under FIX / reactive under !FIX" data-driven behavior is acceptable, but must be present and identical during qualification, not a competition-only toggle. |
| Document it | State in the design report that the costmap is per-run, in-memory, sensor-built, and reset each run. |

One caveat: judge approval of **public CORS/VRS NTRIP** under §I.2 ("No base stations") is still only verbal/pending in writing. If denied at competition → `enable_ntrip:=false` → SBAS → map-frame persistence is the documented runaway regime, so the plan must fall back to odom reactive-only.

---

## Recommended Approach

**Name:** RTK-gated hybrid — reactive `odom` LOCAL + persistent `map` GLOBAL with finite long decay, auto-cleared on RTK degradation.

**Method:**
- **LOCAL costmap** stays `global_frame: odom`, rolling — the fast reactive layer MPPI samples. It must never smear on a map jump, so it keeps short decay (modest bump only). Never cleared on RTK transitions.
- **GLOBAL costmap** (`map` frame, *contingent on confirming the frame state*) becomes the persistence layer: lengthen STVL decay (finite, **not** `-1`), re-enable the semantic layer so camera lanes/potholes persist as world marks. `mark_threshold:1` (≥2 returns) stays as the safety companion so only confirmed obstacles get the long lifetime. `decay_acceleration:2.0` stays so an in-frustum removed obstacle still clears.
- **NEW `rtk_health_node`** parses NMEA GGA field-6 → `/rtk/fix_state` (latched); a supervisor calls `global_costmap/clear_entirely_global_costmap` on FIX→!FIX so a map jump cannot smear persisted points. Persistence is RTK-conditional, off-by-default under degradation, and degrades to today's validated reactive-only behavior.

### Config diff

> **GATED.** Per the adversarial review, several of these are **blocked behind prerequisites** (frame-stability long-run log, covariance floor + real rejection gate, LiDAR de-skew). The "Gate" column states what must be true *first*. Do not apply gated rows until their gate is green.

| File | Param | From | To | Why | Gate |
|---|---|---|---|---|---|
| `nav2_params_humble.yaml` | `global_costmap.stvl_layer.voxel_decay` | `5.0` | `~10–15` (NOT 45 until field-tuned) | Primary LiDAR persistence lever. SBAS-era smear objection (`:538-541`) is dissolved by RTK FIX. Keep `decay_model:0` + `decay_acceleration:2.0`. **45 s is unjustified** — at 0.6 m/s a false cell blocks a 5 ft corridor for ~27 m; derive from measured pass-by time (~10–15 s suffices to prevent chord cut-back, well under the 60 s Blocking threshold). | Long-run TF stability log **+** de-skew (Issue #21) **+** covariance floor + real gate shipped |
| `nav2_params_humble.yaml` | `global_costmap.plugins` | `["stvl_layer","inflation_layer"]` | `["stvl_layer","semantic_layer","inflation_layer"]` | Persist camera lanes + white-circle potholes (camera-only) as world marks. 2026-05-12 removal rationale ("smears under unaided GPS") is now false under RTK FIX. Order: semantic between stvl and inflation so LETHAL wins via `updateWithMax`. | **After** LiDAR persistence validated; cmd_vel ≥18 Hz held as a *gating* metric, not post-hoc |
| `nav2_params_humble.yaml` | `global_costmap.semantic_layer.enabled` | `false` | `true` | Activate retained block (`:571`). **Keep its existing 0.3 s `tile_map_decay_time` + `clearing:true` for the first test** — raise decay only AFTER confirming no lane smear (lanes are LETHAL by design; a persisted false lane walls a narrow corridor). | Same as above |
| `nav2_params_humble.yaml` | `local_costmap.stvl_layer.voxel_decay` | `3.0` | `6.0` | Modest local persistence so a barrel turned out of the narrow FOV clears slightly slower. Safe in odom (no smear). **Do NOT return to 8.0** (re-admitted flicker). | De-skew confirmed + Velodyne unicast "Save Configuration" persisted |
| `nav2_params_humble.yaml` | `global_costmap.stvl_layer.obstacle_range` | `10.0` | `8.0` | Match local; per obstacle_stall RCA ("lower global obstacle_range 10→8 so the global ring stops forming"). Avoids far-ring scan-skew ghosts living the full decay window; reduces CostCritic load. | None (independent improvement) |
| `nav2_params_humble.yaml` | `global_costmap.inflation_layer.inflation_radius` | `0.3` | **(DROP — do not change)** | The adversarial review is correct: raising to 0.45 with `xy_goal_tolerance:0.2` makes any waypoint within ~0.73 m of a barrel unreachable → reproduces the documented **North/5002 failure** (`:70` CAUTION). Lanes are LETHAL + 5 ft corridors → a wider halo walls sub-spec passages. **Rejected.** | — |
| new `rtk_health_node.py` + `navigation.launch.py` | RTK fix-state + clear supervisor | absent | `rtk_health_node` parses GGA field-6 → `/rtk/fix_state`; supervisor clears global costmap on FIX→!FIX dwell, and once on FIX-acquire | REQUIRED enabler. NavSatStatus has no RTK enum, so GGA parsing is the only honest fix-state. Converts "persist forever" into "persist while FIX, auto-wipe on drop." Build per change-plan rank-1 (low risk, additive). | Prerequisite for ALL global persistence rows |

### Run-reset mechanism (IGVC §I.2 compliance)

Three layers, none persisting to disk:

1. **No map files** — `mapping_mode: false` on both costmaps (`:297,:548`). The OpenVDB grid is never serialized; the costmap is purely in-memory and rebuilt from this run's sensors.
2. **Lifecycle reset on every (re)launch** — the documented "kill the stack, fresh `ros2 launch navigation.launch.py` per run" recipe zeroes both costmaps AND the EKF/odom. **This is the operational, mandatory §I.2 mechanism.**
3. **`rtk_health_node` clears the global costmap on FIX-acquisition at startup** — defense-in-depth for a warm restart that skipped a full relaunch. **Do not rely on this as the primary guarantee** (it depends on an as-yet-unbuilt node firing correctly).

Explicitly do **NOT** add an unconditional `ClearEntireCostmap` to the per-cycle BT loop — that defeats persistence. The recovery-subtree `ClearEntireCostmap` (`navigate_igvc_autonav_humble.xml:100`) is acceptable because recovery is rare; verify it is not firing often enough to silently wipe the persisted map (Open Question), and consider switching it to `ClearCostmapAroundRobot`.

### RTK-degradation gating (the highest-risk surface)

| Phase | Behavior |
|---|---|
| **DETECT** | `rtk_health_node` parses NMEA GGA field-6 + staleness watchdog (GGA uplink ≥ every 30 s or iMAX VRS drops FIX) → `/rtk/fix_state` latched. NavSatStatus is useless (no RTK enum). |
| **WIPE** | On FIX→!FIX with a short dwell, supervisor calls `clear_entirely_global_costmap` ONCE — persisted points are deleted, not smeared. |
| **DEMOTE** | While !FIX, global STVL re-accumulates only fresh in-frustum marks; the LOCAL odom costmap (untouched, jump-immune) carries the actual reactive avoidance MPPI samples. |
| **RE-ARM** | On return to FIX for a dwell, clear once more and resume persistence from empty. |

**Honest blocker (see Risks):** the WIPE path is **2 s dwell + NMEA + service latency**, but the `map→odom` step is **instantaneous at the next EKF GPS-fusion cycle**. With absolute fusion + a dead rejection gate, a single FLOAT outlier smears every persisted cell *before* the dwell elapses. This is why the covariance floor + real Mahalanobis gate are **hard prerequisites**, and why — until they are proven — persistence should stay in the LOCAL odom costmap where there is no jump source at all.

---

## Risks & Regression Guards

The adversarial review rated the original aggressive plan **flawed**, with two **blockers**. Folding it in honestly:

| # | Risk / challenge | Severity | Status & mitigation |
|---|---|---|---|
| R1 | **Field proof overstated.** The 26.7 m straight drive does NOT prove `map→odom` is sub-cm-stable over a full switchback course (open question #1 admits this). Single-antenna 65–96° heading rotation can rotate persisted cells about the robot. | **Blocker** | **Accepted as gating.** Hold global `voxel_decay` at 5.0 until a long-run (>150 m, with in-place rotations) TF log shows sub-cm position **and** ~0° rotation drift end-to-end. This is the decisive gate (verification step 1). |
| R2 | **RTK-dropout race the gate cannot win.** `map→odom` snaps on the next EKF cycle; the 2 s detect+dwell+clear loses the race. Gate is effectively OFF (`ekf.yaml:194` ~13.8 σ). Covariance floor (rank 4) + gate fix (rank 6) are unbuilt prerequisites the plan did not require. MDOT CORS FLOAT excursions are routine. | **Blocker** | **Accepted as gating.** Make the covariance floor AND a real (non-disabled) Mahalanobis gate **hard prerequisites that ship + bench-validate before any global decay increase.** Bench-measure the actual `map→odom` step magnitude/timing on a forced FIX→FLOAT→standalone. If the wipe cannot beat the EKF (it likely cannot), disable global persistence on the *first hint* of !FIX, not after a dwell — or keep persistence LOCAL-only until the covariance contract is proven honest. |
| R3 | **Rotation scan-skew ghost re-admitted.** Raising `voxel_decay` re-creates the in-place-rotation far-ring because de-skew (`velodyne.yaml:28` empty, Issue #21) is still OFF. RTK FIX does nothing for this scan-skew artifact. `decay_acceleration:2.0` won't catch a far-ring ghost the 16-beam pattern never revisits. | **Major** | **Accepted as gating.** Correct de-skew (Issue #21) is a **hard prerequisite** before ANY decay increase on either costmap. Re-confirm the Velodyne unicast "Save Configuration" was persisted (else flicker returns on power-cycle and long decay makes it sticky). Hold decay until a rotation-in-place A/B on a fresh bag shows low far-ring excess. |
| R4 | **Density vs MPPI starvation + footprint freeze.** The barrel STALL is footprint/inflation geometry, NOT insufficient persistence — marks were rock-stable at freeze. Persistence enlarges the no-go region against a freeze-prone footprint. Re-enabling the global semantic layer adds per-cell bookkeeping on the core that previously starved the 20 Hz loop. | **Major** | **Partially mitigated.** Sequence the footprint right-size + `time_steps` verification BEFORE any persistence change. Treat cmd_vel Hz + per-cycle costmap update time as a **gating** metric (must hold ≥18 Hz with margin under full ZED+perception+bag load), not a post-hoc check. Note the current footprint is already the rectangle (inscribed 0.2794), so it does not collapse the gradient the way the old 0.914 circle did. |
| R5 | **Global inflation 0.3→0.45 reproduces North/5002.** With `xy_goal_tolerance:0.2`, a 0.45 halo makes any waypoint within ~0.73 m of a barrel unreachable → "fail to compute path" → Goal failed. Contradicts the pinned "lanes stay LETHAL, inflation stays 0.3" decision. | **Major** | **Accepted — change dropped.** Do not raise global inflation while lanes are LETHAL and corridors are 5 ft. If a gradient is wanted it belongs on the LOCAL odom costmap. Couple any future inflation increase to a per-waypoint goal-tolerance widening in mission_manager (which does not yet exist). |
| R6 | **Run-reset warm-restart gap.** The plan relies on `rtk_health`'s clear-on-FIX as the guard for warm restarts; if that node is absent/crashed, a prior-run global costmap could survive into the next run → §I.2 violation. | **Minor** | **Mitigated by protocol.** Make full kill+relaunch between every run the MANDATORY §I.2 mechanism (zeroes in-memory costmaps via lifecycle configure). Treat `rtk_health`'s startup clear as defense-in-depth only. Verify in the run-reset test that BOTH costmaps come up empty even if `rtk_health` is absent. |
| R7 | **45 s decay not derived.** The same long window applies to FALSE positives (scan-skew, multipath ~31% below-ground, glare-frame lanes with single-sample 254). One glare frame = a 45 s wall on a 5 ft corridor. | **Minor** | **Accepted — use shorter, derived value.** Derive decay from measured pass-by duration (~10–15 s for a 0.6 m/s pass), not 45 s. Sequence LiDAR persistence (color-agnostic, `mark_threshold:1` gated) ahead of camera persistence (single-sample 254, high false-positive). |

### Regressions explicitly risked (do not re-trigger)

- **GPS/map-jump lethal-cell smear** (5878→24567, `:493-499`) — re-admitted on any RTK FIX→FLOAT unless covariance floor + gate ship first.
- **Rotation scan-skew phantom ring** (`lidar_rotation_phantom_diagnosis_2026_05_29.md`) — gated on de-skew (Issue #21).
- **Stationary costmap flicker** — if Velodyne unicast "Save Configuration" wasn't persisted, long decay makes flicker sticky.
- **Barrel-avoidance footprint freeze** (`obstacle_stall_rca_2026_05_30.md`) — persistence is on the Do-NOT list for this; right-size the footprint instead.
- **MPPI 20 Hz starvation** (`project_semantic_layer_mppi_starvation`) — re-enabling the global semantic layer adds load on the core that previously dropped cmd_vel below 18 Hz.
- **Near-obstacle waypoint unreachability / North-5002** (`:70` CAUTION) — do not raise global inflation.
- **Lethal-lane corridor walling** (`feedback_lanes_stay_lethal`) — persisting LETHAL lanes + wider inflation walls 5 ft passages.

---

## Rejected Alternatives

| Approach | Why rejected |
|---|---|
| **(a) `decay_model:-1` (PERSISTENT) or `mapping_mode:true`** | Removes the only time-based escape valve (`grid.cpp:320-331`). PERSISTENT alone is a trap — must ALSO set `decay_acceleration:0.0`, after which a removed obstacle is permanent. A single false return (~31% of VLP-16 returns measured below ground) becomes a forever lethal cell; OpenVDB memory is unbounded; per-voxel bookkeeping starves the 20 Hz loop. `mapping_mode:true` serializes the grid to disk every 60 s (`layer.cpp:777`) — **a literal saved course map, violating IGVC §I.2.** A finite long `voxel_decay` achieves the goal while self-healing from false marks and staying compliant. |
| **(b) Keep everything in odom, raise local/global decay to infinity** | Odom is drift-free over a course but NOT a stable WORLD anchor: an obstacle seen, left, and revisited after a loop is re-marked fresh, and integrated yaw drift (stuck Xsens gyro ~-2.86°/s, grass slip) slowly rotates ALL persisted cells with no self-correction. Cannot give world-anchored memory across a 500 ft course, and wastes the RTK investment. (Local *stays* odom — correct — but persistence belongs in the map-frame global costmap.) |
| **Revert to change-plan's "stay in odom, map promotion only behind a heading-observability gate"** | That plan predates the 2026-05-30 field run and assumed 65–96° heading rotation would recede map goals even at cm-FIX. The field run showed map-frame nav works with RTK FIX + COG-heading-at-speed + discard-motion warmup. **But the adversarial review correctly notes that proof is only 26.7 m** — so we keep the change-plan's SAFETY architecture (rtk_health gate, auto-revert, covariance floor) and only relax its "odom is the static default" conclusion *after* a long-run TF log confirms stability over distance. |
| **Re-add global semantic layer with long `tile_map_decay_time` (e.g. 30 s) immediately** | Too aggressive. Lanes are LETHAL(254) by design, the kiwicampus layer cannot clear stale out-of-FOV cells, and sooner25 emits binary-confidence single-pixel LETHAL — one glare frame walls a sub-spec corridor. Re-enable with the existing conservative 0.3 s decay first, confirm no smear, THEN raise. |
| **Unconditional `ClearEntireCostmap` in the per-cycle BT** | Directly defeats persistence — wipes the points the user wants on every tick. Clearing must be EVENT-driven (run start, RTK-degradation transition), not periodic. |

---

## Verification Plan

Run in order; each step gates the next.

1. **FRAME STABILITY (decisive gate).** With RTK FIXED confirmed (GGA quality=4, ~31 sats via MDOT CORS) and a clean fresh launch, echo `map→odom` TF stationary for 60 s **and over a >150 m drive with in-place rotations**; confirm sub-cm position + ~0° rotation drift end-to-end. **If not stable → STOP; persistence in map is unsafe; revert to odom.**
2. **PREREQUISITE BENCH.** Confirm de-skew (Issue #21) corrected; Velodyne unicast "Save Configuration" persisted; covariance floor + real Mahalanobis gate shipped and bench-validated; force a FIX→FLOAT→standalone and **measure the `map→odom` step magnitude and timing** vs the wipe latency.
3. **STATIC PERSISTENCE A/B (LiDAR).** Park a barrel ~6 m ahead, drive a slow arc so it leaves FOV; confirm the global costmap KEEPS it lethal for the chosen decay window (vs ~5 s before) and that a single transient return (wave a hand) self-clears (proves finite decay, not −1). Watch cmd_vel ≥ 18 Hz.
4. **SWITCHBACK BEHAVIOR.** Short sinusoidal barrel slalom; confirm Navfn does NOT plan a chord back into a just-passed barrel and the robot does not freeze (validates persistence didn't worsen the footprint stall — marks stable, robot drives around).
5. **RTK DEGRADATION DRILL.** With persistence active, kill NTRIP so GGA goes 4→5→1. Confirm `/rtk/fix_state` transitions, the supervisor calls `clear_entirely_global_costmap` ONCE, global lethal-cell count drops to ~0 (no smeared blob), and the LOCAL odom costmap is untouched so reactive avoidance continues. Re-acquire FIX → persistence re-arms from empty.
6. **GPS-SMEAR REGRESSION.** Global lethal-cell count over a 60 s sustained-FIX drive must NOT show 5878→24567 runaway. If it climbs, RTK is not holding FIX → disable persistence.
7. **CAMERA PERSISTENCE (second pass).** Enable global semantic layer with existing 0.3 s decay; confirm lanes/potholes persist without smearing into phantom walls; only then experiment with raising `tile_map_decay_time`. Verify a persisted lane does not wall the 5 ft min corridor.
8. **RUN-RESET / IGVC COMPLIANCE.** Kill + fresh-launch; confirm BOTH costmaps come up EMPTY (even if `rtk_health` is absent/crashed) and no `.vdb`/`.pgm` artifact was written (`mapping_mode:false` verified). Confirm the same single autonomous stack is used for qualification and the run (no persistence-only toggle — §I.4).

---

## Open Questions

1. **Does `map→odom` stability hold across a FULL 500 ft / 6 min AutoNav run, or only over the 26.7 m validated drive?** The 65–96° single-antenna heading rotation may re-appear at low speed where COG heading is unobservable. Needs a long-run TF log. **This is the dominant gate.**
2. ~~**Is the global costmap currently in `map` or `odom`?**~~ **RESOLVED 2026-05-30:** live YAML confirms `global_costmap`, `bt_navigator`, `route_server` are all `map` (`:502,21,644`); `local_costmap` is `odom` (`:243`). The frame architecture is already correct; no change needed there.
3. **Does the MTi-680G NavSatFix covariance actually shrink on FIX and inflate on FLOAT, or is it flat/optimistic?** Determines whether the `gps_covariance_relay` (rank 4) is required BEFORE persistence can be trusted, since absolute fusion + a dead gate would let a FLOAT outlier step `map→odom` before `rtk_health` can wipe.
4. **What dwell/hysteresis on `/rtk/fix_state` best balances false-wipe vs slow-wipe?** Needs a bench measurement of the actual `map→odom` step magnitude on FIX→FLOAT→standalone WITH a covariance floor in place.
5. **Is the recovery-subtree `ClearEntireCostmap` (`navigate_igvc_autonav_humble.xml:100`) firing often enough to silently defeat persistence?** If recoveries are frequent, switch it to `ClearCostmapAroundRobot` so recovery clears only local cells, not the persisted global memory.
6. **Has IGVC ruled IN WRITING that public CORS/VRS NTRIP is allowed under §I.2?** If denied at competition → `enable_ntrip:=false` + SBAS → persistence must fall back to odom reactive-only and cannot be relied on for scoring strategy.
7. **What is the right finite `voxel_decay` — tied to expected pass-by traversal time?** At ~0.6 m/s, 45 s = ~27 m (too long, blocks corridors); ~10–15 s likely suffices to prevent chord cut-back while staying well under the 60 s Blocking-Traffic threshold. Field-tune against actual pass-by durations.
