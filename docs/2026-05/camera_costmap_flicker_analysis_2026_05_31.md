# Camera Costmap Flicker — Definitive Root-Cause Analysis & Verified Change Set

**Date:** 2026-05-31
**Audience:** IGVC team lead
**Status:** Root cause confirmed; change set adversarially verified across 3 lenses (upstream-source / codebase-data / official-docs). Several originally-proposed values were **refuted and corrected** — read §4 and §5 before touching the config.

---

## 1. TL;DR

**Root cause (one sentence):** The local semantic costmap layer purges a lane tile's only LETHAL evidence after `tile_map_decay_time = 0.3 s`, but the perception pipeline's worst-case wall-clock inter-frame gap is **0.47–0.48 s** (measured live), so a marked lane tile empties before the next frame re-marks it → the cell sawtooths.

**Highest-leverage fix:** Raise `tile_map_decay_time` on the local `semantic_layer` from **0.3 → 0.6 s** (front source `nav2_params_humble.yaml:371`; mirror left `:411` / right `:448`). This is **config-side, requires a Nav2 relaunch** (the param is read once at layer construction — `ros2 param set` is a silent no-op).

**Did closing RViz fix it? NO.** Closing RViz dropped system load 9.8 → 7.04 and lifted perception 4.4 → 5.8 Hz, but the **worst-case gap stayed at 0.47–0.48 s** (still > 0.3 s decay) and the user confirmed flicker *reduced but persisted*. Load is an amplifier; the decay-vs-gap inequality is structural and load-independent. Killing the residual gnome-shell session is still worth doing, but it cannot fix this on its own.

**Two caveats the rest of this doc makes honest:**
1. There is a **second, independent flicker source**: the perception classification mask itself. The deployed pipeline is `adaptive` (`perception.yaml:34`, **uncommitted working-tree change**), created on 2026-05-30 *because* the ZED auto-exposure limit cycle made the lane mask flicker 375↔909 px frame-to-frame. Raising decay does **nothing** for per-pixel mask instability. We must A/B isolate the two.
2. The originally-circulated "raise decay to **0.8 s**" recommendation was built on **refuted arithmetic** (a double-counted "0.77 s compounded latency") and a **wrong safety claim** (a stale `vx_max=0.7`). The corrected value is **~0.6 s**, and the safety story is more nuanced than "it only lingers behind the robot." See §5 (LBC4, LBC9).

---

## 2. The evidence

Two live captures on the Jetson, same nav stack, RViz/NoMachine up vs. closed. Both captures were taken **after** the switch to the `adaptive` pipeline, so the rates below are the adaptive pipeline's, not sooner25's.

| Metric | BEFORE (RViz up) | AFTER (RViz closed) | What it proves |
|---|---|---|---|
| System load (8 cores) | 9.82 | **7.04** | Closing RViz freed ~2 cores. |
| Top CPU | rviz2 206%, gnome-shell 144% | **controller_server 93.8%, gnome-shell 56.2%** | controller_server (in-process semantic layer + MPPI) is the real heavy node once RViz is gone; gnome-shell still burns 56%. |
| Perception `semantic_mask` Hz | 4.41 | **5.79** | Perception is CPU-bound; freeing CPU lifted the *mean*. |
| Perception worst-case gap | (no min/max in BEFORE) | **mask 0.473 s, points 0.482 s, conf 0.471 s** | **DECISIVE: the max gap exceeds the 0.3 s decay by ~0.18 s — at any load.** |
| ZED rgb / cloud Hz | 8.076 / 7.968 | **8.003 / 7.957 (std 0.024 s)** | ZED rate is **load-invariant** — a deliberate 8 Hz software cap, not a load artifact. |
| Local costmap Hz | — | **7.29 (max gap 0.292 s)** | Costmap is itself starved below its configured 10 Hz target. |

**Source:** `/tmp/cam_costmap_diag.txt` (BEFORE), `/tmp/cam_costmap_diag_AFTER.txt` (AFTER), captured 2026-05-31 10:12 / 10:22.

The single load-bearing number: **perception worst-case inter-frame gap 0.482 s > tile_map_decay_time 0.3 s.** Everything else is amplifier or mechanism detail.

---

## 3. Root cause

### 3.1 Primary mechanism — decay window < perception gap

The kiwicampus semantic layer holds each tile's LETHAL evidence as a deque of `TileObservation`s. The decay is governed **solely** by `tile_map_decay_time`. Step by step, in the patched fork actually running (`src/semantic_segmentation_layer/`, origin `github.com/Paarseus/semantic_segmentation_layer.git`):

1. **Observation stamped at wall-clock arrival.** `segmentation_buffer.cpp:141`:
   `double cloud_time_seconds = clock_->now().seconds(); // FIX: use wall-clock instead of cloud header stamp` — so the relevant quantity is genuinely the **wall-time gap between delivered frames** (verified, LBC2 CONFIRMED 3/3).
2. **Every `updateBounds` purges.** `semantic_segmentation_layer.cpp:345` `current_time = node->now().seconds()`; `:368` `purgeOldObservations(current_time)`. `segmentation_buffer.hpp:245-249` pops any observation with `age > decay_time_`.
3. **A purged tile is erased and skipped.** Empty tiles are erased from `tile_map_` (`segmentation_buffer.hpp:444-460`); the marking loop does `if (tile.second.empty()) continue;` (`:373`) and only ever writes `max_cost` (254) to non-empty tiles (`:387-391`). With `samples_to_max_cost:1` / `mark_confidence:1`, **one** fresh frame re-marks 254 instantly.
4. **Net:** a tile last seen at *t* purges at *t+0.3 s*; the next frame (worst case *t+0.47 s*) re-marks it. Result: `LETHAL(t) → not-re-marked(~t+0.3..0.47) → LETHAL(t+0.47)` = one flicker cycle. Under a normal-tail approximation ~8–17% of inter-frame gaps exceed 0.3 s → ~27–55 events/min/topic.

The repo's own comment already warned about this (`nav2_params_humble.yaml:357-359`): *"Below the … frame interval cells flicker as observations are purged faster than they're refreshed."* The comment assumed a 10 Hz / 100 ms frame interval; the **measured** worst-case gap is 0.47 s, far above 0.3 s.

### 3.2 IMPORTANT correction to the mechanism (what the adversarial review changed)

The originally-stated mechanism — *"gap > decay **deterministically** drives the cell to FREE_SPACE"* — is **REFUTED** (LBC3, 2 of 3 lenses). The truth:

- The layer's `costmap_` array is **persistent** across cycles (no per-cycle `resetMaps()`); `updateWithMax` only **raises** the master grid, never lowers it (`costmap_layer.cpp:109-131`).
- Purge does **not** write FREE_SPACE. A purged cell only flips to FREE_SPACE if `raytraceFreespace` (clearing:true, runs first at `:365`) happens to walk a Bresenham ray **through that exact cell** that cycle, using the latest clearing observation.
- So the visible flicker is **frame-to-frame raytrace-vs-remark jitter**, not a clean deterministic sawtooth — and it is **amplified** by the adaptive pipeline's residual per-pixel mask instability (different cells marked each frame).

This does not change the fix (raise decay so a marked tile survives one dropped frame), but it does mean: **`combination_method` (currently unset → defaults to 1/Maximum) governs the master merge** and should be made explicit, and the fix must be validated by *watching lethal-cell count over 60 s in Foxglove*, not by assuming a 254↔0 toggle.

### 3.3 Contributing factors (amplifiers, not the root)

| Factor | Evidence | Role |
|---|---|---|
| **ZED 8 Hz cap** | `zed_front.yaml:24,34` `pub_frame_rate:8.0 / point_cloud_freq:8.0`; live 8.0 Hz unchanged before/after RViz | Upper bound on perception refresh. Deliberate software floor to relieve the in-process layer — **not** a compute ceiling (SVGA grabs 15+ fps). Raising it would re-starve MPPI (LBC5 CONFIRMED 3/3). |
| **gnome-shell 56% (leftover NoMachine session)** | `cam_costmap_diag_AFTER.txt`; CLAUDE.md:305 (gdm stopped+disabled, headless intent) | Steals ~0.5 core from the starved control loop, widening the perception gap. Safe to kill (LBC7 CONTESTED — see §5). |
| **controller_server 93.8% / costmap 7.29 < 10 Hz** | `cam_costmap_diag_AFTER.txt` | The in-process O(N) semantic layer + MPPI peg one core. The costmap itself misses its 10 Hz target → adds visible-refresh latency. |
| **`observation_persistence: 0.0`** | `nav2_params_humble.yaml:372` | Would *not* help even if raised — it is **DEAD CODE** in this fork (`observation_keep_time_` is set at `segmentation_buffer.cpp:65`, never read; `expected_update_rate` feeds `isCurrent()` which has zero callers). LBC1 CONFIRMED 3/3. **Do not tune it.** |
| **BEST_EFFORT/VOLATILE QoS** | `perception_node.py:207-214`; layer subscribes same (`semantic_segmentation_layer.cpp:194-195`, depth 50) | Can drop frames under load, but is the **correct, Nav2-standard** choice; switching to RELIABLE would not reduce drops and risks HOL-blocking. LBC6 CONFIRMED 3/3. |
| **Adaptive-pipeline mask instability** | `adaptive.py` docstring; AE limit cycle V_median 141↔167 | **Second, independent flicker source.** Raising decay does nothing for it. Must isolate. |

---

## 4. Verified change set

Ordered by leverage. **Minimal fix = C1 alone.** Belt-and-suspenders = C1+C2. Copy-pasteable targets below.

| # | Target (file:param / action) | From → To | Category | Expected effect | Claims relied on | Verdict |
|---|---|---|---|---|---|---|
| **C1** | `nav2_params_humble.yaml:371` `tile_map_decay_time` (mirror `:411`, `:448`) | `0.3` → **`0.6`** | **necessary** | Decay window safely exceeds the **measured perception worst-case gap (0.482 s)** with margin, so a marked tile survives one dropped frame. **Restart required** (read once at construction). | LBC1✓, LBC2✓, LBC3 (corrected), LBC4 | **CONFIRMED mechanism; value CORRECTED 0.8→0.6** |
| **C2** | Jetson: terminate leftover NoMachine virtual GNOME session; use laptop Foxglove | gnome-shell 56% → gone | **necessary** | Reclaims ~0.5 core to controller_server/perception; shrinks the gap from the source-rate side. Headless is the documented posture. Cannot replace C1. | LBC7 | **CONTESTED — safe-to-kill CONFIRMED; "the amplifier" OVERSTATED** |
| **C3** | `nav2_params_humble.yaml:296/360` add explicit `combination_method: 1` to local `semantic_layer` | (unset) → `1` (Maximum) | standard | Documents/locks the master-merge policy the flicker actually flips through; no behavior change (1 is the default) but removes ambiguity. | LBC3 correction | derived from REFUTED LBC3 |
| **C4** | Perception hot path: gate/remove debug overlay; move `perception_node` to `MultiThreadedExecutor` | serial `rclpy.spin` → reentrant callback group | standard | Real producer-side gap-shrink: the single thread runs 3× `cv2_to_imgmsg` + per-class overlay tint + full cloud republish serially (`perception_node.py:393-408`). Removing overlay + parallelizing cuts callback stall under load. | LBC8 (refutation) | **fixes the ACTUAL gap cause** |
| **C5** | Pair any decay raise with a periodic local-costmap clear (BT or service) | recovery BT uses `ClearCostmapAroundRobot` only | standard | Bounds out-of-FOV stale-LETHAL persistence (which raytrace does **not** clear) so a longer decay can't box MPPI after a turn. | LBC9 (refutation) | **mandatory safeguard** |
| C6 | `semantic_segmentation_layer` fork: B2 dirty-flag reset + B4 `pow→d*d`; optional 4-line mutex lock-order patch | in-process O(N) layer pegging 94% | optional | Structural headroom (projected 18–21 Hz cmd_vel) — fixes the cause, not the symptom. Unmeasured; validate after rebuild. | memory: mppi_starvation; `perception_latency_investigation.tex:49-57` | projections, unverified |
| C7 | `perception.yaml:37` `sync_slop` 0.02→0.06; `:38` `sync_queue` 2→5 | — | **optional / likely inert** | Originally "necessary." **Downgraded:** rgb and `cloud_registered` carry the **same ZED grab timestamp**, so slop=0.02 already matches ~100% of received pairs. Widening changes almost nothing. | LBC8 | **REFUTED as a lever** |
| **A1** | `nav2_params_humble.yaml:372/588` `observation_persistence`; `:379` `expected_update_rate` | `0.0` → `0.0` + annotate DEAD | **avoid** | Do not waste a field-test iteration — both are dead/no-op in this fork. | LBC1 | CONFIRMED dead |
| **A2** | `zed_front.yaml:24,34` `pub_frame_rate`/`point_cloud_freq` | `8.0` → `8.0` (do **not** raise) | **avoid** | Raising gives no perception speedup (perception already slower) and re-starves MPPI. | LBC5 | CONFIRMED |
| **A3** | Perception output QoS | BEST_EFFORT → (keep) | **avoid** | Layer subscriber is BEST_EFFORT; RELIABLE pub adds no benefit + HOL-block risk. | LBC6 | CONFIRMED |
| **A4** | `lane_white` cost / `inflation_radius` / global semantic layer / `samples_to_max_cost` | (keep) | **avoid** | Lanes stay LETHAL by policy; semantic stays local; raising `samples_to_max_cost` *delays* marking and worsens miss-sensitivity. | memory: lanes_stay_lethal | CONFIRMED constraints |

---

## 5. Load-bearing claims & adversarial verdicts

Each claim was checked by three independent lenses. Honest tally below; corrections are load-bearing.

| Claim | Verdict | Tally | Decisive citation / correction |
|---|---|---|---|
| **LBC1** — `tile_map_decay_time` is the sole temporal knob; `observation_persistence` + `expected_update_rate` are dead | **CONFIRMED** | 3✓ | `observation_keep_time_` set at `segmentation_buffer.cpp:65`, never read; `isCurrent()` (reads `expected_update_rate_`) has zero callers. **Implication:** decay is the only lever; `ros2 param set` is a no-op (relaunch needed). |
| **LBC2** — decay measured against wall-clock, so the 0.47 s wall gap is the right comparand | **CONFIRMED** | 3✓ | `segmentation_buffer.cpp:141` explicit `clock_->now()` "FIX: use wall-clock instead of cloud header stamp"; purge uses `node->now()`. Same clock both ends. |
| **LBC3** — gap>decay **deterministically** flips cell to FREE_SPACE | **REFUTED** | 1✓ 2✗ | `costmap_` is persistent (no `resetMaps`); purge does not write FREE_SPACE; `updateWithMax` never lowers the master. **Corrected:** flicker is frame-to-frame raytrace-vs-remark jitter (amplified by adaptive-pipeline mask instability), not a clean sawtooth. Fix is unchanged; framing is. |
| **LBC4** — 0.77 s "compounded" latency → decay must be 0.8 s | **REFUTED** | 0✓ 2✗ 1?| Decay age is observation-timestamp-based; the 0.292 s costmap publish gap is **check cadence, not an additive aging term** — adding them double-counts. **Corrected value: ~0.55–0.6 s** = worst perception gap (0.48 s) + margin. |
| **LBC5** — ZED 8 Hz is a load-invariant software cap; raising it can't speed perception & re-starves MPPI | **CONFIRMED** | 3✓ | Wrapper enforces rate by fixed-period `rclcpp::sleep_for` (`zed_camera_component_video_depth.cpp:2800-2812`); live 8.0 Hz unchanged across a 28% load drop. SVGA supports 120/60/30/15 fps grab — 8 Hz is not hardware. |
| **LBC6** — layer subscribes BEST_EFFORT depth 50, matching perception; RELIABLE is wrong | **CONFIRMED** | 3✓ | `semantic_segmentation_layer.cpp:194-195` = identical to Nav2 `obstacle_layer.cpp`. best_effort↔best_effort is compatible; switching pub to RELIABLE reduces no drops + HOL-block risk. |
| **LBC7** — CPU contention is "the" amplifier; gnome-shell 56% is a safe-to-kill leftover NoMachine session | **CONTESTED** | 1✓ 1✗ 1?| Safe-to-kill: **CONFIRMED** (process tree proves NoMachine virtual session; gdm masked; CLAUDE.md:305). "**The** amplifier keeping perception <8 Hz": **OVERSTATED** — `perception_node` uses only ~3.5–5.9% CPU and freeing 2 cores (RViz kill) left the 0.47 s gap unchanged. Contention starves the *in-process layer* and injects jitter; it is not throttling perception's own compute. |
| **LBC8** — `ApproximateTimeSync(slop=0.02,q=2)` sync-set drops are the dominant gap source (~28%) | **REFUTED** | 0✓ 3✗ | rgb + `cloud_registered` share the **same ZED grab timestamp** (`sl::TIME_REFERENCE::IMAGE`), so slop=0.02 matches ~100% of received pairs. Discriminator: mask 5.79 Hz ≠ points 5.36 Hz from the **same callback** → BEST_EFFORT wire-drop, not a sync drop. **Real cause: single-threaded executor serialization + DDS overwrite under load.** → C7 demoted, C4 promoted. |
| **LBC9** — decay 0.8 s is SAFE: clearing handles in-FOV; stale lane only lingers 0.56 m **behind** | **REFUTED** | 0✓ 3✗ | `raytraceFreespace` clears **in-FOV only**; purge never writes FREE_SPACE; a lane that exits the FOV **laterally while still ahead** (90° turn) stays LETHAL for the full decay window and re-stamps onto the master each cycle. **Arithmetic also wrong:** `vx_max` is **1.5** (`nav2_params_humble.yaml:106`), not 0.7 → 0.8 s = 1.2 m, not 0.56 m. **A longer decay lengthens the ahead-stall window** — the exact failure we're avoiding. Hence C5 (periodic clear) is mandatory if decay is raised. |

---

## 6. What we deliberately are NOT doing

- **Lanes stay LETHAL (254).** Lowering lane cost for an MPPI gradient was considered and rejected (stall-safe > line-touch DQ). `feedback_lanes_stay_lethal`.
- **Semantic layer stays local-only.** The global block is `enabled:false` by design — the map frame is GPS-smear-prone and lane lines would smear. `nav2_params_humble.yaml:529,567-573`.
- **Not raising the ZED rate / not making QoS RELIABLE.** Both re-starve the in-process layer in controller_server (the 8 Hz cap exists to prevent cmd_vel collapsing to ~3 Hz).
- **Decay can't be infinite (or 5 s).** Because raytrace clears in-FOV only and `updateWithMax` can't lower the master, a long decay leaves out-of-FOV-ahead-after-turn ghosts that box MPPI. Keep it just above the worst gap (~0.6 s) and pair with a periodic clear. The 5 s value seen elsewhere was a band-aid for a separate mutex bug.

---

## 7. Completeness (gaps the adversarial review surfaced)

1. **Stale config throughout the original analysis.** Pipeline is `adaptive` not `sooner25`; `vx_max` is 1.5 not 0.7; local costmap 50×50 m not 10×10; inflation 0.4. **Re-size any field action against the live YAML.** (Verified live this session.)
2. **Second flicker source.** `adaptive` pipeline exists precisely because AE-driven per-pixel mask flicker is real and *separate* from decay-purge. **Must A/B isolate** (see §8) — if mask instability dominates, C1 changes nothing.
3. **`combination_method` unset** → make explicit (C3); it governs the master merge the flicker flips through.
4. **Costmap is itself starved** (7.29 < 10 Hz) — a controller_server-94% symptom, not an additive latency term. C2/C6 address it.
5. **`collision_monitor` is pending, not running.** `inflation_radius` was cut to 0.4 m on the assumption braking/standoff comes from the (un-launched) collision_monitor. Any inflation-touching change must account for there being **no standoff layer running yet**.
6. **Mutex lock-order bug** (`perception_latency_investigation.tex:49-57`): `updateBounds` iterates `tile_map_` without its mutex — a second, independent contributor. 4-line patch lets decay drop safely. Optional but real.
7. **Fork-patch pinning:** the PR3 raytrace-clear pass and B-patches live in the vendored fork; confirm they are pinned in `avros.repos` so a fresh `vcs import` doesn't silently drop them (the decay-raise safety depends on PR3 being present).

---

## 8. Verification plan

Do this **on the Jetson, headless, with laptop Foxglove**, during a representative nav run.

1. **Baseline (before any change):** With nav running and a real lane in view, watch the local costmap in Foxglove. Confirm the lethal lane cells visibly toggle. Capture `ros2 topic hz /perception/front/semantic_mask` min/max/std and `/cmd_vel` Hz.
2. **Isolate the two sources (critical):** Bump `tile_map_decay_time` to 0.6 s, **relaunch nav**, repeat the watch.
   - If flicker **stops** → decay-purge was the dominant source (C1 sufficient).
   - If flicker **persists** → per-pixel adaptive-mask instability dominates; investigate `adaptive.py` stability (and do **not** keep raising decay).
3. **Headless A/B (C2):** Tear down the NoMachine session; confirm with `top` that gnome-shell + nxnode/nxcodec are gone. Re-measure perception min/max gap. Goal: worst gap shrinks toward/under the new decay.
4. **Safety gate (mandatory before locking decay):** Paint a lane / place a barrel, drive the robot to **turn 90° away** from it, and confirm in Foxglove the stripe directly ahead clears within ~1 s. The current code is predicted to **fail** this with a raised decay unless C5 (periodic local clear) is in place. **Do not run a scored attempt until this passes.**
5. **Control-loop health:** Throughout, confirm `/cmd_vel` holds ≥13 Hz and `/local_costmap/costmap` recovers toward 10 Hz. If raising decay pushes controller_server CPU higher, re-check.
6. **Re-measure the worst gap under FULL nav load** (controller_server 94%, side cameras if enabled) before committing the final decay value — the 0.48 s figure was captured with gnome-shell still at 56%.

---

## 9. Bottom line — exact ordered steps

**Config (requires Nav2 relaunch — `ros2 param set` is a silent no-op for these):**
1. `nav2_params_humble.yaml:371` `tile_map_decay_time: 0.3 → 0.6` (front). Mirror `:411` (left) and `:448` (right). **[C1, necessary, restart]**
2. `nav2_params_humble.yaml` local `semantic_layer`: add explicit `combination_method: 1`. **[C3, restart]**
3. Annotate `observation_persistence` (`:372`) and `expected_update_rate` (`:379`) as DEAD/no-op. **[A1, restart not needed]**
4. Ensure recovery BT (or a periodic timer) clears the **local** costmap, not just `ClearCostmapAroundRobot`. **[C5, restart]**

**Robot-side / operational (headless):**
5. Tear down the NoMachine virtual GNOME session; drive viz from laptop Foxglove. **[C2, no restart]**

**Code (rebuild required, do after the field A/B confirms decay-purge is the source):**
6. Remove/gate the debug overlay in `perception_node.py:393-408`; move to `MultiThreadedExecutor`. **[C4]**
7. (Optional) Apply fork B2/B4 + 4-line mutex patch and re-pin `avros.repos`. **[C6]**

**Do NOT:** raise ZED rate [A2], make perception QoS RELIABLE [A3], lower lane cost / raise `samples_to_max_cost` [A4], tune `observation_persistence` [A1], or widen `sync_slop`/`sync_queue` expecting a fix [C7 — inert].

**Confidence:** Primary mechanism (decay 0.3 s < gap 0.48 s, wall-clock) — high, source + live data. Corrected decay value 0.6 s — high (the 0.8 s figure was refuted arithmetic). **Open and only resolvable by the live A/B in §8 step 2:** whether decay-purge or adaptive-mask instability is the *dominant* visible flicker under the now-deployed `adaptive` pipeline. If it's the latter, this whole change set is necessary-but-not-sufficient and perception mask stability becomes the next investigation.
