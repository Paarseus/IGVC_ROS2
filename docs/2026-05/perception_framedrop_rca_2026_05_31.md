# Perception Frame-Drop RCA — why `perception_node` runs at ~3.8 Hz on 8 Hz inputs

**Date:** 2026-05-31
**Audience:** IGVC team lead
**Scope:** the 8 Hz → ~3.8 Hz drop in `perception_node` output, with low CPU. Companion: `docs/camera_costmap_flicker_analysis_2026_05_31.md` (downstream costmap flicker).

---

## TL;DR

**Root cause:** `perception_node` runs a **single-threaded executor** (`perception_node.py:415` `rclpy.spin(node)`). The one heavy `_on_synced` callback and the two `message_filters` input callbacks all share one thread, so while `_on_synced` is busy (~0.18–0.26 s/fire) it cannot drain the two `BEST_EFFORT KEEP_LAST depth-5` input subscriptions — incoming 8 Hz frames overwrite in the DDS ring and are dropped. **Hypotheses H2 (single-thread intake starvation) and H3 (callback wall-time) are the same mechanism and both win.** A second, co-equal driver the original candidate under-weighted: **CPU contention (H4) is not minor** — killing RViz alone lifted perception from ~3.8 to ~5.4–5.9 Hz with zero code change, so the box being CPU-saturated is part of the BEFORE 3.8 Hz, and a *residual structural cap of ~5.5 Hz* remains even uncontended.

**Highest-leverage fix:** move `_on_synced` off the intake thread (`MultiThreadedExecutor` + a `ReentrantCallbackGroup` on the two intake subscribers) **and** shed per-fire work (gate the 0-subscriber overlay, stop re-serializing the 448 KB cloud), **and** stop running RViz/NoMachine on the Jetson during nav. No single one of these reaches 8 Hz alone.

**Grayscale verdict:** **No — grayscale does not fix this.** It is a minor, secondary efficiency cleanup. It shrinks the RGB image path (~432→~144 KB) and removes one sub-ms `cvtColor`, but it does **not** shrink the 448 KB organized point cloud (the dominant per-fire payload, XYZ+RGBA, no grayscale option) and does **not** touch the executor/contention drop. Adopt it only after the executor + overlay + cloud fixes, paired with `adaptive_channel:'gray'`.

---

## 1. The measurements

All numbers are from live Jetson captures `/tmp/{bottleneck,framedrop,cam_costmap_diag,cam_costmap_diag_AFTER}_diag.txt` (2026-05-31).

### Rate chain (RViz running — contended, load 9.82–10.6)

| Stage | Topic | Rate | Source |
|---|---|---|---|
| ZED in (rgb) | `/zed_front/.../rgb/color/rect/image` | **8.02 Hz** | bottleneck_diag.txt:7 |
| ZED in (cloud) | `/zed_front/.../point_cloud/cloud_registered` | **7.97 Hz** | bottleneck_diag.txt:8 |
| perception out (mask) | `/perception/front/semantic_mask` | **3.44 Hz** | bottleneck_diag.txt:10 |
| perception out (points) | `/perception/front/semantic_points` | 3.44 Hz | bottleneck_diag.txt:11 |
| perception out (conf) | `/perception/front/semantic_confidence` | 3.65 Hz | bottleneck_diag.txt:12 |
| perception out (overlay) | `/perception/front/overlay` | 3.78 Hz | bottleneck_diag.txt:13 |
| costmap out | `/local_costmap/costmap` | **6.62 Hz** | bottleneck_diag.txt:15 |

### The decisive second capture (RViz + NoMachine **killed**, load 9.82 → 7.04)

| Topic | Rate (RViz up) | Rate (RViz killed) | Source |
|---|---|---|---|
| semantic_mask | 4.41 Hz | **5.79 Hz** | cam_costmap_diag.txt:16 → AFTER:20 |
| semantic_points | 3.63 Hz | **5.36 Hz** | cam_costmap_diag.txt:17 → AFTER:25 |
| semantic_confidence | 4.32 Hz | **5.83 Hz** | cam_costmap_diag.txt:18 → AFTER:30 |
| ZED rgb / cloud in | 8.08 / 7.97 | 8.00 / 7.96 | unchanged |

**This is the load-bearing fact the first-pass analysis missed.** The perception rate moved **+45%** with no code change, purely from freeing CPU. The AFTER capture also gives real worst-case gaps: **max 0.47–0.48 s** (AFTER:21,26,31), well above the semantic layer's `tile_map_decay_time: 0.3` (cam_costmap_diag.txt:29) — that gap drives the flicker.

### Discriminators and what each rules in/out

| Observation | Evidence | Rules IN | Rules OUT |
|---|---|---|---|
| **Fresh idle subscriber gets clean 8 Hz** for both rgb (7.97 Hz, max gap 0.14 s) and the 448 KB cloud (7.98 Hz, max gap 0.142 s) | framedrop_diag.txt:1-9 | Loss is **consumer-side** (un-drained queue) | Wire/transport drop; DDS fragmentation; H5 |
| **4 outputs from ONE callback diverge in rate** (3.44/3.44/3.65/3.78) | bottleneck_diag.txt:10-13 | Receiver-side BEST_EFFORT wire-drop of larger msgs (true callback rate ≈ overlay 3.78 Hz) | Sync-match loss (H1) |
| **rgb and cloud share identical grab stamp** | wrapper stamps both from same grab; verified bit-identical in prior changelogs | ApproximateTimeSync matches ~100% | H1 (queue_size=2 match loss) |
| **Rate rises 3.8 → 5.5 Hz when 2 cores freed** | cam_costmap_diag AFTER vs BEFORE | **H4 contention is a real co-driver** | "3.8 Hz is intrinsic / H4 minor" |
| **Even uncontended, perception caps at ~5.5 Hz < 8** | AFTER:20,25,30 | A residual **structural** cap (H2/H3 wall-time) persists | Contention as the *sole* cause |
| **perception_node absent from top-by-CPU**; CPU section empty in captures | bottleneck_diag.txt:1-4; framedrop_diag.txt:26 (empty) | Not compute-bound in the classic sense | "It's pegging a core" |
| QoS: ZED pub RELIABLE depth-10 → perception sub BEST_EFFORT depth-5 | framedrop_diag.txt:14-25 | Compatible pairing, negotiates to BEST_EFFORT | QoS incompatibility |

**Honest caveat on CPU:** the `perception_node CPU` row in framedrop_diag.txt:26 is **empty** — perception's CPU% was never directly captured. "~3.5–12%" is inferred from its absence from the top tables and a sibling doc. The "0.26 s = 31 ms compute + 230 ms I/O wait" split in the first-pass writeup is **fabricated/reverse-fit** and was refuted 2:1 by verification (see §6, LBC2). Treat the wall-time number as ~0.18 s uncontended / ~0.26 s contended, both **inferred from `1/rate`, not profiled.**

---

## 2. Root cause — the winning mechanism

### Step by step, with source

1. **Single-threaded executor.** `main()` calls `rclpy.spin(node)` (`perception_node.py:415`) with no executor argument. Per upstream rclpy (`ros2/rclpy` humble `rclpy/rclpy/__init__.py`: `spin(node, executor=None)` → `get_global_executor()` → `SingleThreadedExecutor`), this runs one callback at a time on one thread. Package-wide grep finds **no** `MultiThreadedExecutor`, `ReentrantCallbackGroup`, `callback_group=`, or `use_intra_process` anywhere in `src/avros_perception/avros_perception/`. *(Verified: LBC1 CONFIRMED 3/3.)*

2. **All callbacks share that one thread.** The two `message_filters.Subscriber` objects (`perception_node.py:270-275`) are created with no `callback_group`, so they land in the node's default `MutuallyExclusiveCallbackGroup`. `message_filters` `signalMessage` → `_on_synced` runs **synchronously inline** inside the `ApproximateTimeSynchronizer.add()` call, while the synchronizer holds its own `threading.Lock` (`ros2/message_filters` humble `src/message_filters/__init__.py`: `add()` acquires `self.lock`, then calls `self.signalMessage(*msgs)` before releasing). So `_on_synced`'s entire wall time serializes intake.

3. **Heavy `_on_synced` body** (`perception_node.py:325-408`), every fire:
   - `cv_bridge` decode bgr8 + `INTER_AREA` resize 540×960 → 224×128 (`:328,336`)
   - `pipeline.run()` — adaptive, incl. a **GIL-bound per-component Python loop** `for i in range(1, n): clean[labels == i] = 255` (`adaptive.py:131-133`) whose cost **scales with scene clutter** (speck count before `min_area`); this is the variable-cost term that makes the rate load/scene-sensitive
   - 3× `cv2_to_imgmsg` (mask, conf, overlay)
   - **unconditional overlay** with 0 subscribers (`:393-403`) — full bgr copy + per-class boolean index + bgr8 encode + publish
   - **verbatim 448 KB cloud republish** (`:407-408`) — only `header.stamp` mutated

4. **The drop.** ROS2 holds undelivered messages in the DDS middleware capped by QoS depth (official: "an incoming message ... [is] kept in the middleware until it is taken for processing by a callback"). The subs are `BEST_EFFORT KEEP_LAST depth-5` (framedrop_diag.txt:21-25). While `_on_synced` blocks, no intake runs; once >5 frames accumulate the oldest overwrite → dropped. The synchronizer's `threading.Lock` further caps **output** at `1 / (per-fire wall time)` regardless of intake parallelism.

### The arithmetic

Single thread → output rate ≤ `1 / (per-fire wall)`. Contended: `1 / 0.26 s ≈ 3.85 Hz` (matches BEFORE 3.8 Hz). Uncontended: `1 / 0.18 s ≈ 5.5 Hz` (matches AFTER 5.4–5.9 Hz). To reach **8 Hz the per-fire wall must drop below 0.125 s** AND the thread must keep draining intake.

### Honest hypothesis ranking

| H | Hypothesis | Verdict | Why |
|---|---|---|---|
| **H2** | Single-thread executor starves BEST_EFFORT depth-5 intake → overwrite drops | **WINNER (co-equal H3)** | Verified single-thread spin + inline synchronous `_on_synced`; consumer-side loss confirmed by fresh-sub clean 8 Hz |
| **H3** | Callback wall-time caps rate | **WINNER (same mechanism)** | `1/wall` arithmetic matches both contended & uncontended rates; dominated by cloud republish + overlay + the scene-variable CC loop |
| **H4** | CPU-scheduling contention | **REAL CO-DRIVER (not minor)** | **+45% rate (3.8→5.5 Hz) when RViz killed** — directly measured. Residual ~5.5 Hz cap means it is *not the sole* cause, but downgrading it to "jitter only" is **refuted by the data** |
| **H1** | ApproximateTimeSync `queue_size=2` drops matches | **REFUTED** | Identical grab stamps → ~100% match; 4-output rate spread from one callback = wire-drop, not match loss |
| **H5** | RELIABLE-pub → BEST_EFFORT-sub | **REFUTED** | Compatible pairing; fresh sub gets clean 8 Hz; wire is fine |
| **H6** | cv_bridge cost | **MINOR** | Tiny msgs at 224×128 (~tens of KB) = single-digit ms; folded into H3. The scene-variable **CC Python loop** is a bigger compute term than cv_bridge |

---

## 3. Grayscale: does it help?

**Verdict: secondary efficiency only. It does NOT fix the 8→3.8 Hz drop.**

**Why it cannot fix it:**
- The dominant per-fire payload is the **448 KB organized point cloud** republished verbatim as `semantic_points` (`perception_node.py:407-408`; 224×128 × point_step 16 = 458,752 B, confirmed in framedrop_diag.txt:11). The ZED cloud is **XYZ+RGBA at fixed point_step 16** with **no grayscale / XYZ-only option** in the pinned v5.2.2 wrapper (`point_cloud_res` only offers `COMPACT|REDUCED`, already `REDUCED`). Grayscale changes the *image* channel count, not the cloud — point_step stays 16, the 448 KB and its ~340 UDP fragments are unchanged. *(Verified: LBC3 CONFIRMED 3/3.)*
- The drop is **executor + contention**, not image color. Grayscale touches neither.
- The active **adaptive pipeline already discards color**: its first image op is `cv2.cvtColor(bgr, COLOR_BGR2HLS)[:,:,1]` (`adaptive.py:104`, default `'L'`) and `adaptiveThreshold` requires single-channel. Feeding mono8 saves exactly **one sub-ms `cvtColor`**. *(Verified: LBC8 CONFIRMED 3/3.)*

**What grayscale DOES help (general efficiency, not the bottleneck):**
- RGB image transport ~432 KB bgr8 → ~144 KB mono8 (~281 KB/frame), cheaper cv_bridge, one fewer `cvtColor`.
- The ZED wrapper publishes mono8 on `/zed_front/zed_node/rgb/gray/rect/image` (off by default, `video.publish_gray:false`; it is subscriber-lazy).

**What it does NOT help:** the 448 KB cloud, the single-thread executor drop, or zed_node's 100% core (that load is NEURAL_LIGHT depth + cloud projection, independent of image color).

**If adopted (post-fix):** set `video.publish_gray:true`, point the rgb sub at the gray topic with `desired_encoding='mono8'`, set `adaptive_channel:'gray'`, and verify no other consumer needs color. Low priority.

---

## 4. Verified change set

Ordered by leverage. **RAISE** = lifts the rate; **WORK** = reduces per-fire work; **OPS** = operational.

| # | Target (file:line / action) | From → To | Type | Expected effect | Verdict / confidence |
|---|---|---|---|---|---|
| **C1** | `perception_node.py:393-403` (overlay) | unconditional bgr copy + per-class index + bgr8 encode + publish → wrap in `if self._overlay_pub.get_subscription_count() > 0:`; pre-build color LUT at init | **WORK** | Removes pure-waste work (0 subs measured, bottleneck_diag.txt:19). ~1–3 ms + one ~86 KB publish/fire. **Will NOT alone reach 8 Hz.** Zero behavior change. Do first — lowest risk. | LBC4 CONFIRMED 3/3 |
| **C8** | Kill leftover RViz / NoMachine `gnome-shell` (~56–144% CPU) on the Jetson during nav | running → killed | **OPS** | **Measured +45% (3.8→5.5 Hz)** — the single biggest *demonstrated* lever in the data. | Per cam_costmap AFTER capture |
| **C2** | `perception_node.py:411-419` + sub callback groups | `rclpy.spin` (single-thread) → `MultiThreadedExecutor`; **intake Subscribers in a `ReentrantCallbackGroup`**, keep `_on_synced` in its own `MutuallyExclusive` group | **RAISE** | Stops overwrite-drops so intake keeps up. **Caveat (LBC9):** the synchronizer's `threading.Lock` + GIL still cap *output* at `1/wall`; reaches 8 Hz only if C1+C3 cut per-fire wall < 0.125 s. | LBC9 CONTESTED — see §6 |
| **C3** | `perception_node.py:405-408` + `perception.launch.py` | standalone Node re-serializes 448 KB cloud/fire → `ComposableNode` in zed_node container w/ `use_intra_process_comms:true` | **RAISE** (structural) | Removes the largest per-fire serialize. **Risk: see §7** — `_on_synced` mutates the cloud object in place (`:407`); under zero-copy this aliases the kiwicampus reader. Must copy-before-restamp. Sequence AFTER C2. | LBC3 CONFIRMED (payload); zero-copy benefit UNVERIFIED |
| **C4** | `perception.yaml:38` `sync_queue` | `2` → `3–5`, **only with C2** | config | No rate change (matches ~100%). Restores jitter headroom so a now-bursty intake doesn't evict a valid same-stamp pair. | H1 refuted; couple to C2 |
| **C6** | `perception_node.py:325-408` | only cv_bridge wrapped in try/except → wrap whole `_on_synced` body | safety | No rate change. Prevents an exception in `pipeline.run`/cloud relay from permanently stalling synchronized callbacks (matters more under C2). | Standard hardening |
| **C5** | `zed_front.yaml video.publish_gray` + rgb sub topic + `adaptive_channel` | bgr8 color → mono8 gray | optional | Image transport ~3×, one fewer `cvtColor`. **Does NOT raise rate.** Apply last. | LBC8 — not load-bearing |
| **C7** | QoS / `cyclonedds.xml` | — | **AVOID** | Do NOT make subs RELIABLE (back-pressures/stalls ZED), do NOT raise depth 5→10 (adds ~0.6 s stale backlog), do NOT enable SharedMemory (needs RouDi daemon, no loaned msgs). None address the root cause. | H5 refuted |

---

## 5. Load-bearing claims & adversarial verdicts

Each claim was checked by 3 lenses (upstream-source / codebase-data / official-docs).

| Claim | Status | Tally | Decisive citation |
|---|---|---|---|
| **LBC1** single-threaded executor, no MT/reentrant/callback_group/intra-process | **CONFIRMED** | 3-0-0 | `perception_node.py:415` `rclpy.spin`; grep clean; rclpy `spin()→SingleThreadedExecutor` |
| **LBC2** ~0.26 s wall = 31 ms compute + 230 ms I/O wait; 3.8 Hz is intrinsic | **REFUTED** | 0-2-1 | **Rate moved 3.8→5.5 Hz when RViz killed (cam_costmap AFTER) — load-variable, not a fixed I/O block.** CPU never measured (framedrop_diag.txt:26 empty); split is reverse-fit. No blocking primitive in `_on_synced`; the 4 BEST_EFFORT publishes are non-blocking |
| **LBC3** 448 KB cloud republished verbatim, single largest per-fire payload | **CONFIRMED** | 3-0-0 | `perception_node.py:407-408`; 224×128×16=458,752 B (framedrop_diag.txt:11); point_step=16 verified in wrapper (4×FLOAT32, *not* 32) |
| **LBC4** overlay computed+published every fire, 0 subscribers, no guard | **CONFIRMED** | 3-0-0 | `perception_node.py:393-403`; grep finds no `get_subscription_count`; bottleneck_diag.txt:19 "count: 0" |
| **LBC5** identical grab stamps → ~100% sync; output spread = wire-drop | **CONFIRMED** | 2-0-1 | bottleneck_diag.txt:10-13 spread from one callback. *Correction: stamp identity is from the shared ZED grab, NOT `sensors_image_sync` (that param only syncs IMU/baro/mag and is inert here).* |
| **LBC6** fresh idle sub gets clean 8 Hz for rgb + 448 KB cloud | **CONFIRMED** | 3-0-0 | framedrop_diag.txt:1-9 (cloud 7.98 Hz, max gap 0.142 s). Proves wire/QoS fine; loss is at the un-drained queue. (Does not by itself pick H2 vs H1/H3 — all consumer-side.) |
| **LBC7** perception not CPU-starved; freeing cores (10.75→3.5) left 0.47 s gap unchanged | **REFUTED** | 0-2-1 | **The "10.75→3.5" numbers are misattributed** (from a 2026-05-21 STVL incident, CLAUDE.md:409); actual experiment was 9.82→7.04. **No BEFORE worst-case gap exists** (cam_costmap_diag.txt has no min/max). Freeing cores **RAISED mean 4.41→5.79 Hz.** "5–6 idle cores" is false — load 7.04 on 8 cores ≈ near-saturated. H4 is a real co-driver, demoted only as *not the sole* cause |
| **LBC8** adaptive pipeline discards color first op; grayscale saves only 1 sub-ms cvtColor | **CONFIRMED** | 3-0-0 | `adaptive.py:99-104, 112-117`; OpenCV docs require single-channel src |
| **LBC9** MT executor removes drops but needs per-fire wall < 0.125 s for 8 Hz | **CONTESTED** | 1-1-1 | `threading.Lock` in synchronizer caps output at `1/wall` (confirms the warning). But the doc's prescribed fix is MT + **reentrant** group; on low-CPU/GIL-releasing waits a reentrant group *may* overlap fires and exceed the cap — empirically open. Also the C1/C2/C3 labels here differ from the sibling flicker doc (its overlay+MTExecutor is a single "C4") |

---

## 6. What NOT to do / regressions

1. **Don't raise `sync_queue` back toward 10.** The 2026-05-12 `10→2` cut (`perception.yaml:38`) fixed a **separate** failure: old synced pairs accumulating **~10 s of lag** → downstream kiwicampus MessageFilter drops. That was *stale-message latency*, a different axis from match-loss. Raising it without C2 gains no throughput (matches are already ~100%) and risks resurrecting the lag. Only raise to 3–5 **alongside** C2.

2. **Don't break the kiwicampus contract.** Mask and cloud **must share `header.stamp`** (`perception_node.py:375-380,384,407`); the cloud **must stay organized** (`height>1`); `LabelInfo` must stay `RELIABLE`/`TRANSIENT_LOCAL` (cam_costmap_diag.txt:42-43). C3 (composition) must preserve all three — do not drop the cloud, do not flatten it.

3. **C3 zero-copy aliasing hazard.** `_on_synced` mutates `cloud.header.stamp` **in place** on the same object the synchronizer delivered (`:407`). Under `use_intra_process_comms` this object is passed by pointer to the kiwicampus subscriber — restamping it in place can race the reader. **Copy/clone the header before restamp** under C3 (this negates part of the zero-copy benefit; measure before/after).

4. **GIL limits of `MultiThreadedExecutor` in Python.** cv2/numpy release the GIL during C work, but the **per-component Python loop** (`adaptive.py:131-133`) and per-frame param dict reads hold the GIL and do **not** overlap. So C2's parallelism is partial; don't expect linear scaling.

5. **Don't switch subs to RELIABLE or enable SharedMemory** (C7). Both are non-fixes; RELIABLE back-pressures the ZED node, SHM needs a RouDi daemon the project disabled.

---

## 7. Verification plan

Before/after each, capture: perception output `ros2 topic hz` (mask/points/conf, full stats incl. **max gap**), ZED input hz, and a **`time.perf_counter()` log around `_on_synced` entry/exit + each publish + `pipeline.run`** (none exists today — grep finds no profiling). Run **both contended and uncontended** to separate H4 from the residual cap.

| Step | Action | Metric to watch | Predicted result |
|---|---|---|---|
| **V0 (do this first — free, no build)** | Kill RViz + NoMachine gnome-shell; re-measure | perception mask Hz; load avg | ~3.8 → ~5.5 Hz (already seen). Quantifies H4's share |
| **V1** | Add `_on_synced` timing log; capture per-stage ms (contended + uncontended) | split: pipeline.run (CC loop) vs 3× cv_bridge vs cloud publish | Converts the keystone wall-time from inference to fact; settles cloud-serialize vs CC-loop |
| **V2** | C1 overlay gate; rebuild | overlay Hz (→0 when unsubscribed), mask max gap | Small wall-time drop; rate ~unchanged but cleaner |
| **V3** | C2 MT executor + reentrant intake; rebuild | mask Hz, **dropped-frame count** (input vs output) | Drops fall; output still ~`1/wall` until V4 |
| **V4** | C3 composition (with copy-before-restamp); rebuild | cloud publish duration; mask Hz | Per-fire wall falls; rate climbs toward 8 Hz |
| **V5 (control)** | Set `sync_queue` back to 10 on live node, re-measure | mask Hz | **No change** → empirically confirms H1 inert |
| **V6 (optional)** | C5 grayscale topic + `adaptive_channel:'gray'` | image bytes, zed_node CPU, mask Hz | Smaller transport; **rate ~unchanged** (proves grayscale secondary) |

Verify on-target prerequisites before V3: confirm the Jetson's `message_filters` `Subscriber` accepts/propagates a `callback_group` in Humble, and confirm intra-process actually zero-copies the PointCloud2 (the Python node uses no loaned messages — it may fall back to a copy).

---

## 8. Bottom line — ordered steps

| Order | Step | Kind | Reaches 8 Hz? |
|---|---|---|---|
| 1 | **Kill RViz + NoMachine on the Jetson during nav** (C8) | operational | No, but +45% measured — biggest free win |
| 2 | **Gate the 0-subscriber overlay** (C1) | code → rebuild | No (shed work) |
| 3 | **Add `_on_synced` timing** (V1) and re-measure both contended/uncontended | code → rebuild | diagnostic — settles the wall-time split |
| 4 | **MultiThreadedExecutor + ReentrantCallbackGroup on intake** (C2) | code → rebuild | Removes drops; not alone |
| 5 | **Compose into zed_node container, copy-before-restamp cloud** (C3) | code + launch → rebuild | Cuts the dominant payload; this + C1 + C2 is the path to ~8 Hz |
| 6 | **Raise `sync_queue` 2→3–5** (C4) | config | headroom only |
| 7 | **Wrap `_on_synced` in try/except** (C6) | code → rebuild | safety |
| 8 | **Grayscale** (C5) | config + code | No — efficiency cleanup only |

**Rebuild vs config vs ops:** C1, C2, C3, C6 need `colcon build --symlink-install --packages-select avros_perception` (C3 also touches the launch file). C4, C5(YAML half) are config-only. C8 is operational. **Do C8 + C1 immediately (free / low-risk); they alone should move ~3.8 → ~6 Hz. C2 + C3 are the structural fixes that close the rest of the gap to 8 Hz — but no single change does it, and the keystone per-fire wall-time number is still inferred (`1/rate`) and should be confirmed by the V1 timing log before committing the C2/C3 refactor.**
