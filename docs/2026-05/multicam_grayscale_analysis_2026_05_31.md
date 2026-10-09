# Multi-Camera & Grayscale Decision — 2026-05-31

**Audience:** IGVC team lead
**Scope:** Three questions, answered with verified evidence (code `file:line`, live measurements, upstream specs). Each load-bearing claim was adversarially checked from three independent lenses (upstream-source, codebase-data, official-docs); verdicts are reported honestly below, including the two that did not come back clean.

---

## TL;DR

- **Q1 — Will the two side ZED X cameras slow the system down?**
  **YES — do not enable them now.** The Jetson is already CPU-saturated on the single front camera (8 of 12 cores online in `MODE_30W`, ~70–90%/core, load ≥13). GPU and GMSL bandwidth have headroom; **CPU is the binding limit.** The first casualty is the 20 Hz MPPI control loop, not depth rate. Enabling the sides as-shipped re-triggers the documented MPPI-starvation failure (cmd_vel → ~3 Hz, goal abort). And the front camera's 110° FOV already covers both edges of an IGVC lane, so the coverage gain is marginal. Not worth it.

- **Q2 — Does grayscale make lane detection more robust?**
  **NO — robustness comes from the adaptive threshold, not the channel.** But the popular framing "color is useless / subtracts robustness" is **wrong for the deployed pipeline**: it runs an *active saturation gate* (`adaptive_max_sat: 70`) that uses chroma to reject orange barrels, the tan pillar, and green grass. A true grayscale stage would forfeit that. Grayscale is a *neutral-to-slightly-negative* change, not a win.

- **Q3 — Is the adaptive pipeline already grayscale?**
  **The threshold stage is single-channel; the full pipeline is NOT pure grayscale.** It collapses BGR to one 8-bit luminance channel before `cv2.adaptiveThreshold` (default HLS-L, `gray`/`V` selectable) — so `adaptive_channel:=gray` is a one-line param change. BUT it then reads a *second* color-derived plane (HLS-Saturation) for the clutter gate. So "flip to grayscale" is a no-op for the threshold and a feature-removal for the gate.

---

## Q1 — Side cameras: the compute math

### Current state (live, `ssh jetson` 2026-05-31)

| Resource | Measurement | Headroom? |
|---|---|---|
| Power mode | `nvpmodel -q` = **MODE_30W (ID 2)** | **No — leaves 4 cores off.** `/sys/devices/system/cpu/online` = `0-7`, `present` = `0-11`. MAXN (ID 0) would bring all 12 online + max clocks. |
| CPU cores online | **8 of 12** | — |
| Per-core load | tegrastats **70–91%@1728 MHz** (cores 9–12 `off`); load avg **13–17** | **No.** |
| GPU (`GR3D_FREQ`) | bursts **0 → 53 → 96%** on one camera; committed capture mean **34.2%**, max **99%** (bimodal: idle gaps + 80–99% peaks) | **YES — GPU is bursty with idle headroom.** |
| `zed_node` | **100% of one CPU core** (NEURAL_LIGHT depth + cloud projection) | — |
| `controller_server` (MPPI + in-process kiwicampus semantic layer) | **~94% CPU** (top ROS consumer) | **No.** |
| Front `perception_node` | input 8 Hz → output **~3.8 Hz contended** (5.5 Hz uncontended) | — |

Sources: live `nvpmodel -q` / `tegrastats` / `/tmp/bottleneck_diag.txt` (zed_node 100.0%, controller_server 94.1%); `CHANGELOG_2026-04-27.md:59-69` (GPU mean 34.2 / max 99); `docs/perception_framedrop_rca_2026_05_31.md:11,53,144`.

### What adding +2 cameras multiplies

1. **Perception/DDS path ~3×.** `perception.launch.py:23-38` spawns **one independent single-threaded `perception_node` per camera** (`rclpy.spin`, no executor → `SingleThreadedExecutor`; `perception_node.py:415`). No composition, no shared threads → **no amortization**. Each republishes a verbatim organized cloud per fire (`perception_node.py:407-408`, only `header.stamp` mutated). Front cloud = 224×128×16 = **458,752 B = 448 KB** (`/tmp/framedrop_diag.txt`).
   - **Caveat (the ~3× is likely an under-estimate):** the side YAMLs (`zed_left.yaml`, `zed_right.yaml`) run **pub/cloud at 15 Hz** (vs front's 8 Hz) and **omit `point_cloud_res`**, inheriting the larger default (front explicitly sets `REDUCED` → 448 KB; sides inherit ~COMPACT, likely ~917 KB). So per-side load is plausibly *larger* than the front's, at nearly 2× the rate.

2. **+2 `zed_node` depth pipelines.** Each runs NEURAL_LIGHT depth + organized cloud + force-enabled pos_tracking on the GPU/CPU. The front already pegs a 100% core.

### What is NOT the limit

- **GPU.** `GR3D` bursts only to 96% on one camera; Stereolabs AGX Orin data is **~linear to 4 NEURAL streams at 15 fps / ~80% GPU** ("you can reach 15FPS with 4 cameras, it's pretty linear" — community.stereolabs.com/t/performance-on-agx-orin/4406). 3 NEURAL_LIGHT streams is GPU-feasible.
- **GMSL bandwidth.** ZED Link Quad = **40 Gb/s, 4 Fakra GMSL2 ports** (stereolabs.com/store). SVGA@15 sides are far below the ceiling.

### What breaks FIRST — and the key nuance

The failure modes are **sequential**, and which one fires depends on one config line:

- **(A) On launch — box CPU/GPU/thermal saturation.** perception drops below 3.8 Hz, cmd_vel rate sags. **Launching the side cameras alone does NOT flood the costmap.** The local `semantic_layer` `observation_sources` is literally **`front` only** (`nav2_params_humble.yaml:363`). The `left`/`right` blocks (`:404-477`) are fully configured but **inert** — the kiwicampus plugin only builds buffers for tokens in that string (`semantic_segmentation_layer.cpp:98-105`, `while (ss >> source)`). The global semantic_layer is `enabled: false` (`:573`) and not in the global plugins list (`:529`).

- **(B) MPPI starvation — only on a config edit.** It fires when someone changes `observation_sources: front` → `front, left, right`, tripling the in-process O(N)-per-cloud-point SegmentationBuffer + raytrace work inside the already-94% `controller_server`. Documented result: **cmd_vel → ~3 Hz, "Optimizer fail to compute path", costmap obstacle flicker, goal abort, actuator 0.5 s cmd_vel-stale brake.** (`docs/cv_costmap_deep_analysis_2026_05_29.md` `[perf-budget-4]`; `project_semantic_layer_mppi_starvation`.)

**Conclusion:** the binding constraint is CPU, the first casualty is the MPPI loop, and the cost is *loss of stable control*, not graceful degradation.

### Is the coverage even needed? (the front FOV already covers the lane)

ZED X = **110°(H) × 80°(V)** (2.2mm lens, stereolabs.com/store; matches `camera_model: zedx` in configs). Robot-centered geometry, half-width = d·tan(55°):

| Forward distance | Lateral coverage | 10 ft lane (±1.524 m) | 20 ft lane (±3.048 m) |
|---|---|---|---|
| 1.07 m | ±1.53 m | **both edges in frame** | no |
| 2.0 m | ±2.86 m | yes | edges enter at 2.13 m |
| 5.0 m | ±7.14 m | yes | yes |

The **15° down-tilt is a pure pitch — it does not reduce HFOV** (rigid-body invariant; `avros.urdf.xacro:178` rpy `0 radians(15) 0`, zero roll/yaw; corroborated `cv_costmap_deep_analysis_2026_05_29.md:226` arch-gaps-4). Both edges of a 10 ft lane are in view from ≥1.07 m, a 20 ft lane from ≥2.13 m. Continuous lanes on a forward-planning ≤5 mph stack are re-detected from ahead as the robot advances; passed lanes are retained as costmap memory.

**Winning-team evidence agrees minimal forward sensing wins:** OU/Sooner took 2021/2023/2024/2025 with **1–2 forward cameras and no LiDAR** (`igvc_autonav_winners.json`). The only 3-camera top reference (RoboJackets) yawed its side cameras **sideways/backward (±2.05 rad)** at a steep down-tilt — a near-field bird's-eye-warp architecture, fundamentally unlike our wide forward ZED X (`robojackets/.../swervi_prop.urdf.xacro`).

### Prerequisites before side cameras could ever be enabled (ordered)

1. **`nvpmodel` MODE_30W → MAXN (ID 0)** — config-only, brings all 12 cores online + max clocks. The single biggest unused lever. *Confirm thermal/power budget first* — shared-12V-rail brown-out is a known issue; MAXN draws 50W+. (unverified on this robot)
2. **Stop RViz/NoMachine on the Jetson during nav** — RViz alone ate ~178–194% CPU live and is the documented MPPI-starvation cause; use laptop Foxglove.
3. **`perception_node` → MultiThreadedExecutor** (RCA C2) so each node stops self-dropping before you triple the count.
4. **Compose `perception_node` into the `zed_node` container, intra-process** (RCA C3, copy-before-restamp) so the 448 KB cloud isn't serialized 3× over DDS.
5. **Fix side YAMLs:** add `point_cloud_res: REDUCED` and drop pub/cloud `15 → 8 Hz` to match front.
6. **Move the kiwicampus semantic layer out of `controller_server`** to its own lifecycle node before adding left/right to `observation_sources`.
7. **Provision the right ZED X** — `zed_right.yaml` has no fixed serial (original faulted), so "3 cameras" is **not currently buildable hardware**.
8. **Verify GMSL block topology** — if front (8 fps grab) and a side (15 fps grab) share a GMSL block, same-block cameras are force-coupled to one shared grab rate.
9. **Validate empirically** — bring up ONE side camera under MAXN first, watch cmd_vel Hz, before committing to two.

---

## Q2 — Grayscale & robustness

### Robustness comes from the adaptive threshold, not the channel

The deployed `adaptive` pipeline (`pipeline: 'adaptive'`, perception.yaml:34) is exposure-robust **by construction of `cv2.adaptiveThreshold`**, not the channel choice:

- `T(x,y)` = Gaussian-weighted neighborhood mean − C; `THRESH_BINARY` fires when `src(x,y) > T(x,y)` (OpenCV docs.opencv.org/4.x/d7/d1b; `adaptive.py:120-125`, `ADAPTIVE_THRESH_GAUSSIAN_C`, `C=-8.0`).
- A **uniform exposure shift** lifts the pixel and its neighborhood mean together: `src+Δ > mean+Δ−C ⟺ src > mean−C` — Δ cancels exactly, binary result preserved. This is what defeats the ZED-X polarized auto-exposure limit cycle (V_median 141↔167 every ~0.3–0.4 s; sooner25's fixed threshold flickered lane px 375↔909) — `adaptive.py:4-15`.
- The channel (gray vs L vs V) is **incidental**: the code measures them near-identical (CV 0.029–0.031, `adaptive.py:103-104`).

### Does color help? — the honest answer (this corrects a common framing)

**The deployed pipeline actively uses chroma, and it adds real discrimination.** This is the part the simpler "color is useless on white-vs-concrete" story misses:

```python
# adaptive.py:127-135
# Low-saturation (white-paint) gate. White lane paint is near-grey
# (low HLS saturation); colored clutter — orange barrels, tan pillar,
# green grass — is high-S. ... Field-added 2026-05-31.
if max_sat < 255:
    sat = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)[:, :, 2]
    raw[sat > max_sat] = 0
```

`adaptive_max_sat: 70` is **active in the deployed config** (`perception.yaml:187`), so the running pipeline drops every HLS-S > 70 pixel. It is the mechanism that rejects orange barrels / tan pillar / green grass that the sky-ROI cannot separate by height.

Reconciling the two true facts:

- **TRUE:** white paint and light/gray concrete are *both* low-saturation, so a saturation gate **cannot separate those two surfaces** — brightness (the adaptive threshold) does that. On that narrow point, dropping chroma costs nothing.
- **ALSO TRUE (and the reason the "color subtracts robustness" framing is overstated):** saturation **does** add discrimination — it rejects *colored confounders*. The team's own committed field record shows the `S<=80` HSV gate was the load-bearing mechanism that excluded **yellow parking lines** (S>120) — `perception.yaml:79-83`, `SESSION_FINAL.md:102` ("Yellow parking lines correctly excluded by S<=80, verified iter 4"). The deep-analysis explicitly recommends *restoring* an achromatic S ceiling (~60–95) to reject glare/barrels/clothing — `cv_costmap_deep_analysis_2026_05_29.md:164-167` `[cv-pipeline-1]`.

**Verdict (Q2):** Grayscale does **not** make detection more robust — robustness lives in the adaptive threshold. But grayscale is **not a free / "color is useless" swap** either: a true grayscale input plane would forfeit the saturation gate's colored-clutter rejection (barrels, grass, yellow lines). It is **neutral-to-slightly-negative**, not a win. The "bright concrete mis-marked as a line" problem the team hit was a *brightness (Value)* overlap, never a saturation-gate failure — concrete is low-S and passes any S gate regardless.

---

## Q3 — Already grayscale?

**The threshold stage is single-channel; the full pipeline is not pure grayscale.**

Channel-select reduces BGR to one 8-bit luminance plane *before* thresholding (`adaptive.py:105-110`):

```python
channel = str(self.params.get('adaptive_channel', 'L'))   # :92
if channel == 'gray':  chan = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)        # :105-106
elif channel == 'V':   chan = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:,:,2]  # :107-108
else:                  chan = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)[:,:,1]  # 'L' default :109-110
```

- `cv2.adaptiveThreshold` then runs on that single channel (`:120-125`) — OpenCV **hard-requires** 8-bit single-channel input (passing the 3ch image raises, `thresh.cpp:1908`). So the threshold is single-channel by API contract.
- `gray` and `V` are **near-identical to the L default (CV 0.029–0.031)**, so switching is a **one-line param change** (`ros2 param set ... adaptive_channel gray`), not a redesign.
- **BUT** the pipeline then reads a **second** color-derived plane — HLS-Saturation — for the clutter gate (`:134`). So `adaptive_channel:=gray` does NOT make the pipeline grayscale; it still consumes color. A true mono pipeline (e.g. subscribing to the ZED `publish_gray` topic) would **lose access to S and break `adaptive_max_sat` clutter rejection**.

**Verdict (Q3):** YES the *threshold* is already single-channel and `gray` already exists as a selectable option ≈ the L default; NO the full pipeline is not pure grayscale, because of the saturation gate.

---

## Load-bearing claims & adversarial verdicts

Each claim checked from 3 lenses (upstream-source / codebase-data / official-docs). Reported honestly — including the contested one.

| # | Claim (abridged) | Tally (C/R/U) | Status | Key citation |
|---|---|---|---|---|
| LB1 | Jetson in MODE_30W, 8 of 12 cores online, ~70%/core, MAXN unused | 2C / 0R / 1U | **CONFIRMED** | live `nvpmodel -q`=MODE_30W/2; `/sys/.../cpu/online`=0-7, `present`=0-11; NVIDIA r36.3 power table (30W→8 cores @1728MHz) |
| LB2 | +1 camera = +1 independent single-thread `perception_node`, ~3.8 Hz cap, 448 KB cloud republish | 3C / 0R / 0U | **CONFIRMED** | `perception.launch.py:23-38`; `perception_node.py:415,407-408`; `/tmp/framedrop_diag.txt` (458,752 B) |
| LB3 | Launching sides ≠ costmap flood; `observation_sources: front` only; global semantic disabled | 3C / 0R / 0U | **CONFIRMED** | `nav2_params_humble.yaml:363,529,573`; `semantic_segmentation_layer.cpp:98-105` |
| LB4 | GPU + GMSL not the limit; CPU is | 3C / 0R / 0U | **CONFIRMED** | live `GR3D` 0–96%; Stereolabs "15 FPS @ 4 cams, linear" / 80% GPU; ZED Link Quad 40 Gb/s |
| LB5 | Pipeline already single-channel before threshold; L/gray/V near-identical | 3C / 0R / 0U | **CONFIRMED** | `adaptive.py:92-110,120-125`; OpenCV adaptiveThreshold = 8-bit 1-ch |
| LB6 | Exposure-robustness from adaptiveThreshold's local-relative T, not channel | 3C / 0R / 0U | **CONFIRMED** | `adaptive.py:10-21`; OpenCV docs `T = Gaussian mean − C, src>T` |
| LB7 | Color adds no discrimination for white lines; saturation only helps yellow | 1C / **1R** / 1U | **CONTESTED** | **Refuted by team's own `perception.yaml:79-83` + `SESSION_FINAL.md:102`: S<=80 gate excluded yellow lines (real on-course confounder); deep-analysis `:164-167` recommends restoring S ceiling. Deployed `adaptive_max_sat:70` uses chroma actively.** |
| LB8 | Front 110° HFOV covers both lane edges from 1.07 m (10 ft) / 2.13 m (20 ft); 15° tilt ≠ HFOV reduction | 3C / 0R / 0U | **CONFIRMED** | Stereolabs ZED X 110°(H); geom half_w=d·tan(55°); `avros.urdf.xacro:178` |

**LB7 is CONTESTED — and that hedge propagates into the Q2 answer above.** Do not act on "color is useless." The deployed design keeps the saturation channel *on purpose*; treat grayscale as feature-removal, not simplification.

---

## Recommendation

1. **Side cameras: DO NOT enable now.** The box is CPU-saturated on one camera, enabling the sides as-shipped re-triggers MPPI starvation (cmd_vel → ~3 Hz, goal abort), and the front 110° FOV already covers both lane edges of an IGVC AutoNav lane. If a *real* switchback/center-island coverage gap is ever observed in field test, gate enablement on the ordered prerequisites in Q1 (MAXN + no-RViz first; then executor/compose fixes; then side-YAML REDUCED@8 Hz; then move semantic layer off `controller_server`; provision the right unit; verify GMSL topology; re-measure cmd_vel Hz on ONE side first). **Highest-leverage, zero-code first step regardless: `nvpmodel` → MAXN + kill RViz on the Jetson.**

2. **Grayscale: adopt only as a marginal efficiency tweak, or skip.** It is **not** a robustness win — robustness is in `cv2.adaptiveThreshold`. If you want it, it's `adaptive_channel:=gray` (≈ identical to current L). **Do NOT** "simplify" to a true mono input (e.g. ZED `publish_gray`): that adds a plane to the already-saturated `zed_node` for zero CV gain **and** would forfeit the `adaptive_max_sat:70` saturation gate that rejects barrels/grass/yellow lines. The per-frame `cvtColor` savings on the CPU-bound box were **not measured** — do not assume "CV-neutral" implies "free CPU."

3. **The single-channel adaptive *threshold* design is correct** — exposure-invariant by construction, and the right tool for white lines on asphalt/concrete. Keep it. Keep the saturation gate too; it is doing real work.

---

## Completeness / caveats

**Verification gaps (no field data behind these):**
- **No empirical multi-camera trial.** The entire "breaks navigation" conclusion is a structural projection from a single-front-camera capture. Exact cmd_vel Hz under 2/3 cameras (with/without MAXN) is unmeasured (`code:history-constraints`: "3-camera rate is a projection, not measured").
- **Right ZED X serial is TBD** — "3 cameras" is not buildable hardware until provisioned.
- **GMSL block topology unverified** on this robot — same-block 8 fps / 15 fps coupling unresolved.
- **MAXN thermal/power feasibility unconfirmed** against the shared-12V-rail brown-out issue (50W+ draw).
- **Switchback worst case unmeasured** — if a lane edge swings to >2.86 m lateral while still <2 m ahead in a very tight corner, the front camera could momentarily lose it (medium-confidence open item; validate by driving a real IGVC switchback).
- **CV 0.029–0.031** for L/V/gray is a prior team field measurement asserted in the `adaptive.py` code comment, not re-measured this session.
- **Grayscale CPU delta unquantified** — the per-frame `cvtColor` cost (up to two BGR2HLS conversions: L + S) on the CPU-bound box was not profiled.

**Confidence:** High on the structural conclusions (CPU is the binding limit; MPPI is the first casualty; the pipeline already uses chroma; front FOV covers the lane). Lower / bounded on the precise magnitudes (exact 3-camera cmd_vel Hz, exact per-side cloud size, exact grayscale CPU savings) — these need a live multi-camera trial to pin down.
