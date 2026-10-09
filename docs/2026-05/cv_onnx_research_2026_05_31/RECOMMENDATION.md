# CV Lane-Detection Approach — FINAL Decision-Grade Recommendation

**Date:** 2026-05-31 · **Project:** IGVC_ROS2, AutoNav Challenge ONLY · **Author:** lead synthesis
**Decision context:** competition is **May 29 – June 1, 2026** (TODAY is May 31). No labeled data. No
training GPU. User wants the most standard/professional approach that **deploys fast**.

> **Brief:** `docs/cv_onnx_research_2026_05_31/CONTEXT_BRIEF.md:9-23` (decisive constraints).

---

## 1. TL;DR + the ONE recommended approach for THIS WEEKEND

**Do NOT switch to ONNX this weekend. Harden the classical pipeline you already run.**

Recommended approach: **"Harden-Classical-Now"** — three additive, zero-training changes to the
already-deployed `adaptive` pipeline:

1. **Lock the ZED video pipeline** (manual exposure/gain/white-balance) in `zed_front.yaml` — kills
   the auto-exposure limit cycle at the source, the single highest-leverage fast fix
   (`CONTEXT_BRIEF.md:67-68`). Config-only, **zero CPU**, independently shippable in ~1 h.
2. **Add a small per-pixel temporal vote** inside `adaptive.py` (K-of-N consecutive frames before a
   pixel marks lane) — converts single-frame speckle flicker into stable, precision-biased output
   before it ever reaches the **LETHAL(254)** semantic layer.
3. **Precision tune** `adaptive_C` and `adaptive_min_area` — widens the measured line-vs-speckle
   margin so residual asphalt speckle stops becoming lethal costmap cells.

All three keep the **exact 4-topic kiwicampus contract** at 224×128, single-class, same stamp
discipline — **zero downstream rewrite**, and every step reverts to today's known-good output with a
single `ros2 param set`. Field-verified state is reachable in **half a day (4–6 h)**.

**Why this and not ONNX:** research found **no pretrained lane/road ONNX model transfers cleanly
zero-shot** to 3-inch white tape on open asphalt (all are highway/urban-trained — CULane, TuSimple,
BDD100K, Cityscapes). The instinct that "classical is at its limit" is **directionally true only for
worn/faint paint**, which the captured field frames do **not** contain — so the acute failure the
brief worries about is **uncharacterized in our data**, and the model that would fix it doesn't exist
off-the-shelf. Switching this weekend trades a near-certain-to-deploy classical hardening for an
unvalidated model + an unverified onnxruntime-GPU toolchain on the exact JetPack 6 box, with every
false positive becoming an **instant lethal cell** (`nav2_params_humble.yaml:388` `samples_to_max_cost: 1`).

---

## 2. Honest ONNX verdict

### 2a. Is switching to ONNX viable *now* (this weekend)? — **NO.**

A from-scratch trained model this weekend is impossible (no labeled data, no training GPU —
`CONTEXT_BRIEF.md:14-16`). That leaves only **pretrained, zero-shot** ONNX. The research panel
benchmarked every credible public lane/road model against the actual IGVC scene
(`docs/cv_adaptive_debug_2026_05_31/input_rgb_001.png`, `input_rgb_018.png`: a low, ~15°-down-tilted
ZED looking across grey asphalt at ONE thick white-TAPE boundary, with grass + tent + orange barrel
in frame and **no road vanishing point**).

### 2b. Is ANY pretrained model deployable zero-shot? — **NONE. Name them and why:**

| Model | Training data | Output | Zero-shot to IGVC? | Verdict |
|---|---|---|---|---|
| **UFLD v1/v2** | CULane/TuSimple (highway) | sparse keypoints | **NO** — hard-codes a fixed ~4-lane highway structure converging to the horizon; nothing to lock onto. Double mismatch (domain + sparse-keypoint output ≠ dense mask). | Skip |
| **CLRNet** | CULane/TuSimple/LLAMAS | line keypoints | **NO** — same highway-anchor prior, plus non-trivial ONNX export (custom anchor/deformable ops). Export friction alone blows the budget. | Skip |
| **LaneNet / PINet** | TuSimple | seg+cluster / keypoints | **NO** — dated, TuSimple-only, clustering post-proc is highway-tuned, no clean ONNX. | Skip |
| **YOLOPv2** | BDD100K | **dense** lane + drivable masks | **POOR-to-MARGINAL** — only learned model whose output format fits (binary mask → `class_id_lane=1`) AND has a ready ONNX (`PINTO_model_zoo/326_YOLOPv2`). But BDD lanes are painted highway lines at windshield height; 3-inch tape on open asphalt is OOD. **The only model worth a strictly time-boxed 1-h curiosity trial — NOT a deploy commitment.** | Trial-only |
| **TwinLiteNet** | BDD100K | dense masks (0.4M params) | **NO** — lane IoU only **~31% on its OWN in-domain test** → no margin to survive a domain shift. Drivable-area head is "asphalt vs grass" at best, not lanes. | Skip |
| **PIDNet/DDRNet/BiSeNet/SegFormer (Cityscapes)** | Cityscapes (German urban) | 19-class seg | **NO** — **Cityscapes has no lane-marking class.** Lanes fold into "road" → would mark the whole asphalt pad (across the tape) as drivable, the *opposite* of finding the boundary. | Wrong tool |
| **SAM/MobileSAM/FastSAM/NanoSAM** | SA-1B (generic) | class-agnostic masks | **NO** — no semantics; needs prompts or a downstream classifier (rebuilds classical CV). Real value is **offline auto-labeling** (future). | Wrong abstraction |

**Bottom line: zero models deploy zero-shot for IGVC lane detection.** YOLOPv2 is the single
defensible 1-hour offline curiosity (dense mask + ready ONNX + most-diverse training set), but bias
is strong skepticism: its BDD lane head almost certainly will not cleanly segment tape-on-asphalt,
and every false positive is an instant lethal cell.

> **Sources:** `github.com/cfzd/Ultra-Fast-Lane-Detection-v2`,
> `github.com/ibaiGorordo/onnx-Ultra-Fast-Lane-Detection-Inference` (confirms "max 4 lanes …
> typical highway geometry"), `github.com/Turoad/CLRNet`,
> `github.com/PINTO0309/PINTO_model_zoo/tree/main/326_YOLOPv2`, `arxiv.org/abs/2307.10705` (TwinLiteNet),
> `github.com/XuJiacong/PIDNet`, `arxiv.org/abs/2105.15203` (SegFormer),
> `docs.ultralytics.com/models/mobile-sam`, `github.com/NVIDIA-AI-IOT/nanosam`.

### 2c. Would GPU inference relieve the CPU bottleneck? — **Largely NO, on the metric that matters.**

GPU **has** headroom (GR3D mean ~34%, `CONTEXT_BRIEF.md:86`). But the **dominant per-frame cost is
not the OpenCV ops** — it is the **448 KB organized-cloud verbatim republish** + the single-threaded
executor (`perception_framedrop_rca_2026_05_31.md:140`, **LBC3 CONFIRMED 3-0-0**: 224×128×16 =
458,752 B). The current `adaptive` pipeline's OpenCV is already cheap. So **moving inference to the
GPU would NOT fix the 8→5 Hz drop** — that needs the executor/overlay/cloud fixes (C1–C3 in the RCA)
**regardless of pipeline**. **Do not justify an ONNX switch on frame-rate grounds.**

A GPU ONNX model *could* relieve CPU **independent of accuracy** only if onnxruntime-gpu /
TensorRT-EP is actually working on this exact JetPack 6 / L4T R36 / SDK 5.2 box — which is
**UNVERIFIED in-repo** (stock pip `onnxruntime-gpu` on JP6 often ships CPU/Azure EP only; you need the
Jetson-Zoo/NVIDIA wheel). On **CPU EP**, YOLOPv2 is ~150–400 ms/frame at 640×384 → would **blow the
CPU budget and re-trigger MPPI starvation**. So GPU/TensorRT-EP is *mandatory* for any learned model,
and its feasibility is exactly the thing we cannot confirm before the field.

### 2d. Post-competition robust ONNX path (the right long-term answer)

The code's own comments correctly conclude faint/worn paint is the classical limit and "ONNX is the
robust answer" (`perception.yaml:178-179,190-191`; `adaptive.py:35`). The disciplined path:

1. **Collect data now** — record ZED RGB bags on the actual IGVC course this weekend (varied
   lighting, shadow, glare, ramp, worn sections). This is the asset you lack.
2. **Auto-label with SAM offline** — prompt SAM/MobileSAM to mask your tape lines on the recorded
   frames; hand-correct. Builds a labeled set with no manual pixel-painting.
3. **Fine-tune a small dense-mask net** (TwinLiteNet-class or a U-Net — note Sooner uses U-Net only
   to *supplement* HSV for black barrels, not for lanes — `CONTEXT_BRIEF.md:98`) on the labeled set,
   on a real training GPU off-vehicle.
4. **Export → TensorRT-EP**, validate on-Jetson that it's truly GPU-resident and CPU-cheap, and that
   per-frame precision beats the hardened classical on faint paint **before** trusting it on a
   LETHAL layer. Slot it in as the `'onnx'` pipeline already reserved at `perception.yaml:19`.

This is **weeks of work**, not hours. It is the right answer for *next* season, not this weekend.

---

## 3. Ranked options with panel scores

Panel weighting was **deploy-speed-heavy** (matches the brief). Three judges scored the winner;
the other three candidates were dominated early and not separately re-scored (n_judges=0).

| Rank | Option | deploy | robust | rules | cpu | integ. | **Weighted** |
|---|---|---|---|---|---|---|---|
| **#1** | **Harden-Classical-Now** (ZED exposure lock + temporal voting + speckle precision tune) | **5.0** | 3.0 | 3.33 | **5.0** | 4.0 | **4.07** |
| #2 | Switch to YOLOPv2 ONNX (GPU/TensorRT-EP), lane-head only, behind a time-boxed empirical gate, adaptive.py as live fallback | — | — | — | — | — | (dominated) |
| #3 | Classical adaptive + ZED organized-cloud geometric drivable-area prior (camera-frame height gate) | — | — | — | — | — | (dominated) |
| #4 | BEV/IPM occupancy (depth-cloud variant): add the missing IPM-ROI + dilation stage in-contract | — | — | — | — | — | (dominated) |

### Why #1 won

- **Deploy speed 5/5, CPU 5/5 — uncontested and independently verified.** The ZED video-lock params
  are real, `[DYNAMIC]`, override-capable (`common_stereo.yaml:42-49`); the temporal vote is a few ms
  of vectorized numpy on the already-running perception thread that **does not touch the 448 KB cloud
  republish** (the dominant per-fire cost, LBC3) → it **provably cannot re-trigger MPPI starvation**.
  No other candidate can affirmatively claim it fits the saturated Jetson.
- **#2 (YOLOPv2 ONNX) is dominated on the binding constraint:** zero-shot accuracy is unvalidated,
  GPU/TensorRT-EP feasibility is unverified on this box, and a LETHAL-on-first-sample layer turns its
  unknown false-positive rate into instant stalls. It is the *future* path, gated behind a 1-h
  curiosity trial — not a weekend deploy.
- **#3 and #4 need a NEW costmap-injection path.** A BEV/IPM occupancy grid or a camera-frame height
  gate breaks the tightest kiwicampus coupling — the layer projects mask classes through the
  **organized cloud** (`perception_node.py` cloud relay). A different output grid needs its own
  projection, a far bigger change than the 4-day window allows. Bias strongly toward in-contract.
- **#1's weaknesses are upside-not-realized, not deploy-blockers.** It does not fix worn/faint paint
  (no zero-training option does), does not fix the structural 5 Hz drop, and carries motion-blur +
  scene-specific-WB risk. All are honest caps on the **robustness 3/5** score — not reasons it won't
  run on the robot.

---

## 4. Exact change set for #1 (copy-pasteable)

### Change A — ZED exposure/WB lock (config-only, zero CPU, ship first)

**File:** `src/avros_bringup/config/zed_front.yaml`. There is **no `video:` block today** (verified —
the file jumps `general:` → `depth:`), so this is purely additive. Add under `/**: ros__parameters:`:

```yaml
    # 2026-05-31: LOCK the auto-exposure limit cycle at the source (the single
    # highest-leverage fast fix). These keys exist + are [DYNAMIC] in the wrapper
    # base common_stereo.yaml:42-49; the /**: override wins over the base.
    video:
      auto_exposure_gain: false        # MASTER AE lock (model-agnostic, sends AEC_AGC=0)
      auto_whitebalance: false         # MASTER WB lock
      # GMSL ZED X prefers the native µs/analog controls (zedx.yaml:12-16). Set
      # BOTH the legacy 0-100 and the native, then field-tune whichever the unit honors:
      exposure: 50                     # 0-100 legacy; PLACEHOLDER — field-tune
      gain: 40                         # 0-100 legacy; PLACEHOLDER — field-tune
      exposure_time: 3000              # microseconds (ZED X native); PLACEHOLDER
      analog_gain: 1500                # ZED X native; PLACEHOLDER
      whitebalance_temperature: 50     # PLACEHOLDER — field-tune
      saturation: 4
      sharpness: 4
```

> **Field-tune live** on the IGVC asphalt (these are `[DYNAMIC]`):
> `ros2 param set /zed_front/zed_node video.exposure_time <µs>` (and `.analog_gain`,
> `.whitebalance_temperature`), watching the Foxglove overlay until the asphalt is well-exposed
> (target HLS-L ~136 like `capture_stats.txt`) with the line clearly brighter. **Then write the
> chosen values back** to `zed_front.yaml`.
> **Verify the lock took:** `ros2 param get /zed_front/zed_node video.exposure_time` after launch
> (the adversarial verifier flagged that the *master* AE lock is unambiguous, but confirm the *value*
> writes on this GMSL unit — see Risks).
> **MOTION-BLUR WARNING:** a fixed exposure that looks clean stationary can smear the 3-inch line at
> 1–5 mph. Keep exposure as SHORT as the asphalt brightness allows (raise gain/analog_gain to
> compensate) and verify **while driving**, not just stationary.

### Change B — temporal voting in `adaptive.py`

**File:** `src/avros_perception/avros_perception/pipelines/adaptive.py`. Add an `__init__` to hold
the ring buffer (the pipeline is one long-lived instance per node — state persists), and apply the
vote at the end of `run()` after `clean` is computed and **before** the class-ID map at line 154:

```python
class AdaptivePipeline(Pipeline):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._vote_buf = None      # uint8 ring buffer, shape (N, H, W); reset on shape change
        self._vote_idx = 0

    def _update_votes(self, hit, n, k):
        """hit: bool HxW this frame. Return bool HxW where >=k of last n frames hit."""
        h, w = hit.shape
        if (self._vote_buf is None
                or self._vote_buf.shape[0] != n
                or self._vote_buf.shape[1:] != (h, w)):
            # First frame / resolution change: (re)allocate. Load-bearing guard —
            # a stale-shape buffer would index-mismatch and crash _on_synced,
            # stalling ALL perception (same discipline as the block-size odd guard).
            self._vote_buf = np.zeros((n, h, w), dtype=np.uint8)
            self._vote_idx = 0
        self._vote_buf[self._vote_idx] = hit.astype(np.uint8)
        self._vote_idx = (self._vote_idx + 1) % n
        return self._vote_buf.sum(axis=0) >= k
```

Then in `run()`, replace the current `clean → mask` step (lines 153-155):

```python
        # --- temporal vote: require K-of-N consecutive frames before marking ---
        vote_n = max(1, int(self.params.get('adaptive_vote_n', 3)))
        vote_k = max(1, int(self.params.get('adaptive_vote_k', 2)))
        voted = self._update_votes(clean > 0, vote_n, vote_k)   # bool HxW

        # Map binary -> class IDs for kiwicampus (single 'danger' class).
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[voted] = class_id_lane
```

> Buffer is at **process resolution (480×300)** when `process_at_full_res: true`. Set
> `adaptive_vote_k: 1` to **disable** the vote (degrades to today's behavior) if it ever erodes a
> real line under motion.

### Change C — register the two new params

**File:** `src/avros_perception/avros_perception/perception_node.py`.

(c1) Add to the `_PIPELINE_PARAM_NAMES` tuple (currently ends line 67 with
`'adaptive_max_sat', 'adaptive_blur',`):

```python
    'adaptive_max_sat', 'adaptive_blur',
    'adaptive_vote_n', 'adaptive_vote_k',
)
```

(c2) Declare them alongside the other adaptive declarations (after line ~184, in the
`adaptive_min_area` / `adaptive_blur` region):

```python
        self.declare_parameter(
            'adaptive_vote_n', 3,
            ParameterDescriptor(
                description='Temporal-vote window: frames retained in the ring buffer',
                integer_range=[IntegerRange(from_value=1, to_value=10, step=1)],
            ),
        )
        self.declare_parameter(
            'adaptive_vote_k', 2,
            ParameterDescriptor(
                description='Temporal-vote threshold: a pixel marks lane only if it hit in >=k of the last N frames (k=1 disables)',
                integer_range=[IntegerRange(from_value=1, to_value=10, step=1)],
            ),
        )
```

> This wiring is **verified sufficient** by how `adaptive_max_sat` (the most recent addition) is
> wired: declared in `__init__`, listed in `_PIPELINE_PARAM_NAMES:67`, read via
> `self.params.get(...)` in `adaptive.py:95`. Same pattern, no node-structure change.

### Change D — precision tune in `perception.yaml`

**File:** `src/avros_perception/config/perception.yaml` (adaptive block, lines 180-200):

```yaml
    adaptive_block_size: 21    # unchanged
    adaptive_C: -11.0          # 2026-05-31: -8.0 -> -11.0. Widens the brightness margin a
                               # pixel must clear above local mean. Asphalt-speckle local-excess
                               # p99 ~10-17; real-line top-1% ~26 -> -11 opens the gap. (precision-bias)
    adaptive_channel: 'L'      # unchanged
    adaptive_min_area: 120     # 2026-05-31: 80 -> 120. Drop more speckle comps; lane lines are large.
    adaptive_use_open: false   # unchanged (open erases thin lines)
    adaptive_blur: 9           # unchanged
    adaptive_max_sat: 70       # unchanged (white-paint low-S gate)
    adaptive_vote_n: 3         # NEW: temporal-vote window
    adaptive_vote_k: 2         # NEW: 2-of-3 confirm; set 1 to disable
```

> All `[DYNAMIC]` — field-tune live. If the line fragments too much: lower `adaptive_min_area` or
> `adaptive_vote_k`. If speckle returns: raise `|adaptive_C|` or `adaptive_vote_k`.

### Change E — (optional, free) gate the 0-subscriber overlay

**File:** `perception_node.py:413-423`. Wrap the overlay build/publish in
`if self._overlay_pub.get_subscription_count() > 0:`. Pure-waste removal (RCA **C1, LBC4 CONFIRMED
3-0-0**, 0 subscribers measured). ~1–3 ms + one ~86 KB publish/fire saved. Compounds the CV change;
won't alone reach 8 Hz.

### Build + deploy

```bash
# On the Jetson:
cd ~/IGVC && colcon build --symlink-install --packages-select avros_perception
# zed_front.yaml is config in avros_bringup (symlink-install picks it up); rebuild only if not symlinked:
# colcon build --symlink-install --packages-select avros_bringup
source install/setup.bash
ros2 launch avros_bringup navigation.launch.py enable_perception:=true enable_zed_front:=true
```

---

## 5. IGVC AutoNav 2026 rules-compliance matrix

Rules text: `docs/cv_onnx_research_2026_05_31/igvc_2026_rules_fulltext.txt`.

| Rule requirement | Rule cite | How #1 handles it | Compliance |
|---|---|---|---|
| **Outer boundaries = continuous OR dashed white lines ~3 in wide, taped on asphalt** | rules:282 | `adaptive` local-threshold + low-S white gate + temporal vote marks bright low-S paint regardless of continuous/dashed (dashes are separate connected comps, each ≥ `min_area`). Exposure lock stabilizes contrast. | **GOOD** on well-painted; **PARTIAL** on worn/faint (classical limit, uncharacterized in our data) |
| **Crossing internal lines not allowed; boundary crossings penalized (Careless Driving / Leave the course)** | rules:342-376 | Lanes marked **LETHAL(254)**, `samples_to_max_cost:1` (`nav2_params_humble.yaml:385-388`) → MPPI treats any detected line as a hard no-go. Precision tune + vote reduce false LETHAL stalls. **Recall holes on faint paint = crossing risk** — the honest gap. | **GOOD** when line detected; **risk** on missed faint line |
| **Simulated potholes = 2-ft solid white circles, must be avoided** | rules:293; pothole function test rules:536,568,593 | **Accidental coverage:** pipeline collapses everything detected → `class_id_lane=1` → a white circle is marked LETHAL as "lane" → **avoided, but not distinguished** as a pothole. `pothole` class is neutered (`perception.yaml:124-125`) and never marked by the layer (`nav2_params_humble.yaml:398`). | **INCIDENTALLY SAFE** (avoided), **not rule-distinguished** |
| **Ramps up to 15% grade** | rules:287 | Out of CV scope by design — LiDAR/STVL `obstacle_range` is set short so Velodyne never sees the ramp deck (MEMORY: "LiDAR range avoids the ramp"). Camera does not mark ramp surface. Watch `two_d_mode` pitch trap (MEMORY) if any height gate is ever added — **not added here**. | **N/A to CV** (by design) |
| **Min 5 ft clearance line↔obstacle; track ~10 ft wide; turning radius ≥ 5 ft; sinusoidal curves** | rules:283; brief:30 | Navfn (holonomic) + MPPI handle geometry; CV only supplies the line cost. Tracked diff-drive has 0 m turning radius. | **N/A to CV** |
| **Speed 1–5 mph; avg ≥ 1 mph or DQ; max 5 mph hardware-governed** | rules:157-164,221-226 | Actuator caps (`max_linear_mps 1.5` ≈ 3.4 mph) within band; MPPI `vx_max` tuned for grass/asphalt. **The motion-blur risk in Change A is the CV-side speed interaction** — a too-long locked exposure smears the line at 5 mph. | **GOOD** if exposure tuned short |
| **Barrels (white/orange/brown/green/black), trees, signs** | brief:32-33 | **Velodyne→STVL, NOT camera** (`nav2_params_humble.yaml:384`, camera marks ONLY lanes). `adaptive_max_sat:70` actively rejects colored barrels from the lane mask. | **By design** (LiDAR owns it) |

**Net:** #1 hardens the camera's *actual* AutoNav job (robust white line detection, precision-biased,
correct given LETHAL lanes). Potholes are incidentally safe, ramps/barrels are by-design off-camera.
The one un-closeable gap is **faint-paint recall → boundary-cross risk**, which no zero-training
option fixes this weekend.

---

## 6. CPU / latency budget (won't starve the 20 Hz MPPI loop)

**The binding limit is CPU**, and the first casualty of added load is the 20 Hz MPPI loop (documented
starvation → cmd_vel ~3 Hz → goal abort, `CONTEXT_BRIEF.md:85`, CLAUDE.md Known Issues).

| Added work in #1 | Cost | Touches MPPI-starvation drivers? |
|---|---|---|
| **ZED exposure/WB lock** (Change A) | **FREE** — same grab path, fixed instead of auto. Zero new CPU, zero GPU. | No |
| **Temporal vote** (Change B) | A few ms of **vectorized numpy** at 480×300 (~144k px): one uint8 ring-buffer write + a `sum(axis=0)` + a compare. GIL released during numpy ops. | **No** — runs on the already-running perception thread |
| **Precision tune** (Change D) | Zero new ops (same `adaptiveThreshold` + same CC loop, just different thresholds). | No |
| **Overlay gate** (Change E) | **Negative** — removes ~1–3 ms + an ~86 KB publish/fire. | Reduces load |

**The dominant per-frame cost is untouched.** The framedrop RCA confirms the **448 KB cloud
republish** (`perception_node.py:407-408`) is the single largest per-fire payload
(`perception_framedrop_rca_2026_05_31.md:140`, **LBC3 CONFIRMED 3-0-0**). #1 adds **nothing** to that
path. The MPPI starvation drivers are (a) RViz-on-Jetson CPU contention (`pkill -x rviz2`, use laptop
Foxglove — MEMORY) and (b) the in-process kiwicampus layer projecting cloud points — the cloud stays
**REDUCED** (128×224 = 28k pts, `zed_front.yaml:35`) and the mask stays 224×128. **Net: zero new MPPI
risk.**

**Honest caveat:** #1 does **NOT** raise the perception rate (still ~5 Hz on 8 Hz input — that's the
executor/cloud-serialize problem, RCA C2/C3). But: `tile_map_decay_time` is already **1.5 s**
(`nav2_params_humble.yaml:371`) ≫ the 0.47 s worst-case inter-frame gap, and the temporal vote makes
each published frame **more stable**, so the costmap sees consistent marks even at 5 Hz. Latency is
bounded by 1/rate; the framedrop RCA flags the wall-time number as **inferred, not profiled** — a
residual uncertainty, not a regression #1 introduces.

---

## 7. Deploy + verify on the Jetson (headless, laptop Foxglove)

**Ship order:** Change A first (1 h, independently shippable, biggest leverage) → then B+C+D
together → then E.

1. **Edit `zed_front.yaml`** (Change A), rebuild config or relaunch. Verify the lock took:
   `ros2 param get /zed_front/zed_node video.exposure_time` and `... video.auto_exposure_gain`
   (expect `false`).
2. **Field-tune exposure/gain/WB live** on the IGVC asphalt via `ros2 param set` while watching the
   `/perception/front/overlay` topic in **laptop Foxglove** (never RViz on the Jetson during nav —
   MEMORY). **Drive a short leg** to confirm no motion blur on the line. Lock the values back into the
   YAML.
3. **Build perception** with Changes B+C+D: `colcon build --symlink-install --packages-select
   avros_perception`. Relaunch.
4. **Verify the 4-topic contract is intact** (any break = silent frame drop):
   ```bash
   ros2 topic hz /perception/front/semantic_mask          # ~5 Hz
   ros2 topic echo /perception/front/label_info --once     # latched LabelInfo present
   ros2 topic echo /perception/front/semantic_points --field height --once   # >1 (organized)
   ```
   Mask H×W must equal cloud H×W (224×128). If kiwicampus logs "Class … not defined" or marks
   nothing, the contract broke.
5. **Stationary check** in Foxglove: overlay should trace the near/mid line cleanly with the
   **speckle dots gone** from the bottom third (compare to `output_overlay_018.png`). Lane px count
   stable frame-to-frame.
6. **Slow-drive check:** watch the local costmap in Foxglove — the lane should mark as a **stable
   LETHAL band**, not a flickering dashed set of cells. cmd_vel should hold ~13–16 Hz (not collapse to
   ~3 Hz — that's the MPPI-starvation signature).

**What success looks like:** clean line trace, no scattered lethal speckle, stable mark under motion,
cmd_vel healthy, robot holds lane. **Fallback (each reversible by one `ros2 param set`):**
- Vote erodes a real line under motion → `ros2 param set /perception_node adaptive_vote_k 1` (vote off).
- Line fragments / faint section → lower `adaptive_min_area` (e.g. 80) or `adaptive_C` (e.g. -8.0).
- Speckle returns → raise `|adaptive_C|` or `adaptive_vote_k`.
- Exposure lock under/over-exposes after a lighting shift → re-`ros2 param set video.exposure_time`,
  or worst case revert to auto: `ros2 param set /zed_front/zed_node video.auto_exposure_gain true`
  (back to today's known-working `adaptive`-with-AE behavior).

---

## 8. Load-bearing-claim verification table (including REFUTED/UNCERTAIN)

| # | Claim | Verdict | Decisive evidence |
|---|---|---|---|
| LBC-A | ZED video params (`auto_exposure_gain`/`exposure`/`gain`/`auto_whitebalance`/`whitebalance_temperature`) exist, are `[DYNAMIC]`, override-able via `/**:`, and verifiable post-launch | **CONFIRMED (HIGH)** | `common_stereo.yaml:45-49` (each `# [DYNAMIC]`); declared+read for ZED X **not** behind an isZEDX gate (`zed_camera_component_video_depth.cpp:380-400`); override layered AFTER base (`zed_camera.launch.py:397-398`); `applyAutoExposureGainSettings()` sends `AEC_AGC=0` (master lock, model-agnostic) |
| LBC-A′ | The "serial_number is silently ignored" analogy means video params may also be ignored | **REFUTED — inverted** | `serial_number` is ignored because it's a **launch-arg** appended AFTER the override file (`zed_camera.launch.py:402-422`); `video.*` are NOT in that launch-arg dict → the override-file **wins**. The lock is *more* robust than the original claim feared. **But** for GMSL ZED X prefer the native `exposure_time`(µs)/`analog_gain` (`zedx.yaml:12-16`); confirm the *value* writes on this unit (couldn't run the camera here — MEDIUM on the value path) |
| LBC-B | "Exposure lock removes the AE limit cycle, so the lock helps LESS than hoped / vote carries more weight" | **UNCERTAIN — directionally REFUTED on magnitude** | Captured frames show meanL flat 127.4–128.1 (range 0.7, `capture_stats.txt:2-38`) → AE was **NOT hunting during capture**, so the live A/B was never run (`camera_costmap_flicker_analysis_2026_05_31.md:18,129,145`). A real AE-independent residual **does** exist (18.6% of ever-marked px flicker at stable exposure). BUT simulating the documented V141↔167 swing: AE-driven churn ~895 px XOR/frame vs ~94 px residual → **if** the AE actually limit-cycles in the field, the lock removes ~90% of flicker (mostly via the exposure-coupled sat-gate, `adaptive.py:133-135`). So the lock likely helps **MORE**, not less — *provided* the AE hunts at competition, which is the one unmeasured premise |
| LBC-C | Temporal-vote state persists (one long-lived pipeline instance) | **CONFIRMED** | `perception_node.py:245` builds `self._pipeline` once; `run()` called on the same object every frame |
| LBC-D | The line is temporally MORE stable than speckle across consecutive frames at drive speed | **CONFIRMED stationary / UNVERIFIED under motion** | True for the stationary capture (line px 362–380, `capture_stats.txt`); per-pixel vote can erode a line that shifts pixels under ego-motion at 1–5 mph on a 15°-tilt camera. Mitigation built in: small N, low K, `k=1` disable |
| LBC-E | Adding the two params to `_PIPELINE_PARAM_NAMES` + `declare_parameter` makes them YAML-loadable + live-tunable | **CONFIRMED** | Same pattern as `adaptive_max_sat` (`perception_node.py:67`, `adaptive.py:95`) |
| LBC-F | The vote does NOT touch the 448 KB cloud republish → cannot worsen MPPI starvation | **CONFIRMED** | RCA LBC3 (`perception_framedrop_rca_2026_05_31.md:140`); the vote operates only on the mask path |
| LBC-G | No `video:` block exists in `zed_front.yaml` today → the change is purely additive | **CONFIRMED** | File goes `general:` (18) → `depth:` (30); no `video:` key |
| LBC-H | Quantified speckle-vs-line margin (line excess ~26 vs asphalt p99 ~10-17) holds on the real course | **CONFIRMED on captured surface / UNCERTAIN on worn asphalt** | From the captured IGVC practice frames; a washed-out/worn surface could widen the speckle band and the -11 C tune could start dropping the line. Field-tunable |
| LBC-I | No pretrained ONNX lane model transfers zero-shot to IGVC tape-on-asphalt | **CONFIRMED (research panel)** | All public models are CULane/TuSimple/BDD/Cityscapes-trained, near-horizon windshield mount; IGVC is low-tilt, single diagonal tape, no vanishing point (sources §2) |
| LBC-J | GPU/ONNX would relieve the CPU bottleneck and fix the frame drop | **REFUTED** | Dominant cost is the cloud republish + single-threaded executor, NOT OpenCV (RCA LBC3); ONNX would not fix 8→5 Hz. onnxruntime-gpu/TensorRT-EP on this JP6/L4T-R36/SDK-5.2 box is **UNVERIFIED** |

---

## 9. Open risks / what was not verifiable

1. **The central AE-hunt premise is unmeasured at competition.** The captured frames are stationary
   with already-flat exposure, so whether the ZED AE actually limit-cycles during a real moving run
   was never A/B-tested (`camera_costmap_flicker_analysis_2026_05_31.md` open question). If the AE is
   quiescent in the field, the lock buys little and the vote carries the residual; if it hunts (as the
   code documents), the lock removes ~90% of flicker. **The data cannot currently decide — only a live
   A/B can.** Mitigation: ship the lock anyway (it's free and reversible) and run the A/B in step 2.
2. **Motion blur from a fixed exposure** can smear the 3-inch line at 1–5 mph — the exact failure that
   matters. **Must** verify exposure short enough *while driving*. Highest-attention field risk.
3. **Per-pixel vote can erode a moving line.** Verified safe stationary, unverified under ego-motion.
   `k=1` escape hatch exists; consider this the first thing to disable if recall drops.
4. **Worn/faint paint is uncharacterized.** The captured frames contain ONE high-contrast taped line
   on bright even sunlit asphalt — **no** shadows-on-line, glare, ramp deck, or worn tape. Every
   adversarial lighting condition this decision is meant to survive is **absent from the only data we
   have.** Precision-first on faint paint means recall holes → boundary-cross risk. No zero-training
   option closes this; it is the future-ONNX case.
5. **GMSL ZED X may honor only the native `exposure_time`/`analog_gain`, not the legacy 0-100
   `exposure`/`gain`.** The *master* AE lock (`AEC_AGC=0`) is unambiguous; the locked-*value* path on
   this specific SDK 5.2 unit could not be run here. Set both, verify with `ros2 param get`.
6. **The 5 Hz perception rate is NOT fixed by #1.** It's a separate executor/cloud-serialize problem
   (RCA C2/C3). If the live worst-case gap exceeds `tile_map_decay_time` during motion, lane tiles
   still purge. The overlay gate (Change E) is a free partial relief; decay is already 1.5 s ≫ 0.47 s
   measured. Monitor flicker in Foxglove during the drive test.
7. **onnxruntime-gpu / TensorRT-EP feasibility on this exact JetPack 6 box is unverified** — which is
   precisely why the ONNX path is not a weekend option and is deferred to the post-competition plan.

---

### One-line verdict

**Switching to ONNX this weekend is not viable — no pretrained model deploys zero-shot to IGVC tape,
GPU inference wouldn't fix the (cloud-serialize) frame drop, and the toolchain is unverified; instead
lock the ZED exposure, add a K-of-N temporal vote, and tune for precision — all in-contract,
zero-CPU, reversible, field-ready in half a day — and collect data this weekend to fine-tune a custom
ONNX model after the competition.**
