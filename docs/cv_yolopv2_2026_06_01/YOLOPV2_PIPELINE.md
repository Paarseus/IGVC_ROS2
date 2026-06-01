# YOLOPv2 Lane-Detection Pipeline — Setup, Validation & Verdict

**Date:** 2026-06-01 · **Project:** IGVC_ROS2, AutoNav · **Model:** Opus 4.8
**Scope:** Add a learned (ONNX) lane-line pipeline to `avros_perception`, validate it
**empirically** on the real IGVC frames, and give an honest deploy verdict.

> This revisits the deferral in `docs/cv_onnx_research_2026_05_31/RECOMMENDATION.md`, which
> (correctly, for that weekend) said "don't switch to ONNX mid-competition." The competition
> window (May 29 – Jun 1) is now closing — this is the *post-competition robust ONNX path* that
> same document recommended pursuing. Unlike the prior panel, this one **ran the model on the
> actual frames** instead of predicting from the training distribution.

---

## 1. TL;DR

- **Built & shipped** a fully-working `yolopv2` pipeline behind the existing `Pipeline` abstraction —
  zero downstream changes (same 4-topic kiwicampus contract, single `class_id_lane=1` mask).
- **Verified the model end-to-end**: official TorchScript → ONNX export (ONNX↔torch parity **2.6e-5**),
  preprocessing + decode derived from and confirmed against the official `demo.py`/`utils.py` source.
- **Empirical domain-gap result (the headline):** zero-shot YOLOPv2 **cleanly traces continuous
  high-contrast IGVC white tape** with **no false positives** on barrels/grass/tent — *better than
  the prior research panel predicted ("likely_poor")*. BUT it **misses short isolated stubs**
  (`exp_f00` → 0%), the documented BDD-highway-lane failure mode. Net verdict: **MARGINAL —
  promising, not yet a sole competition lane source.**
- **Recommended posture:** classical `adaptive` stays PRIMARY; run `yolopv2` as a confirming
  vote / A-B candidate; ROI-crop the horizon; fine-tune on collected frames as the real fix.

---

## 2. What was added (files)

| File | Purpose |
|---|---|
| `src/avros_perception/avros_perception/pipelines/yolopv2.py` | `YolopV2Pipeline` — ONNX Runtime session, letterbox preprocess, lane-head decode → `class_id_lane` mask + graded confidence. onnxruntime imported lazily; missing model/runtime degrades to empty mask (never crashes `_on_synced`). |
| `pipelines/__init__.py` | registered `'yolopv2'` in `PIPELINES`. |
| `perception_node.py` | declared 8 `yolopv2_*` params + added them to `_PIPELINE_PARAM_NAMES` (live-tunable). |
| `config/perception.yaml` | `yolopv2_*` config block (selector unchanged — opt-in). |
| `scripts/fetch_yolopv2_model.py` | downloads the official weights + exports the ONNX (verified reproducible). |
| `test/unit/test_yolopv2_pipeline.py` | 14 tests (pure helpers + run-contract via fake session + graceful degradation + optional real-model). Pass on numpy<2 (Jetson-like) and numpy≥2. |
| `docs/cv_yolopv2_2026_06_01/validation/` | overlays on demo + IGVC frames. |

**How to use:**
```bash
# 1. Build the model (on a box with torch+onnx, e.g. the laptop):
python3 scripts/fetch_yolopv2_model.py --out ~/yolopv2_lane_384x640.onnx
# 2. Point the pipeline at it and select it:
#    perception.yaml:  pipeline: 'yolopv2'
#                      yolopv2_model_path: '~/yolopv2_lane_384x640.onnx'
#                      process_at_full_res: true     # recommended
# 3. (Jetson) copy the .onnx over + install the aarch64 onnxruntime-gpu wheel (§5).
colcon build --symlink-install --packages-select avros_perception
```

---

## 3. Verified I/O contract (the part the code depends on byte-for-byte)

Derived from the official `CAIC-AD/YOLOPv2` (MIT) `demo.py` + `utils/utils.py` **and confirmed
by actually running the model** (`introspect.py`) and by the adversarial research panel.

- **Weights:** `https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt`
  — direct GitHub release asset, 156,380,200 B, TorchScript (PK-zip), MIT. (Trained on BDD100K.)
- **Input** `images`: `(1,3,384,640)` float32, **RGB** (BGR→RGB flip required), **`/255.0` only**
  (NO ImageNet mean/std), NCHW. Built by: optional resize to BDD framing `1280×720` → keep-aspect
  letterbox to 384×640 (pad **114**, 12 px top/bottom) → BGR→RGB → CHW → `/255`.
- **Lane output** `ll`: `(1,1,384,640)`, already **sigmoid in [0,1]**; official decode is
  `torch.round` ⇒ **threshold 0.5** (exposed as `yolopv2_lane_thresh`). Drivable head `seg` is a
  *different* head: `(1,2,384,640)` argmax — not used here.
- **Output selection is by SHAPE, not name.** Export tools differ (PINTO puts lane at name `'ll'`;
  Kazuhito00 at index 5; our direct export names it `759`). The pipeline picks the `(1,1,H,W)`
  output and logs it — robust to any re-export. *(This was the #1 silent-garbage risk the research
  flagged; the pipeline is built to avoid it.)*

Parity: `ONNX vs torch lane |maxdiff| = 2.6e-5` (effectively identical).

---

## 4. Validation results (the decisive part)

Run offline with CPU onnxruntime on the captured IGVC practice frames + the model's own demo image.
Overlays in `validation/` (yellow = lane @0.5). Threshold sweep shows the model is **confident**
(0.5 ≈ 0.3 ≈ 0.1) — lowering the threshold does NOT recover missed lines.

| Frame | lane %@0.5 | result |
|---|---|---|
| `demo_lane1` (BDD night highway) | 1.66 | **sanity OK** — traces both lane lines |
| `igvc_rgb001` (tape + barrel + tent) | 2.29 | **traces the continuous white boundary cleanly; 0 FP on barrel/grass/tent** |
| `igvc_rgb018` | 2.33 | clean trace, same scene |
| `igvc_rgb044` | 2.34 | clean trace |
| `igvc_rgb_now` (two boundary lines) | 3.07 | **both boundaries traced cleanly** |
| `igvc_exp_f00` (short white stub, bottom-right) | **0.00** | **MISSED** — short isolated mark, no lane structure |

**Interpretation.** On **continuous, high-contrast** tape the BDD-trained head transfers
*surprisingly well* and is cleaner than the classical pipeline needs per-scene tuning to achieve.
The failure is **structural and predictable**: YOLOPv2 learned long continuous/dashed *highway*
lanes, so a **short isolated stub** is invisible to it. This reconciles the empirical "works on
continuous tape" with the research's "likely_poor" prediction — both are true on different inputs.

**Untested (honest gaps):** worn/faint paint, curves, shadow/glare, the actual competition surface
& lighting, and **fp16-vs-fp32** parity on the Jetson (can flip borderline 0.5 pixels — verify on a
fixed frame). In-domain BDD lane IoU is only ~27% (paper Table 4) — so there is little margin.

---

## 5. Jetson Orin deployment (JetPack 6 / L4T R36)

1. **onnxruntime-gpu wheel — NOT from PyPI** (PyPI has no aarch64 build → CPU/Azure EP only, no GPU).
   Use the NVIDIA Jetson AI Lab index (`jp6/cu126`, cp310 aarch64):
   ```bash
   pip uninstall -y onnxruntime onnxruntime-gpu      # CPU build shadows the GPU one (same name)
   # browse https://pypi.jetson-ai-lab.io/jp6/cu126/  and grab the onnxruntime_gpu cp310 aarch64 wheel
   pip install <onnxruntime_gpu-*-cp310-cp310-linux_aarch64.whl>
   python3 -c "import onnxruntime as o; print(o.get_available_providers())"
   # MUST list CUDAExecutionProvider (and ideally TensorrtExecutionProvider)
   ```
2. **EP order** (pipeline default): `TensorRT → CUDA → CPU`. The pipeline filters to what's actually
   available, so a wrong wheel degrades to CPU (slow) instead of crashing — watch the startup log:
   `[yolopv2] loaded … | EP=[…]`. For **field reliability prefer CUDA-FP16 as the default** (zero
   engine-build wall time, no cache-invalidation foot-guns); switch to TensorRT EP only once the
   model+SDK are frozen and you've confirmed `~/.cache/yolopv2_trt` survives reboots.
3. **FP16** on (Orin tensor cores). **Verify mask parity** laptop-fp32 vs Jetson-fp16 on one frame.
4. **CPU protection:** `intra_op = inter_op = 1` (set in code) so ORT can't grab all cores and starve
   the 20 Hz MPPI loop (same failure class as RViz-on-Jetson). The conv math is on the GPU; only
   pre/post-process touch the CPU.
5. **Warmup:** the pipeline runs a dummy inference in `warmup()` — the first TensorRT inference
   *builds the engine* (tens of seconds → minutes; cache cuts it to ~9 s on reload). Don't do it on
   the first real frame.
6. **numpy<2 mandatory** (cv2/onnxruntime are NumPy-1.x ABI). The decode uses only the version-agnostic
   subset (`np.where/squeeze/astype` + `cv2.resize`) — validated on numpy 1.26 and 2.4.
7. **Latency:** no published YOLOPv2-Orin number, but comparable CSP nets put the 384×640 FP16 forward
   at ~8–20 ms (~50–120 FPS) — well under the 15 fps camera budget. The net is **not** the bottleneck;
   the cloud republish + executor are (per `perception_framedrop_rca_2026_05_31.md`).

---

## 6. Verdict & recommended posture

**Stock YOLOPv2 is a strong candidate, not yet a sole lane source.** It is clean and zero-tuning on
continuous bright tape, but its short-stub/dashed/curve recall is unproven on our course and every
false negative is a costmap gap (boundary-cross risk); every false positive is a LETHAL cell.

Recommended, in order:
1. **Keep `adaptive` PRIMARY.** YOLOPv2 runs as an A/B candidate or a *confirming vote* (e.g. mark
   lane only where classical ∧ yolopv2 agree, or fill classical's low-confidence gaps with yolopv2) —
   never let the raw NN drive the LETHAL layer alone. *(Ensemble = natural next PR; the pipeline
   abstraction makes it a small change.)*
2. **ROI-crop the horizon** (`sky_roi_poly`, default top 40% — already wired) — free, helps both.
3. **The real fix is data:** collect ZED bags on the course → auto-label with the classical mask +
   hand-correct → fine-tune a *small* net (TwinLiteNet-class / tiny U-Net), **not** full 38.9M-param
   YOLOPv2 (tiny-dataset fine-tune risks catastrophic forgetting). Then export → TensorRT → validate
   precision beats classical on faint paint *before* trusting it on the LETHAL layer.

---

## 7. Reproduce

```bash
# Build model + verify parity:
python3 scripts/fetch_yolopv2_model.py --out ~/yolopv2_lane_384x640.onnx
# Tests (CI-safe, no model/onnxruntime needed for 13/14):
cd src/avros_perception && PYTHONPATH=. python3 -m pytest test/unit/test_yolopv2_pipeline.py -q
# Re-run the field-frame validation overlays:
#   (the standalone validate.py used here lives in this doc's git history / /tmp work dir)
```

**Note (unrelated):** `test/unit/test_hsv_thresholds.py::test_hsv_thresholds_unchanged` fails at HEAD
(pre-existing) — the `sky_roi_poly 0.46→0.40` commit drifted a hash-tracked key without updating
`EXPECTED_HASH`. Not touched here (it guards *someone else's* HSV change); update its pin in that
owner's change.
