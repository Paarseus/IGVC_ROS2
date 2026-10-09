# avros_perception — Review

## Summary

`avros_perception` (~847 LOC source, ~1018 LOC tests) is the camera-perception bridge that converts a ZED RGB+organized cloud into the four-topic surface (`semantic_mask` mono8, `semantic_confidence` mono8, `semantic_points` organized PointCloud2, latched `label_info`) consumed by the kiwicampus `semantic_segmentation_layer` Nav2 plugin. It is the most-tested package in the workspace and the only one with a >1.0 test:source ratio.

**Architecture is sound.** The `_on_synced` callback inherits `image.header.stamp` and `image.header.frame_id` and copies them onto every output, resizes the image to the cloud's HxW *before* running segmentation (the canonical fix for ZED's image-vs-cloud resolution split), uses `qos_profile_sensor_data` on inputs and a properly-configured `TRANSIENT_LOCAL + RELIABLE + KEEP_LAST(1)` QoS for `LabelInfo`, and exposes runtime-tunable HSV bounds via a validated `add_on_set_parameters_callback`. ZED v5 topic naming is baked in correctly. The pipeline abstraction (Stub / HSV / ONNX) is minimal and the registry pattern is idiomatic.

**Two P0 issues exist:** (1) `LabelInfo.header.frame_id` is published empty rather than the camera optical frame (`perception_node.py:212` calls `build_label_info(self._classes)` without a `frame_id`), violating the standards §1 contract though kiwicampus tolerates it today. (2) HSV barrel detection uses a single-range `cv2.inRange` on hue without wraparound handling — under sunset/sunrise lighting where orange shifts toward red across the H=0/179 boundary, the pipeline silently misses pixels. Standards §8 explicitly warns about this.

**P1 issues are mostly robustness gaps:** the `_on_synced` callback isn't exception-safe (a single bad frame from a buggy pipeline can halt synchronized callbacks), `pipeline` parameter swap is documented but not implemented (read-once at startup), numeric class IDs are duplicated across `class_map.yaml` / `perception.yaml` / `hsv.py:31` rather than name-resolved per standards §10, and the integration test in `test_perception_launch.py` doesn't actually publish synthetic image+cloud so the synced-callback header propagation is unverified end-to-end.

**Test quality is the workspace high-water mark.** Includes a hash-based threshold-drift guard (`test_hsv_thresholds.py` — every HSV change requires updating `EXPECTED_HASH` in the same PR), `launch_pytest` integration test verifying late-joining LabelInfo subscription, mock `ApproximateTimeSynchronizer` slop tests, and live-tuning tests proving the shared-mutable-params pattern. Coverage gaps are at the integration glue: no test of the HxW resize, header propagation, or pipeline crash recovery.

Verdict: this package is competition-ready after the two P0 fixes (~2 hours of work each) and is in better shape than the rest of the workspace.

## Per-file findings

### perception_node.py

`src/avros_perception/avros_perception/perception_node.py` (344 LOC). The only ROS-facing executable: subscribes to one ZED camera's rectified RGB + organized cloud, runs a swappable Pipeline, republishes mono8 mask + mono8 confidence + relayed organized cloud + latched LabelInfo under `/perception/<cam>/...`.

Strengths:

- **Latched LabelInfo correctly configured** (lines 202–206): `QoSProfile(depth=1, RELIABLE, TRANSIENT_LOCAL)`. Published once at startup (line 214). Standards §1, §9.
- **Stamp/frame inheritance is bit-exact** (lines 301–302, 305–306, 311, 322, 327). `image.header.stamp` and `frame_id` are copied verbatim onto mask/conf/overlay/cloud — no `now()` regeneration. Standards §2 invariants 1–2.
- **HxW invariant enforced before pipeline runs** (lines 287–291): `cv2.resize(bgr, (cloud.width, cloud.height), INTER_AREA)`. Inline comment notes "cloud is the costmap-side truth." Canonical fix for ZED `pub_downscale_factor` vs `point_cloud_res` divergence (standards §7). Post-pipeline shape check (295–299) is defensive.
- **`qos_profile_sensor_data` on inputs and outputs** (lines 188–201, 219–224). Standards §3.
- **Live-tunable parameters via `add_on_set_parameters_callback`** (236–272). Validates HSV bounds element-wise (`H≤179, S/V≤255`), checks low≤high pairwise, returns `SetParametersResult(successful=False, reason=...)`. Atomic stage-then-merge pattern. Pipeline reads `self._pipeline_params` on every frame so `ros2 param set` takes effect on next frame.
- **Shutdown correct** (331–340): `try/except KeyboardInterrupt`, `finally destroy_node() + try_shutdown()`.

Issues:

- **P0 — `LabelInfo.header.frame_id` is published empty.** Line 212 calls `build_label_info(self._classes)` with default `frame_id=''` (see `class_map.py:36`). Standards §1 requires the optical frame. Fix: defer first publication until first synced frame so `image.header.frame_id` is known, or accept a `label_info_frame_id` parameter.
- **P1 — `_on_synced` callback is not exception-safe.** Only the `cv_bridge` call (lines 275–280) is guarded. `pipeline.run()` (294), cloud relay (328), or the three downstream `cv2_to_imgmsg` encodings can raise. Exceptions propagate through `message_filters` and may stop further synchronized callbacks. Wrap entire callback in `try/except Exception` with throttled error logging.
- **P1 — Pipeline hot-swap is documented but unimplemented.** `pipeline` param read once at line 145; not in `_live_param_names` (line 183). `ros2 param set ... pipeline stub` returns `successful: True` but does nothing. Either implement rebuild in `_on_set_params` or document the restart-required limitation.
- **P1 — `cv_bridge` failure logging doesn't include `image.encoding`.** Line 279: `cv_bridge failed: {e}` — add the source encoding for triage.
- **P1 — HSV bound parameter declarations lack explicit `ParameterType.PARAMETER_INTEGER_ARRAY` descriptor** (lines 123–134). Type is inferred from defaults; a YAML override with one float (e.g. `[0, 0, 180.0]`) flips inference to `DOUBLE_ARRAY` and breaks the `int(x) for x in triplet` cast in `_on_set_params`.
- **P2 — `_on_set_params` silently accepts unknown parameter names** (line 243: `continue`). User typos like `lain_low` return `successful: True` but no-op. Acceptable today.
- **P2 — Overlay publisher allocates per-class color array per frame** (315–319). Pre-build at init.
- **P2 — `LabelInfo` not republished on class-map change** (line 214 latches once, no callback for `class_map_path`). Acceptable for IGVC AutoNav (palette stable mid-run); flag with TODO.

Frame-id correctness: `image.header.frame_id` flows through to mask/cloud/conf/overlay headers — assuming ZED publishes in the optical frame (`zed_<cam>_left_camera_frame_optical`), which it does. The LabelInfo header is the only gap.

### pipelines/base.py

`pipelines/base.py` (34 LOC). `PipelineResult` dataclass + `Pipeline` ABC with `__init__(params, logger)`, `warmup()`, `run(bgr, depth=None)`.

- **Strength** — `run()` accepts BGR matching OpenCV/cv_bridge convention; `depth` slot reserved for ONNX. `warmup()` is the right hook for ONNX session-load (called at `perception_node.py:177`).
- **P2** — Not declared via `abc.ABC` / `@abstractmethod`. Trivial fix; lets type checkers catch missing `run` impls.
- **P2** — No dtype/shape enforcement on `mask`/`confidence`. Currently caught downstream at `perception_node.py:295` (shape only, not dtype).

### pipelines/stub.py

`pipelines/stub.py` (36 LOC). Plumbing-validation pipeline that paints a vertical band of class-ID into a zero mask.

- **Strengths** — Reads params from `self.params` every frame for live tunability; defensive width/start clamps (lines 27, 30); `class_id & 0xFF` coerces to mono8 range; auto-center on `start=-1`.
- **P2 — Confidence semantics differ from HSV.** Stub: `confidence = 255` everywhere (line 35). HSV: `255 where mask>0, else 0`. kiwicampus's `mark_confidence` knob will treat them differently. Document the convention in `pipelines/base.py`.

### pipelines/hsv.py

`pipelines/hsv.py` (151 LOC). IGVC-grass classical pipeline: per-class `cv2.inRange` thresholds (Sooner Robotics lineage), iscumd-style adaptive V-floor, polygonal sky/tree ROI.

Strengths:

- **BGR→HSV via `cv2.COLOR_BGR2HSV`** (line 116). Hue range honors OpenCV's `[0, 179]` (standards §8).
- **Live params re-read every frame** (lines 98–110). Cost negligible vs cv2 ops.
- **Per-class priority documented** (137–143): pothole < barrel < lane (last write wins).
- **`_reshape_poly` accepts flat or pair forms** (62–74) — ROS 2 params can't hold list-of-list, so flat is canonical.

Issues:

- **P0 — Hue wraparound for orange/red unhandled.** Single-range `cv2.inRange` on barrel `H ∈ [5, 25]` (line 103–104). Under sunset lighting orange shifts toward red across H=0/179; current code silently misses those pixels. Standards §8 explicitly warns. Fix: support second-range pair (`barrel_low_2`, `barrel_high_2`) and OR the masks.
- **P1 — Adaptive V-floor `k=0.0` doesn't actually disable the bright filter.** YAML comment at `perception.yaml:42-46` claims it does, but lines 120-123, 127 still compute `_v_floor = mean(V)` and AND with `bright = (V >= mean(V))`. ~Half the pixels pass. Either short-circuit `bright` to all-pass when `adaptive_k == 0` or rewrite the comment.
- **P1 — Adaptive V-floor lag** with `adaptive_period > 1`. Default 5 at 15 Hz = 333 ms lag after orientation changes. Drop to 1.
- **P1 — `pothole` class skips adaptive-V gate** (lines 128–129) yet its bounds are nearly identical to lane (per `perception.yaml:62`, both target white paint). May match noise in shadows that `lane` rejects. Either consolidate or document.
- **P1 — Morph kernel size hardcoded 3×3** (line 50). Add `morph_kernel_size` param.
- **P2 — `_DEFAULT_CLASS_IDS` const (lines 31-35) duplicates `class_map.yaml` and `perception.yaml`.** Standards §10 anti-pattern (numeric IDs hardcoded). Look up by name.
- **P2 — `_reshape_poly` raises on odd-length flat list but `sky_roi_poly` is not validated in `_on_set_params`** (perception_node.py allowlist excludes shape-checking it). Bad param crashes pipeline.run().

### pipelines/__init__.py

25 LOC. `PIPELINES = {'stub': ..., 'hsv': ...}` registry + `build_pipeline(name, params, logger)` factory. ONNX is referenced in YAML but not registered — passing `pipeline: onnx` raises a loud `ValueError`. Idiomatic, no issues.

### utils/class_map.py

44 LOC. `ClassEntry` dataclass + `load_class_map(path)` + `build_label_info(entries, frame_id='')`.

- `yaml.safe_load`, defensive int/str/tuple casts, mid-gray RGB default (line 31). RGB stored outside `LabelInfo` per standards §1.
- **P1** — `frame_id=''` default is the root cause of the LabelInfo P0. Make it a required parameter.
- **P2** — `LabelInfo.threshold` not set (defaults to message zero); fine for HSV, set explicitly for ONNX.
- **P2** — No bounds check on `class_id` against `uint16` max.

### config/class_map.yaml

19 lines. Five classes: `free` (0), `lane_white` (1, cyan), `barrel_orange` (2), `pothole` (3), `unknown` (255). Header comment correctly cross-references `nav2_params.yaml` `class_types`.

- **P2** — `lane_white` RGB is `[0, 255, 255]` (cyan, not white). Visual quirk for RViz; harmless for kiwicampus.
- **P2** — No element-bounds validation on RGB.

### config/perception.yaml

75 lines. `/**:` wildcard root for multi-camera launches.

- **Strengths** — `sync_slop: 0.02` (50 Hz tolerance, well inside standards §3 rule of thumb for 15 Hz), `sync_queue: 10`, inline calibration history with dates/reasons (lines 39-46, 51-54, 67-73), HSV ranges respect OpenCV's `[0, 179]`.
- **P1** — Lane bounds disagree between YAML (`[0, 0, 80]..[179, 120, 255]`, lines 53-54) and in-code defaults (`[0, 0, 180]..[179, 60, 255]`, perception_node.py:123-124). YAML wins via launch, but a CLI `ros2 run` without the YAML uses wrong defaults. Drop the in-code defaults.
- **P2** — No YAML comment warning that `pipeline` is read-once.
- **P2** — `sky_roi_poly` format gotcha not surfaced in `ParameterDescriptor.description`.

### launch/perception.launch.py

58 LOC. Multi-camera spawn via `OpaqueFunction` + comma-split camera list. YAML+dict parameter merging.

- **Strengths** — `OpaqueFunction` is the right escape hatch for comma-splitting (standards §4); `name=f'perception_{cam}'` pairs with `/**:` wildcard for scoping; YAML-first then dict-override merge.
- **P1** — `DeclareLaunchArgument('camera_name', ...)` at line 53 is declared but never read. Drop it or honor as single-camera shortcut.
- **P2** — No `log_level` launch arg.

### tests (group)

8 unit + 1 launch test (~1018 LOC) plus boilerplate copyright/flake8/pep257. Empty `test/data/` and `test/fixtures/` reserved for field corpus.

- **`conftest.py`** — pins `cv2.setNumThreads(1)` and `np.random.seed(0)` for determinism. `load_image` fixture skips on missing file (graceful degradation).
- **`test_perception_launch.py`** (124 LOC) — `launch_pytest` integration. Spawns the real node with `pipeline: stub`, late-joins a `_TopicWatcher` (line 107), asserts latched LabelInfo arrives within 10 s (line 115). **P1:** doesn't publish synthetic image+cloud, so `_on_synced` and the mask/conf/cloud outputs are unverified end-to-end (the watchers exist but only `seen['label']` is asserted).
- **`test_pipeline_interface.py`** (74 LOC) — Pipeline ABC contract. `run()` on base raises `NotImplementedError`, `params=None` → `{}`, subclass dtype/shape match input.
- **`test_stub_pipeline.py`** (108 LOC) — 9 tests including the crucial `test_stub_stripe_params_mutation_takes_effect_next_run` that proves the shared mutable params dict pattern.
- **`test_hsv_pipeline.py`** (185 LOC) — 11 tests on synthetic numpy frames. Covers shape, non-3-channel raises, mid-gray is free, white-patch is lane, orange-patch is barrel, lane-wins-priority, ROI polygon, confidence semantics, adaptive V-floor first-frame init, custom class IDs, registry. Uses `adaptive_k=-100` baseline to isolate HSV thresholds. **Gap:** no test for hue wraparound (the P0 above).
- **`test_hsv_thresholds.py`** (78 LOC) — SHA256 hash guard on HSV tuning fields. Excellent governance pattern; forces threshold changes through PR review.
- **`test_label_info.py`** (106 LOC) — class-map round-trip; explicitly tests `frame_id` propagation through `build_label_info`. The fact that `perception_node.py:212` doesn't pass a frame_id is therefore an unenforced contract — the regression test could be extended to catch it.
- **`test_sync_slop.py`** (149 LOC) — Mock `ApproximateTimeSynchronizer`. 6 tests covering identical/within/beyond slop, ordering, replacement, boundary. **Gap:** no `queue_size` overflow test.
- **`test_hsv_live_tuning.py`** (115 LOC) — V-channel gradient + 5 mutation tests proving "param set takes effect next frame".

**Coverage matrix:**

| Aspect | Coverage |
|---|---|
| Latched LabelInfo QoS | YES |
| Mask+cloud HxW invariant (resize) | NO |
| Stamp/frame propagation onto outputs | NO |
| Pipeline ABC contract | YES |
| Stub behavior | YES |
| HSV synthetic classification | YES (11 cases) |
| HSV live tuning | YES (5 cases) |
| HSV threshold drift | YES (hash) |
| ApproximateTime slop | YES (6 cases) |
| Class-map loader edge cases | YES |
| Hue wraparound | NO |
| Pipeline crash recovery | NO |
| NaN-cloud handling | NO |

Per-component tests are strong; the integration glue inside `_on_synced` (HxW resize, header propagation, exception path) is the high-leverage gap.

### package.xml / setup.py

`package.xml` — format=3, `ament_python`, MIT, deps via `<depend>` for rclpy/sensor_msgs/vision_msgs/std_msgs/cv_bridge/message_filters/numpy/opencv/yaml; test_depends for copyright/flake8/pep257/pytest. **P2:** missing `launch_pytest` test_dep; missing explicit `ament_index_python` exec_dep (transitively present).

`setup.py` — installs resource marker + package.xml + `config/*.yaml` + `launch/*.launch.py` glob. `entry_points` resolves `perception_node = avros_perception.perception_node:main`. `version='0.0.0'` never bumped (P2).

`setup.cfg` — `script_dir=$base/lib/avros_perception` ✓. `resource/avros_perception` marker present ✓.

## kiwicampus contract compliance

Walking the standards §9/§11 checklist:

| # | Invariant | Status |
|---|---|---|
| 1 | LabelInfo TRANSIENT_LOCAL+RELIABLE | YES (perception_node.py:202-209) |
| 2 | LabelInfo at same namespace as mask | YES (`/perception/<cam>/label_info`) |
| 3 | uint16 IDs, reserve top value | YES (`class_map.yaml` reserves 255) |
| 4 | Mask/cloud/conf share bit-identical stamp | YES (line 301 → 305, 311, 327) |
| 5 | Same frame_id (optical) on outputs | PARTIAL — mask/cloud/conf YES; **LabelInfo NO** (P0) |
| 6 | Cloud organized; HxW = mask | YES (resize at 287-291) |
| 7 | Mask is mono8 | YES (line 304) |
| 8 | NaN-aware cloud | NOT CHECKED (relay only) |
| 9 | slop ≤ 1/rate; queue sized | YES (slop=20 ms, queue=10 at 15 Hz) |
| 10 | allow_headerless=False | YES (default) |
| 11 | Monotonic stamps | EXTERNAL (ZED wrapper) |
| 12 | image_transport over raw | NO — raw Image, but perception+ZED same host so acceptable |
| 13 | cv_bridge explicit encoding | YES (`bgr8`, `mono8`) |
| 14-15 | URDF / ZED launch | EXTERNAL (avros_bringup) |
| 16 | ZED v5 topic names | YES (rgb/color/rect/image, point_cloud/cloud_registered) |
| 17 | Image resized BEFORE segmentation | YES (287-291 before 294) |
| 18 | Heavy inference offloaded | YES today; ONNX needs MultiThreadedExecutor |
| 19 | qos_profile_sensor_data on sensor subs | YES |
| 20-22 | kiwicampus subscriber side | EXTERNAL |
| 23 | Synthetic + real + sweep tests | PARTIAL (synthetic only; real fixtures TODO) |
| 24 | CameraInfo consumed | NO (not needed; cloud pre-projected by ZED) |
| 25 | Class IDs by name from LabelInfo | NO — three hardcoded sources (anti-pattern §10) |

The fundamental contract — stamp + frame_id + mono8 + organized HxW + latched LabelInfo — is correctly implemented; the known kiwicampus silent failures (volatile QoS, HxW mismatch, stamp drift) are all guarded. The two gaps are LabelInfo's empty frame_id and hardcoded numeric IDs.

## Cross-cutting issues

1. **Three sources of truth for class IDs** — `class_map.yaml`, `perception.yaml` (`class_id_lane` etc.), `hsv.py:31` (`_DEFAULT_CLASS_IDS`). Standards §10 anti-pattern. Fix: HSVPipeline resolves IDs by name from `LabelInfo`.
2. **`_on_synced` not exception-safe.** Only cv_bridge is wrapped. Pipeline crashes, cloud-relay errors, or output encoding failures propagate through message_filters and may halt callbacks. Wrap the entire callback.
3. **Pipeline hot-swap documented but unimplemented.** Read once at startup. Either implement rebuild on `_on_set_params` or document.
4. **Confidence semantics inconsistent** (stub all-255, HSV zero-on-background). Document the convention in `pipelines/base.py` and align.
5. **Param allowlist hides typos** — line 243 `continue` silently skips unknown names; user gets `successful: True`.
6. **No `log_level` launch arg** for field debugging.
7. **No frame_id sanity check.** If ZED is misconfigured to publish in body frame instead of `_optical` (REP 103 violation), perception silently emits wrong-frame data. Add `assert frame.endswith('_optical')`.
8. **`pipeline.warmup()` blocks node startup.** Acceptable for HSV (no-op); ONNX (~200 ms) should defer to a timer-zero one-shot so subscribers come up first.

## Test coverage analysis

~1018 LOC test : ~847 LOC source — the only package with a >1.0 test:source ratio. Per-component coverage is strong; integration-level coverage of `_on_synced` glue is the gap.

**Well covered:** Pipeline ABC contract (74 LOC, 7 tests), stub behavior (108 LOC, 9 tests including shared-params mutation), HSV synthetic classification (185 LOC, 11 tests with isolation via `adaptive_k=-100` baseline), HSV live tuning (115 LOC, 5 mutation tests), HSV threshold-drift SHA256 hash guard, class-map YAML edge cases, latched LabelInfo QoS via `launch_pytest`, ApproximateTime slop boundaries (6 cases).

**Gaps to close before competition:**

- `_on_synced` header propagation (no test confirms stamp/frame_id flows onto outputs)
- HxW resize path at lines 287-291 (no test triggers it)
- Pipeline crash recovery
- Hue wraparound (the P0 — synthetic test on `H ∈ [170, 5]` band missing)
- Sync queue overflow (15 images at 1 cloud)
- LabelInfo frame_id propagation through node (only `build_label_info` direct call tested)
- `_on_set_params` validation paths (length-3, range, low≤high) not directly tested
- Real-image regression corpus (`test/data/`, `test/fixtures/` empty)

`conftest.py` pins OpenCV threads and numpy seed for determinism. The `load_image` skip-on-missing pattern correctly defers fixture-dependent tests until the corpus lands.

This is the workspace's highest-quality test suite — the gaps are integration-level and the existing infrastructure (launch_pytest, hash guard, mock synchronizer) makes them straightforward to fill.

## Punch list

### P0

1. **LabelInfo `header.frame_id` empty.** `perception_node.py:212` calls `build_label_info(self._classes)` with default `frame_id=''`. Defer first publication until first synced frame so `image.header.frame_id` is known, or add a `label_info_frame_id` param.
2. **HSV barrel hue wraparound unhandled.** Single-range `cv2.inRange` on `H ∈ [5, 25]` (`hsv.py:103-104`) misses pixels when sunset shifts orange toward red across `H=0/179`. Add second-range pair (`barrel_low_2`, `barrel_high_2`) and OR.

### P1

3. **`_on_synced` not exception-safe** (only cv_bridge wrapped). Wrap entire callback `perception_node.py:274-328`.
4. **Pipeline hot-swap unimplemented.** `pipeline` param read once at line 145; either implement or document.
5. **Adaptive V-floor `k=0` doesn't disable bright AND** (`hsv.py:120-127`). Comment promises behavior the code doesn't deliver.
6. **Adaptive V-floor lag** at `adaptive_period > 1` (~333 ms at default 5). Drop to 1.
7. **Numeric class IDs duplicated** in 3 places. Resolve by name from `LabelInfo`.
8. **`launch_pytest` missing from `<test_depend>`.**
9. **`pothole` skips adaptive-V gate** despite being a stricter-lane class. Either consolidate or document.
10. **`sky_roi_poly` not validated in `_on_set_params`.** Bad poly crashes pipeline.
11. **Lane defaults disagree YAML vs in-code** (perception_node.py:123-124 vs perception.yaml:53-54).
12. **No `MultiThreadedExecutor` scaffold for ONNX.** HSV today is fast; ONNX inference will block.
13. **`camera_name` launch arg declared but unused** (perception.launch.py:53-55).
14. **Integration test doesn't publish synced image+cloud** — only verifies latched LabelInfo. Big coverage gap to close.
15. **No HxW-resize regression test** for `perception_node.py:287-291`.
16. **HSV bound declarations lack explicit `INTEGER_ARRAY` descriptor** — YAML override with one float flips type inference.
17. **`cv_bridge` error doesn't log `image.encoding`** for triage (line 279).

### P2

18. `Pipeline` ABC not declared via `abc.ABC`/`@abstractmethod`.
19. LabelInfo not republished on class-map change (TODO acceptable for IGVC).
20. `_DEFAULT_CLASS_IDS` const duplicates YAML.
21. Overlay publisher allocates per-class color array per frame.
22. `lane_white` RGB is cyan `[0, 255, 255]`.
23. Empty `test/data/`, `test/fixtures/` reserved for field corpus.
24. Stub vs HSV confidence semantics differ — document in `base.py`.
25. `version='0.0.0'` never bumped.
26. Missing `<exec_depend>ament_index_python</exec_depend>`.
27. No `log_level` launch arg.
28. No frame-id sanity check (`endswith('_optical')`).
29. `build_label_info` doesn't validate IDs against uint16.
30. `_on_set_params` silently accepts unknown param names.
31. No queue overflow test for `ApproximateTimeSynchronizer`.
32. Morph kernel size 3×3 hardcoded; not a tunable.

## Positives

- **Kiwicampus contract largely correct.** Latched LabelInfo (TRANSIENT_LOCAL+RELIABLE), bit-identical stamp/frame_id propagation, mono8 mask, organized HxW-matched cloud, resize before pipeline. Every silent failure mode the standards doc warns about is guarded.
- **Live tunability is first-class.** Shared mutable params dict + atomic validate-then-mutate `_on_set_params` pattern. Standards-aligned.
- **HxW resize before segmentation** uses `INTER_AREA` (correct for downscaling) and runs before `pipeline.run()`. The canonical ZED `pub_downscale_factor` vs `point_cloud_res` fix.
- **ZED v5 topic names** baked in with inline comment naming the v4 deprecation.
- **Pipeline abstraction minimal and correct.** ONNX slots in without changing the node.
- **Test quality is the workspace high-water mark.** Hash-based threshold-drift guard (`test_hsv_thresholds.py`) is governance most teams skip. Mock `ApproximateTimeSynchronizer` tests, late-joining `launch_pytest` integration test, live-tuning end-to-end.
- **`/**:` wildcard YAML + `OpaqueFunction` multi-camera launch** matches standards.
- **Inline calibration history in `perception.yaml`** (with dates and reasons) makes regression triage easy.
- **Docstrings accurate.** perception_node and HSV docstrings cite prior-art teams (Sooner Robotics, iscumd) and precisely describe the kiwicampus contract.
- **Defensive post-pipeline shape check** + `try_shutdown()` over `shutdown()`.

Overall: better shape than most of the workspace. The two P0s are ~2 hours each. P1s are robustness hardening for known fragile paths. Test suite is strong enough that fixes won't regress unnoticed. Ready for IGVC AutoNav competition after the LabelInfo frame_id and barrel-hue-wraparound fixes.
