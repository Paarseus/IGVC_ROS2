# Perception Phase 4 + 5 — plan with tests at every step

Written 2026-04-24 after 6-agent research session. Sources at the bottom.

## Context

Semantic segmentation Phase 0–3 is shipped and Jetson-verified (commits `3ac4214..cdec3c9`).
Current state:

- `avros_perception` package with `StubPipeline` producing mono8 mask + organized PC2 + LabelInfo
- kiwicampus `semantic_segmentation_layer` plugin wired into `local_costmap` as `semantic_front`
- URDF + sensors.launch + perception.launch scaffolded for 3 ZED X (front/left/right) via `zed_macro.urdf.xacro`
- nav2_params only has the `semantic_front` instance — left/right plugin blocks missing
- `enable_zed_left` / `enable_zed_right` args don't plumb through `localization.launch.py` + `navigation.launch.py`
- No test suite yet beyond ament linter stubs on `avros_control`

This plan covers the work from "multi-cam wiring complete" through "HSV pipeline in production" through "3-camera deploy." Every step ships code *with* tests, not code then tests.

## Scope

| Step | What | Hardware needed | Test artifact |
|---|---|---|---|
| **A** | Finish multi-cam kiwicampus wiring + launch plumbing | None (file edits) | YAML parse + launch-test for 3-node spawn |
| **B** | Test infrastructure scaffold + CI | None | Linter stubs + conftest + GHA workflow |
| **C** | Stub pipeline regression + node-level tests | None | 4 pytest files, all pass on dev laptop |
| **D** | **Phase 4** — HSVPipeline implementation | None for code; Jetson + ZED for smoke test | 6 pytest files covering thresholds, regression, illumination, negatives, perf |
| **E** | Nav2 integration tests (end-to-end chain) | None | 3 tests: in-process costmap, launch_test, sync-slop |
| **F** | **Phase 4 field calibration** | IGVC-grass + sunlight + cones | Updated fixtures + regenerated goldens |
| **G** | **Phase 5** — 3-camera deploy + perf gate | Right-camera replacement + Jetson | 3-cam launch test, GPU budget assertion |

Phase 6 (ONNX/TRT) is deliberately out of scope — post-competition.

## Test strategy — overall shape

Pyramid, widest at the bottom:

```
              ┌─ Hardware smoke (manual, Jetson) ─┐      ← G only
              │          E: integration           │
              │   test_perception_costmap_launch  │
              │   test_sync_slop                  │
              │   test_perception_to_layer_inproc │
              └───────────────────────────────────┘
           ┌─ Launch tests (launch_pytest, run in CI) ───┐
           │   test_perception_launch                    │
           │   test_multi_camera_launch                  │
           └─────────────────────────────────────────────┘
     ┌─ Node tests (rclpy in-process via ros2-easy-test) ──┐
     │   test_node_io                                      │
     │   test_pipeline_switch                              │
     └─────────────────────────────────────────────────────┘
┌─ Unit tests (pure Python, no ROS) — the bulk of coverage ──┐
│   test_hsv_thresholds       test_mask_regression           │
│   test_illumination_invariance  test_negative_inputs       │
│   test_pipeline_interface   test_label_info                │
│   test_stub_pipeline        test_synthetic                 │
└────────────────────────────────────────────────────────────┘
```

Unit tests run in < 5 seconds. Launch tests run in ~10 s each. Integration tests run ~30–60 s. Hardware tests are manual, gated behind `pytest.mark.hardware`, run only on Jetson.

Determinism knobs (in `conftest.py`):
- `cv2.setNumThreads(1)` — OpenCV threaded inner loops are nondeterministic
- `np.random.seed(0)` — for any test using synthetic noise
- Force CPU for ONNX paths in CI — `pytest.mark.hardware` tags the GPU tests

---

## Step A — finish multi-cam kiwicampus wiring

**Code changes** (~45 lines total):

1. `src/avros_bringup/config/nav2_params.yaml` + `nav2_params_humble.yaml`
   - Under `local_costmap.local_costmap.ros__parameters.plugins`: `[voxel_layer, semantic_front, semantic_left, semantic_right, inflation_layer]`
   - Add `semantic_left` + `semantic_right` blocks mirroring `semantic_front`, but subscribed to `/perception/left/*` and `/perception/right/*`

2. `src/avros_bringup/launch/localization.launch.py`
   - Declare `enable_zed_left` + `enable_zed_right` args
   - Pass through to `sensors.launch.py`

3. `src/avros_bringup/launch/navigation.launch.py`
   - Declare the same two args
   - Pass through to `localization.launch.py`
   - Pass `cameras:='front,left,right'` into `perception.launch.py` when ≥2 cameras enabled (via an `OpaqueFunction` that reads the three `enable_zed_*` configs and assembles the camera list)

**Tests alongside (new — land in this step):**

- `src/avros_bringup/test/test_config_parse.py` — assert all four YAMLs (`nav2_params*.yaml`, `perception.yaml`, `class_map.yaml`) `yaml.safe_load` cleanly and the `local_costmap` plugins list has exactly one `semantic_*` entry per kiwicampus source topic (symmetric — no orphans)
- `src/avros_perception/test/launch/test_multi_camera_launch.py` — `launch_pytest` that spawns `perception.launch.py` with `cameras:='front,left,right'`, waits for all three `/perception/{cam}/label_info` topics to appear (latched), asserts three separate node names exist (`perception_front`, `perception_left`, `perception_right`)

**Exit criteria:**
- Both YAML files re-read and validated by `test_config_parse.py` without drift
- Launch test passes with 3 perception nodes visible in `ros2 node list`
- On Jetson: `ros2 launch avros_bringup navigation.launch.py enable_zed_front:=true enable_zed_left:=true enable_zed_right:=true enable_perception:=true` produces 3 `/perception/{cam}/semantic_mask` topics at 15 Hz

---

## Step B — test infrastructure scaffold

**Directory layout** (creates the whole skeleton):

```
src/avros_perception/test/
├── __init__.py
├── conftest.py                              # pytest fixtures (rclpy init, cv2 seed, np seed, load_image)
├── test_copyright.py                        # ament stub
├── test_flake8.py                           # ament stub
├── test_pep257.py                           # ament stub
├── unit/                                    # pure Python, no ROS
│   ├── __init__.py
│   ├── test_pipeline_interface.py
│   ├── test_stub_pipeline.py
│   ├── test_label_info.py
│   └── test_image_utils.py
├── node/                                    # rclpy in-process
│   ├── __init__.py
│   └── test_node_io.py
├── launch/                                  # launch_pytest
│   ├── __init__.py
│   └── test_perception_launch.py
├── fixtures/                                # committed to git, ~3 MB total
│   ├── .gitkeep                             # populated in step D
└── data/
    └── .gitkeep                             # goldens populated in step D
```

**package.xml additions:**
```xml
<test_depend>ament_copyright</test_depend>
<test_depend>ament_flake8</test_depend>
<test_depend>ament_pep257</test_depend>
<test_depend>python3-pytest</test_depend>
<test_depend>launch_pytest</test_depend>
<test_depend>launch_testing_ros</test_depend>
<test_depend>ros2bag</test_depend>
```

**setup.py additions:**
```python
extras_require={'test': [
    'pytest',
    'pytest-regressions',     # golden-output diffing
    'pytest-benchmark',       # perf tests
    'numpy<2',
    'opencv-python-headless',
    'imagehash',              # perceptual hash fallback
]},
```

**`conftest.py`** sets determinism + shared fixtures:
```python
import cv2, numpy as np, pytest, os, rclpy

def pytest_configure(config):
    cv2.setNumThreads(1)
    np.random.seed(0)

@pytest.fixture(scope='session')
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def fixtures_dir():
    return os.path.join(os.path.dirname(__file__), 'fixtures')

@pytest.fixture
def load_image(fixtures_dir):
    def _load(name):
        path = os.path.join(fixtures_dir, name)
        img = cv2.imread(path)
        if img is None:
            pytest.skip(f"fixture {name} not yet collected")
        return img
    return _load
```

**CI — `.github/workflows/test.yml`:**
```yaml
jobs:
  test-humble:
    runs-on: ubuntu-22.04
    container: ros:humble
    steps:
      - uses: actions/checkout@v4
      - uses: ros-tooling/setup-ros@v0.7
        with: { required-ros-distributions: humble }
      - uses: ros-tooling/action-ros-ci@v0.3
        with:
          target-ros2-distro: humble
          package-name: avros_msgs avros_perception avros_bringup
          colcon-mixin-name: coverage-pytest
      - uses: codecov/codecov-action@v4
        with: { files: ros_ws/lcov/total_coverage.info }
```

**Tests alongside (this step IS the test infrastructure):**

- `test_copyright.py`, `test_flake8.py`, `test_pep257.py` — ament linter stubs (one-liner each)
- `test_pipeline_interface.py` — import `Pipeline`, instantiate a subclass, assert `run(numpy.zeros((100, 100, 3), dtype=uint8))` returns `PipelineResult` with correctly-shaped `mask` and `confidence`
- `test_stub_pipeline.py` — inject `StubPipeline` with `inject_stripe_width=50`, assert mask has exactly `50 * H` non-zero pixels with class ID 1

**Exit criteria:**
- `cd src/avros_perception && colcon test --packages-select avros_perception` passes 5 tests
- CI workflow green on a scratch commit

---

## Step C — stub pipeline regression + node-level tests

Before writing HSV, prove the plumbing works with the stub. Tests written in this step become regression baselines as Phase 4 lands.

**Tests:**

1. `test/unit/test_label_info.py` — load `class_map.yaml`, build a `vision_msgs/LabelInfo`, assert round-trip (ids stable, names stable, count matches YAML entries)
2. `test/unit/test_image_utils.py` — test the image-resize-to-cloud-shape logic in isolation (feed `bgr` at 540×960, cloud shape (256, 448), assert resized BGR shape matches)
3. `test/node/test_node_io.py` — using `ros2-easy-test` `@with_single_node(PerceptionNode, watch_topics={...})`: publish synthetic Image + PointCloud2 with identical stamp, assert mask + cloud + confidence published with that same stamp, LabelInfo latched
4. `test/launch/test_perception_launch.py` — `launch_pytest` spawns `perception_node` with `pipeline:=stub`, asserts all 4 output topics exist within 5 s

**Exit criteria:**
- 4 new tests pass
- `colcon test` reports 9 tests total (5 from B + 4 from C)
- CI still green

---

## Step D — Phase 4: HSVPipeline implementation

**Code changes** (~250 LOC):

1. `src/avros_perception/avros_perception/pipelines/hsv.py` — new file, implements the design from the 6-agent research:
   ```python
   class HSVPipeline(Pipeline):
       def run(self, bgr, depth=None):
           # Sooner 2023 preprocessing — 3× 5×5 box blur
           for _ in range(self.blur_iters):
               bgr = cv2.blur(bgr, (5, 5))
           hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

           # iscumd adaptive V-threshold — recompute every N frames
           v = hsv[:, :, 2]
           self._tick = (self._tick + 1) % self.adaptive_period
           if self._tick == 0 or self._v_floor is None:
               self._v_floor = float(v.mean() + self.adaptive_k * v.std())
           bright = (v >= self._v_floor).astype(np.uint8) * 255

           # Per-class fixed HSV (Sooner 2024)
           lane = cv2.inRange(hsv, self.lane_low, self.lane_high)
           lane = cv2.bitwise_and(lane, bright)                   # AND the two gates
           barrel = cv2.inRange(hsv, self.orange_low, self.orange_high)
           pothole = cv2.inRange(hsv, self.pothole_low, self.pothole_high)

           # Morphology
           k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
           lane = cv2.erode(lane, k)
           barrel = cv2.morphologyEx(barrel, cv2.MORPH_OPEN, k)
           pothole = cv2.morphologyEx(pothole, cv2.MORPH_OPEN, k)

           # ROI polygon — cut out the sky region
           mask = np.zeros(v.shape, dtype=np.uint8)
           mask[pothole > 0] = CLASS_POTHOLE
           mask[barrel > 0] = CLASS_BARREL
           mask[lane > 0] = CLASS_LANE   # lane wins priority for overlaps
           if self.sky_roi is not None:
               cv2.fillPoly(mask, [self.sky_roi], 0)

           confidence = np.where(mask > 0, 255, 0).astype(np.uint8)
           return PipelineResult(mask=mask, confidence=confidence)
   ```
2. `src/avros_perception/avros_perception/pipelines/__init__.py` — register `'hsv': HSVPipeline` in `PIPELINES`
3. `src/avros_perception/config/class_map.yaml` — expand to 5 classes (free / lane_white / barrel_orange / pothole / unknown)
4. `src/avros_perception/config/perception.yaml` — add HSV ranges + ROI polygon + adaptive params, all per-camera
5. `src/avros_bringup/config/nav2_params*.yaml` — update each `semantic_*` block's `class_types` to `[traversable, danger]` with the new class names

**Fixtures** (~20 images committed to `test/fixtures/`, ~3 MB total — git, not LFS):

```
test/fixtures/
├── lanes/
│   ├── lane_sun_01.jpg         ← clear painted line, bright sun
│   ├── lane_shadow_01.jpg      ← painted line under tree shadow
│   ├── lane_wet_01.jpg         ← painted line with water sheen
│   ├── lane_dashed_01.jpg      ← dashed line segment
├── obstacles/
│   ├── orange_barrel_01.jpg
│   ├── orange_barrel_shadow_01.jpg
│   ├── white_tarp_01.jpg       ← false-positive bait
│   ├── sawhorse_01.jpg
├── potholes/
│   ├── pothole_ring_01.jpg     ← white-paint ring on grass
├── negatives/
│   ├── grass_only.jpg          ← should be empty mask
│   ├── sky_only.jpg            ← should be empty mask
│   ├── asphalt_only.jpg        ← should be empty mask
```

Collect these from any preliminary ZED footage you have, or from IGVC tech report photos, or from Google Images tagged with course conditions. Before field day, these are placeholders; after field day, they become the calibration corpus.

**Tests alongside (6 new files):**

1. **`test/unit/test_hsv_thresholds.py`** — load `perception.yaml`, hash the `hsv:` subtree SHA256, assert against a frozen hash constant. Any PR that changes HSV values must also update the frozen hash → explicit review signal.
2. **`test/unit/test_mask_regression.py`** — use `pytest-regressions` `ndarrays_regression`:
   ```python
   def test_lane_sun_01_mask_stable(hsv_pipeline, load_image, ndarrays_regression):
       img = load_image('lanes/lane_sun_01.jpg')
       result = hsv_pipeline.run(img)
       ndarrays_regression.check({'mask': result.mask}, default_tolerance={'atol': 2})
   ```
   First run writes `test/data/test_lane_sun_01_mask_stable.npz`; subsequent runs diff.
3. **`test/unit/test_illumination_invariance.py`** — highest-ROI outdoor test:
   ```python
   @pytest.mark.parametrize("gamma", [0.5, 0.75, 1.25, 1.5, 2.0])
   def test_lane_survives_gamma(hsv_pipeline, load_image, gamma):
       base = load_image('lanes/lane_sun_01.jpg')
       perturbed = np.power(base / 255.0, gamma) * 255
       m_base = hsv_pipeline.run(base).mask
       m_pert = hsv_pipeline.run(perturbed.astype(np.uint8)).mask
       iou = compute_iou(m_base == CLASS_LANE, m_pert == CLASS_LANE)
       assert iou >= 0.85, f"gamma={gamma} dropped IoU to {iou:.2f}"
   ```
4. **`test/unit/test_negative_inputs.py`** — grass/sky/asphalt → FP ratio < 1%:
   ```python
   @pytest.mark.parametrize("fixture", ["negatives/grass_only.jpg", "negatives/sky_only.jpg"])
   def test_negative_empty(hsv_pipeline, load_image, fixture):
       mask = hsv_pipeline.run(load_image(fixture)).mask
       fp_ratio = np.count_nonzero(mask) / mask.size
       assert fp_ratio < 0.01, f"{fixture}: {fp_ratio*100:.2f}% false-positive"
   ```
5. **`test/unit/test_synthetic.py`** — albumentations-based perturbation suite (JPEG quality, motion blur, sun flare, shadow) applied to baseline, each pipeline output asserted IoU ≥ 0.80
6. **`test/unit/test_perf.py`** — `pytest-benchmark`, gated `@pytest.mark.hardware`:
   ```python
   @pytest.mark.hardware
   def test_hsv_perf(benchmark, hsv_pipeline, load_image):
       img = load_image('lanes/lane_sun_01.jpg')
       result = benchmark(hsv_pipeline.run, img)
       assert benchmark.stats['mean'] < 0.033  # ≥ 30 Hz target on Jetson
   ```

**Exit criteria:**
- 6 new tests alongside the new code; all unit tests pass in CI on dev laptop
- HSV pipeline runs end-to-end on Jetson with the front camera (`pipeline: hsv` in perception.yaml)
- Stripe injection test from Phase 3 still works (backward compat) by re-setting `pipeline: stub`
- `test_perf.py` passes on Jetson; skipped in CI

---

## Step E — Nav2 integration tests (end-to-end chain)

These are the tests the user specifically called out as "silent failures we'd never catch otherwise": QoS mismatches, TF timing, message-filter sync drops, stamp drift.

**Tests (3 new files):**

1. **`test/unit/test_sync_slop.py`** — mirror of `ros2/message_filters test_approxsync.py`. Mock `message_filters.Subscriber`s, push Image + PointCloud2 with stamps offset by `slop ± ε`, assert the sync callback fires for within-slop and drops for beyond-slop. Catches our `sync_slop: 0.02` being too tight under real cloud-vs-image jitter.

2. **`test/launch/test_perception_costmap_launch.py`** — `launch_pytest` that composes:
   - `perception_node` with `pipeline: stub` + `inject_stripe_width: 100`
   - `nav2_costmap_2d` standalone node loading a minimal `costmap_params.yaml` with the kiwicampus `semantic_front` plugin only
   - `static_transform_publisher` for `odom→base_link→zed_front_camera_link`
   - Synthetic Image + PointCloud2 publisher (within the test) at 5 Hz with matched stamps
   
   Then subscribes `/local_costmap/costmap`, waits up to 10 s, asserts ≥ 500 cells with cost > 80 (OccupancyGrid scale). This single test catches *every* real-world failure mode: QoS mismatch, TF timing, pluginlib load, plugin class-name typo, label lookup, frame_id mismatch, stamp drift.

3. **`test/integration/test_perception_to_layer_inprocess.cpp`** (only gtest — C++) — mirror Nav2's `inflation_tests.cpp` pattern. Construct `LayeredCostmap` in-process, add `SemanticSegmentationLayer` directly, call `addStaticObservation()` with a synthetic mask + organized cloud + LabelInfo, run `updateMap(0,0,0)`, assert `getCost(mx,my) == LETHAL_OBSTACLE`. This catches kiwicampus-internal bugs with no DDS involved — fast (<1 s), deterministic. Requires adding a `test/` gtest target in avros_perception's CMakeLists-alike, which means adding a C++ test subdirectory to our ament_python package via `ament_cmake_test` — doable but slightly unusual.

**Exit criteria:**
- All 3 tests pass locally and in CI (the C++ one needs Nav2 + kiwicampus headers; CI container must include them)
- Deliberately breaking the sync slop in a scratch commit causes `test_sync_slop.py` to fail
- Deliberately writing wrong class name in `class_types` causes `test_perception_costmap_launch.py` to fail

---

## Step F — Phase 4 field calibration

Must be done outdoors on IGVC-representative grass + lighting. Not a code task; a tuning task.

**Procedure:**

1. Drive the robot around the field with `ros2 bag record /zed_front/zed_node/rgb/color/rect/image` — get 5 minutes of variety (sun/shadow/barrels-in-frame/lanes-at-distance)
2. Open [`ctu-mrs/color_picker`](https://github.com/ctu-mrs/color_picker) RQT plugin subscribed to the bag playback
3. Tune each class's HSV bounds by clicking on pixels: white paint, orange barrels, pothole-white-ring, grass (to exclude), sky (to exclude)
4. Export bounds, paste into `perception.yaml`
5. Re-run the pipeline against all 20 fixtures in `test/fixtures/`; visually inspect the new masks
6. `pytest test/unit/ --force-regen` regenerates all `ndarrays_regression` goldens against the new thresholds
7. Update the SHA in `test_hsv_thresholds.py` (the immutability hash)
8. Git commit: "Phase 4 field calibration — YYYY-MM-DD"
9. Re-run the full test suite — should all pass against the new baseline

**Exit criteria:**
- 5+ minutes of field bag recorded and archived (off-repo)
- 20 fixtures in `test/fixtures/` replaced or augmented with real field frames
- All unit tests green against the new thresholds
- Running `pipeline: hsv` on live ZED in the field produces masks a human judges as "mostly right" before we care about integration scores

---

## Step G — Phase 5: 3-camera deploy + perf gate

Depends on right-camera replacement hardware arriving.

**Code changes** (mostly already done from your manual edits — only gap):

- Confirm right-camera serial `42569280` in `sensors.launch.py:179` is the real unit (ask before flipping `enable_zed_right:=true`)
- Any per-camera HSV offsets — left/right cameras see the scene at different angles and may need slightly different ROI polygons. Structure `perception.yaml` so each camera has its own `cameras.{front,left,right}.{hsv,roi}` block; `perception_node` reads by its `camera_name` param

**Tests alongside:**

1. **`test/launch/test_multi_camera_launch.py`** (already drafted in step A, expanded here) — bring up 3 cameras + 3 perception_nodes + costmap, wait for all 3 `semantic_*` plugin instances to log "Initialized plugin", assert at least one cell is marked in `/local_costmap/costmap` via a stripe injection on any of the three cameras
2. **`test/unit/test_perf.py`** — add a Jetson-gated multi-camera perf test:
   ```python
   @pytest.mark.hardware
   def test_three_camera_perf(hsv_pipeline, load_image):
       """3 instances of HSV at 15 Hz each = 45 frames/sec sustainable"""
       img = load_image('lanes/lane_sun_01.jpg')
       start = time.perf_counter()
       for _ in range(150):   # 5s × 30Hz × 1 instance = budget
           _ = hsv_pipeline.run(img)
       elapsed = time.perf_counter() - start
       assert elapsed < 5.0, f"HSV pipeline too slow: 150 frames in {elapsed:.1f}s"
   ```

**Exit criteria:**
- 3 cameras running simultaneously; Jetson CPU < 80%, GPU < 70%
- All 3 semantic plugin instances writing into `/local_costmap/costmap`
- Nav2 can plan a route through a course with injected costs from any of the 3 cameras
- `test_three_camera_perf` passes on Jetson

---

## Dependencies we're adding

| Package | Why | Where |
|---|---|---|
| `pytest-regressions` | golden `.npz` diffing for mask outputs | `setup.py` extras_require['test'] |
| `pytest-benchmark` | perf assertions with statistical spread | same |
| `imagehash` | perceptual-hash fallback for minor numpy drift | same |
| `albumentations` | synthetic perturbations (gamma, JPEG, shadow) | same |
| `opencv-python-headless` | OpenCV for unit tests in CI (no display) | same |
| `ros2-easy-test` | clean rclpy fixture for node tests | pip install; add to CI workflow |
| `launch_pytest` | launch tests as pytest markers | package.xml test_depend |
| `launch_testing_ros` | launch_pytest ROS-specific helpers | same |

## Timeline estimate

Rough effort per step, assuming solo developer with AVROS + ROS2 familiarity:

| Step | Effort | Blocker |
|---|---|---|
| A | 2 h | none |
| B | 2 h | none |
| C | 3 h | none |
| D (code + fixtures) | 4 h | fixtures can be Google-Images until field day |
| D (tests, per file) | ~30 min each × 6 = 3 h | — |
| E | 4 h | C++ gtest setup is the trickiest bit |
| F | 1 day (field) | IGVC-grass access, sunshine |
| G | 2 h code + half-day tune | right-camera hardware |

Steps A–E are code-only and can run in parallel with competition logistics. Step F is the real pacing item — it's outdoors with the robot.

## Sources

- **ROS2 testing:** [Humble Python Testing](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html), [launch_pytest (humble)](https://github.com/ros2/launch/tree/humble/launch_pytest), [ros2/rosbag2 test_sequential_reader.py](https://github.com/ros2/rosbag2/blob/rolling/rosbag2_py/test/test_sequential_reader.py), [felixdivo/ros2-easy-test](https://github.com/felixdivo/ros2-easy-test), [abaeyens/ros2-integration-testing-examples](https://github.com/abaeyens/ros2-integration-testing-examples), [ros-tooling/action-ros-ci](https://github.com/ros-tooling/action-ros-ci), [pytest-regressions docs](https://pytest-regressions.readthedocs.io/)
- **CV testing:** [ros-perception/vision_opencv cv_bridge tests](https://github.com/ros-perception/vision_opencv/blob/rolling/cv_bridge/test/python_bindings.py), [NVIDIA-ISAAC-ROS/ros2_benchmark](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark), [ctu-mrs/color_picker](https://github.com/ctu-mrs/color_picker), [albumentations writing_tests](https://albumentations.readthedocs.io/en/latest/writing_tests.html), [pytest-benchmark](https://pypi.org/project/pytest-benchmark/)
- **Nav2 integration:** [nav2_costmap_2d/test/integration/inflation_tests.cpp](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/test/integration/inflation_tests.cpp), [nav2_costmap_2d/test/integration/range_tests.cpp](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/test/integration/range_tests.cpp), [ros2/message_filters test_approxsync.py](https://github.com/ros2/message_filters/blob/master/test/test_approxsync.py), [Humble Integration Testing docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Integration.html), [Autoware integration testing guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/testing-guidelines/integration-testing/)
- **HSV algorithm research** (pre-plan): Sooner Robotics [2023](https://github.com/SoonerRobotics/autonav_software_2023) / [2024](https://github.com/SoonerRobotics/autonav_software_2024) / [2025](https://github.com/SoonerRobotics/autonav_software_2025), [iscumd/white_line_detection](https://github.com/iscumd/white_line_detection), [kiwicampus/mono_color_segmentation](https://github.com/kiwicampus/mono_color_segmentation)
