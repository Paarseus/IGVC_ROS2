# ROS 2 Perception Pipeline Standards (Humble)

A reference for what professional / upstream-quality looks like for a semantic-segmentation perception node that feeds an organized point cloud + label mask + class palette to a Nav2 costmap layer. Each section cites primary sources inline. This document is forward-looking only; it intentionally does not look at the in-tree `avros_perception` code (that is Phase 2).

Distros assumed: ROS 2 Humble, message_filters 4.x, vision_msgs ROS 2 branch, image_common Humble branch, ZED ROS 2 wrapper v5.x.

---

## 1. vision_msgs Conventions

**`LabelInfo` is the analogue of `CameraInfo` for vision pipelines.** It carries the class palette so subscribers can convert numeric label IDs back to human-readable class names. ([ros-perception/vision_msgs LabelInfo.msg](https://github.com/ros-perception/vision_msgs/blob/ros2/vision_msgs/msg/LabelInfo.msg))

- Fields (Humble / `ros2` branch):
  - `std_msgs/Header header`
  - `vision_msgs/VisionClass[] class_map` — array of `(uint16 class_id, string class_name)` pairs. ID `65535` is reserved for the `UNLABELED` class when using `uint16`; ID `255` is reserved when the pipeline only uses `uint8` masks. ([VisionClass.msg](https://github.com/ros-perception/vision_msgs/blob/ros2/vision_msgs/msg/VisionClass.msg))
  - `float32 threshold` — confidence threshold in `[0, 1]` used for the inference (interpretation is pipeline-specific; downstream consumers use it to know what threshold the publisher already applied). ([LabelInfo.msg](https://github.com/ros-perception/vision_msgs/blob/ros2/vision_msgs/msg/LabelInfo.msg))
- **Topic placement:** publish at the same namespace level as the associated image, e.g. image at `/my_seg/image` → label info at `/my_seg/label_info`. This mirrors the `image_raw` ↔ `camera_info` pairing. ([vision_msgs README](https://github.com/ros-perception/vision_msgs/blob/ros2/README.md?plain=1))
- **QoS — latched is mandatory.** Use `transient_local` durability + `reliable` reliability so late-joining subscribers receive the palette. README quote: "a latched publisher … achieved using a `transient local` QoS profile". Both publisher and subscriber must request TRANSIENT_LOCAL or DDS will not deliver historical samples. ([vision_msgs README](https://github.com/ros-perception/vision_msgs/blob/ros2/README.md?plain=1), [ROS 2 QoS concepts](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html))
- **Recommended subscriber pattern:** subscribe transient-local, latch the first `LabelInfo`, then either keep the subscription or unsubscribe and cache. ([vision_msgs README](https://github.com/ros-perception/vision_msgs/blob/ros2/README.md?plain=1))
- **`header.frame_id`:** match the optical frame of the image (e.g. `<camera>_left_camera_frame_optical`). LabelInfo is metadata for that image stream, so its frame should agree with what `sensor_msgs/Image` carries. ([vision_msgs README](https://github.com/ros-perception/vision_msgs/blob/ros2/README.md?plain=1))
- **Relationship to other vision_msgs types:**
  - `Detection2D` / `Detection2DArray`, `Detection3D` / `Detection3DArray` carry per-instance hypotheses (`ObjectHypothesisWithPose`) — they reference class IDs, but the ID-to-name mapping comes from `LabelInfo` (or the older `VisionInfo`). ([ros-perception/vision_msgs](https://github.com/ros-perception/vision_msgs))
  - `Classification.msg` returns `ObjectHypothesis[] results` for whole-image / region classification — same ID convention.
  - `LabelInfo` is the authoritative palette source for **dense semantic segmentation** masks (mono8/mono16 images), where there is no per-instance hypothesis but every pixel has a class.
- A "class palette" in this context is exactly `class_map`: the ordered set of (id, name) pairs the pipeline emits in its mask. RGB color for visualization is **not** in `LabelInfo`; downstream tools (or a sidecar YAML) own colors.

---

## 2. Mask + Cloud Sync — The Triple Contract

For a label mask to project pixel-by-pixel onto a point cloud, three invariants must hold:

1. **Same `header.stamp`** — bit-identical timestamp on both messages. Downstream synchronizers (and the kiwicampus layer in particular) match by stamp; even microsecond-level drift breaks the match. ([ros2/message_filters ApproximateTime](https://docs.ros.org/en/ros2_packages/humble/api/message_filters/doc/index.html))
2. **Same `header.frame_id`** — both must be the camera's optical frame (`*_camera_frame_optical`), not the body frame. The `sensor_msgs/Image` doc explicitly says the image frame's `+x` points right, `+y` down, `+z` into the scene — that's the optical frame. ([sensor_msgs/msg/Image.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg))
3. **Same `(height, width)` = same HxW** — the cloud must be **organized** (`height > 1`) and shaped exactly like the mask, so cloud row `r`, column `c` is the 3D point for mask pixel `(r, c)`. ([sensor_msgs/msg/PointCloud2.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud2.msg))

What breaks if any one fails:

- **Stamp drift** → `message_filters::ApproximateTime` drops the pair (no callback fires) or, worse, pairs your mask with the previous frame's cloud (if `slop` is permissive). Costmap then marks obstacles in stale 3D positions. ([ApproximateTime tutorial](https://docs.ros.org/en/ros2_packages/humble/api/message_filters/doc/index.html))
- **Frame mismatch** → TF resolves both frames to `map` independently, so the geometry "works" but the lookups are at the wrong instants of `tf` or with the wrong intrinsics; subtle ~10 cm errors at distance.
- **HxW mismatch** → no safe per-pixel lookup is possible. Upstream usually resizes the mask to the cloud's shape (or the cloud to the mask's shape) before publishing. The kiwicampus layer fails silently (no marks deposited) when this is violated.

How professionals enforce it:

- **Single source of truth for the stamp.** Whichever message is taken as the "master" (typically the image), copy its `header` (or at minimum `stamp` + `frame_id`) onto the derived mask **and** the derived cloud — do not regenerate stamps from `node.now()`.
- **Resize before publish.** If the camera publishes a 1280×720 image but a 256×448 cloud (the ZED case below), resize the image to 256×448 before running segmentation, so mask and cloud emerge with identical HxW.
- **Use `image_pipeline`'s composable nodes when feasible.** `depth_image_proc/point_cloud_xyzrgb` and friends are guaranteed to emit organized clouds aligned with the input image, propagating headers correctly. ([ros-perception/image_pipeline](https://github.com/ros-perception/image_pipeline/tree/humble))

---

## 3. message_filters ApproximateTime

`ApproximateTimeSynchronizer` (Python) / `ApproximateTime<MsgA, MsgB, ...>` (C++) matches messages from N topics by `header.stamp` within a `slop` window. ([rclpy bindings](https://github.com/ros2/message_filters/blob/humble/src/message_filters/__init__.py))

- **Constructor (Python):** `ApproximateTimeSynchronizer(filters, queue_size, slop, allow_headerless=False)`. `slop` is in seconds. ([rclpy bindings](https://github.com/ros2/message_filters/blob/humble/src/message_filters/__init__.py))
- **Pivot algorithm:** once each per-topic queue has at least one message, the latest of the queue heads becomes the **pivot**; the algorithm then searches for the smallest set of messages (one per topic) bounded by the pivot, and publishes when it can prove no smaller set exists. ([ros wiki: message_filters/ApproximateTime](http://library.isr.ist.utl.pt/docs/roswiki/message_filters%282f%29ApproximateTime.html))
- **Slop selection rule of thumb:** `slop ≈ (1 / frame_rate) / 2` to `(1 / frame_rate)`. For 15 Hz streams (66 ms period), `slop = 0.05`–`0.066`. Slop larger than the frame period invites pairing across frames — old-mask + new-cloud bugs. ([ApproximateTime wiki](http://library.isr.ist.utl.pt/docs/roswiki/message_filters%282f%29ApproximateTime.html))
- **Same-stamp pitfall:** the algorithm assumes timestamps are **strictly monotonic per topic**. If two consecutive messages on the same topic share a stamp, or if two messages on different topics share a stamp the policy treats as ambiguous, behavior is undefined for those candidates. The fix is to ensure each producer stamps with monotonic, sub-millisecond-distinct timestamps (`node.get_clock().now()`, never a constant). ([ApproximateTime wiki](http://library.isr.ist.utl.pt/docs/roswiki/message_filters%282f%29ApproximateTime.html))
- **`queue_size` vs `slop` interaction:**
  - `queue_size` caps how many messages per topic are buffered before the oldest is dropped.
  - With multi-topic streams at 15 Hz, a `queue_size` smaller than `slop * rate` will drop messages before the synchronizer has a chance to pair them — a common cause of "callback never fires."
  - Practical default: `queue_size = 10`, `slop` in `[0.05, 0.1]` for 15 Hz; tune `slop` first, raise `queue_size` only if you see drops. ([rclpy bindings](https://github.com/ros2/message_filters/blob/humble/src/message_filters/__init__.py))
- **`allow_headerless=True` is a footgun.** It lets messages without `header.stamp` join the policy, but rclpy stamps them with "now," producing unpredictable ordering. The docstring itself cautions: "you should avoid this as much as you can, since the delays are unpredictable." Do not enable for sensor data. ([rclpy bindings](https://github.com/ros2/message_filters/blob/humble/src/message_filters/__init__.py))
- **TF buffer is not a substitute.** `tf2_ros::Buffer` interpolates transforms in time, but `ApproximateTime` matches messages, not transforms. Even if `tf` is perfect, pairing the wrong mask with the wrong cloud still corrupts the depth-segmentation join. Use both: `ApproximateTime` to align mask+cloud, `tf2` only to transform the result into `base_link` / `map`.

---

## 4. image_transport / cv_bridge

**Why image_transport over raw `Image` topics:** image_transport is the ROS 2 middleware layer that lets a node subscribe via different transports (raw, compressed, theora, …) without changing application code. Bandwidth-constrained links should always use `compressed` rather than raw `sensor_msgs/Image`. ([ros-perception/image_common](https://github.com/ros-perception/image_common))

- API (rclcpp): `image_transport::Subscriber sub = it.subscribe("camera/image_raw", 10, callback, nullptr, &TransportHints("compressed"));`. Plugin selection is via `TransportHints` strings. ([image_transport.hpp](https://github.com/ros-perception/image_common/blob/humble/image_transport/include/image_transport/image_transport.hpp))
- **Transport plugins** (separate apt packages):
  - `compressed_image_transport` — JPEG/PNG, default for color.
  - `theora_image_transport` — Theora video, higher rates over WiFi.
  - `compressed_depth_image_transport` — RVL/PNG for `16UC1` depth, mandatory over WiFi. ([image_common](https://github.com/ros-perception/image_common))
- **When raw is correct:** if perception node and camera share a host over zero-copy DDS / Cyclone shared mem, raw avoids per-frame JPEG decode. Cross-host → always compressed.

**cv_bridge encoding strings (ROS Humble):** the canonical list is in `sensor_msgs/include/sensor_msgs/image_encodings.hpp`:

- Color: `rgb8`, `rgba8`, `rgb16`, `rgba16`, `bgr8`, `bgra8`, `bgr16`, `bgra16`
- Mono / depth: `mono8`, `mono16`
- Generic OpenCV types: `8UC1`–`8UC4`, `16UC1`–`16UC4`, `32FC1`–`32FC4`, etc.
- Bayer: `bayer_rggb8`, `bayer_bggr8`, `bayer_gbrg8`, `bayer_grbg8` (+ 16-bit variants)
- YUV / NV: `yuv422`, `yuv422_yuy2`, `nv21`, `nv24`
- ([sensor_msgs/image_encodings.hpp](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/include/sensor_msgs/image_encodings.hpp))

**OpenCV BGR vs ROS RGB.** OpenCV stores color as BGR by default (`cv::imread`, `cv::cvtColor`, every imshow). ROS allows either, with the encoding tag deciding. Two common bugs: (1) subscriber requests `bgr8` from an RGB source — cv_bridge silently swaps R/B; (2) publisher tags an OpenCV BGR array as `rgb8` — wrong colors in RViz and HSV thresholds.

**Defensive pattern:** explicitly request the encoding you expect (`desired_encoding="bgr8"`) so cv_bridge converts; don't pass `"passthrough"` for color images. Use `"passthrough"` for mono masks and `16UC1` depth to avoid lossy conversion. `mono8` and `bgr8` are what most OpenCV functions natively expect. ([Robotics SE answer](https://answers.ros.org/question/65517/sensor_msgsimage-encoding-conversion/))

---

## 5. PointCloud2 Anatomy

`sensor_msgs/PointCloud2` (Humble) fields, in order:

- `std_msgs/Header header`
- `uint32 height` — number of rows. **`height > 1` ⇒ organized cloud** (image-like 2D layout). `height == 1` ⇒ unorganized (1D point list, `width = N`). ([PointCloud2.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud2.msg))
- `uint32 width` — number of columns / total points if unorganized.
- `sensor_msgs/PointField[] fields` — describes the data layout: name (`x`, `y`, `z`, `rgb`, `ring`, `intensity`, …), `offset`, `datatype` (FLOAT32 = 7, etc.), `count`. Velodyne adds `ring`/`time`; ZED packs `rgb` as a float-bitcast or `rgba` as 4×UINT8.
- `bool is_bigendian`
- `uint32 point_step` — bytes per point (e.g. xyz+rgb = 16 with 4-byte alignment).
- `uint32 row_step` — bytes per row; for organized clouds equals `point_step * width` (+ alignment); for unorganized clouds same identity.
- `uint8[] data` — raw bytes, length `row_step * height`.
- `bool is_dense` — true iff every point is finite. **Organized clouds from depth cameras are almost always `is_dense = false`** because invalid pixels (no depth return) become `(NaN, NaN, NaN)` rather than being culled (so the 2D structure is preserved).

Organized vs unorganized:

- **Organized** (depth camera, ZED, RealSense) — point at row `r`, col `c` is byte `data[r*row_step + c*point_step]`. Lets perception pipelines do a per-pixel mask lookup in O(1). Required by costmap layers like kiwicampus.
- **Unorganized** (LiDAR, accumulated clouds) — flat list, no spatial layout. Cheaper to filter/voxel-grid; useless for image-aligned perception.

**Consuming the cloud:**

- Slow path: `sensor_msgs_py.point_cloud2.read_points(cloud, field_names=("x","y","z"), skip_nans=False)` — Python generator, fine for offline analysis. ([sensor_msgs_py](https://github.com/ros2/common_interfaces/tree/humble/sensor_msgs_py))
- Fast path: `np.frombuffer(cloud.data, dtype=structured_dtype).reshape((height, width))` is a zero-copy structured-dtype view. For ZED's 16-byte points (xyz f32 + rgb f32), `dtype = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<f4')])`. Required for >5 Hz on a Jetson.
- NaN handling: never assume `is_dense=True`; always mask with `np.isfinite(z)` before geometry.

---

## 6. TF for Sensors (REP 105 / REP 103)

REP 105 defines the platform frame chain: `earth → map → odom → base_link`. ([REP 105](https://www.ros.org/reps/rep-0105.html))

REP 103 defines the body and optical conventions:

- **Body frame:** x forward, y left, z up — right-handed.
- **Optical frame:** z forward, x right, y down — also right-handed but rotated 90° from body.
- The **`_optical` suffix** marks frames that follow the optical convention. ([REP 103](https://www.ros.org/reps/rep-0103.html))

**Why two frames per camera.** The mounting frame uses body convention so URDF/TF tools render correctly. Image processing (intrinsics, projection, OpenCV) assumes optical convention because that's how image axes naturally point.

**Recommended URDF chain for a camera:**

```
base_link
  └── <cam>_camera_link              (body convention, mount pose)
        ├── <cam>_camera_center      (optional, image plane center)
        ├── <cam>_left_camera_frame  (left lens, body convention)
        │     └── <cam>_left_camera_frame_optical  (rotated to optical)
        └── <cam>_right_camera_frame
              └── <cam>_right_camera_frame_optical
```

Image messages and clouds use `frame_id = <cam>_left_camera_frame_optical`. The mount static transform is the ONE place to put measured offsets. ([REP 103](https://www.ros.org/reps/rep-0103.html))

**Use upstream xacro macros, never hand-roll.** Vendor wrappers ship `urdf/<sensor>_macro.urdf.xacro` (ZED's `zed_macro.urdf.xacro`, RealSense's `_d435.urdf.xacro`) that emit body+optical chains with correct rotations. Hand-rolling causes silent off-by-90° errors. ([zed-ros2-wrapper urdf](https://github.com/stereolabs/zed-ros2-wrapper/tree/master/zed_wrapper/urdf))

**Common pitfall:** publishing `Image.header.frame_id = "<cam>_camera_link"` instead of `..._optical`. Downstream `tf2.lookupTransform` then projects pixels with the wrong basis — points appear rotated 90°.

---

## 7. ZED ROS 2 Wrapper Specifics

(Targets: stereolabs/zed-ros2-wrapper v5.1+, ZED SDK 5.x.)

- **Composable component, not standalone executable.** The camera node is `stereolabs::ZedCamera` in `zed_components`. The `zed_wrapper` package is a metapackage; `launch/zed_camera.launch.py` does manual composition. Do NOT try `Node(package='zed_wrapper', executable='zed_wrapper')` — use `IncludeLaunchDescription` of `zed_camera.launch.py`. ([zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper))
- **The `namespace` / `node_name` / `camera_name` gotcha.** Logic in `zed_camera.launch.py`:

  ```python
  if namespace_val == '':
      namespace_val = camera_name_val      # default: namespace = camera_name
  else:
      node_name_val = camera_name_val      # if you pass namespace, node_name is overwritten
  ```

  Pass **only** `camera_name`, `camera_model`, `serial_number` to get conventional `/<camera_name>/zed_node/...` tree. Passing both `namespace` and `node_name` collapses to `/<camera_name>/<camera_name>/...`. ([zed_camera.launch.py](https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/launch/zed_camera.launch.py))
- **v5 topic naming (breaking change vs v4):** new convention is `~/<sensor_type>/<color_model>/<rect_type>/image`. Examples:
  - `~/rgb/color/rect/image` — rectified left color (default)
  - `~/rgb/color/raw/image` — unrectified
  - `~/depth/depth_registered`
  - `~/point_cloud/cloud_registered`
  - Old v4 `~/rgb/image_rect_color`, `~/left/image_rect_color` are gone in v5.1.0; subscribers must update. ([Stereolabs ZED Stereo Node](https://www.stereolabs.com/docs/ros2/zed-node))
- **`point_cloud_res` is independent of `pub_downscale_factor`.** Image resolution: `pub_resolution` + `pub_downscale_factor` (default 2.0 → 540×960 from HD1080). Cloud resolution: `point_cloud_res` ∈ {`COMPACT`, `REDUCED`}. Cloud HxW derives from depth resolution, NOT image downscale — published cloud and image will *not* match shape unless aligned deliberately. ([common_stereo.yaml](https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/config/common_stereo.yaml))
- **Perception implication:** if downstream requires mask HxW == cloud HxW, resize **image** to cloud's shape before segmentation. Don't upsample the cloud.
- **v5 enum changes** (using v4 values throws `InvalidParameterValueException`):
  - `depth_mode`: `NONE | NEURAL_LIGHT | NEURAL | NEURAL_PLUS` (old `PERFORMANCE/QUALITY/ULTRA` gone).
  - `grab_resolution` for ZED X: `HD1200 | HD1080 | SVGA | AUTO` (no `HD720`).
- **Default QoS:** `RELIABLE`, `KEEP_LAST(10)`, `VOLATILE`. Switch subscriber to `BEST_EFFORT` for tolerant multi-host streaming. ([Stereolabs ZED docs](https://www.stereolabs.com/docs/ros2/zed-node))
- **`serial_number` is a launch arg, not YAML param** — pass via `launch_arguments` on `IncludeLaunchDescription`; YAML is ignored. ([zed_camera.launch.py](https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/launch/zed_camera.launch.py))

---

## 8. HSV / Classical Vision Pipelines

Classical color thresholding still has a place in 2026 — knowing when is the standard.

**When HSV thresholding is appropriate:**

- Bright, saturated, single-color targets under semi-controlled lighting: cones, lane lines (white/yellow), traffic vests, painted gates. Hue is the dominant feature; saturation gates out grays.
- Tight latency budgets where 10–50× speedup over CNN inference matters more than a few percentage points of recall — costmap layers at 15 Hz on Jetson with shared GPU.
- Bringup / regression baseline — get a working pipeline first, swap in ONNX later. Interface (mask + organized cloud + LabelInfo) is identical.

**When HSV is wrong and ONNX/learning is necessary:**

- Outdoor scenes with mixed lighting, shadow boundaries, varying white balance — HSV's hue rotates with white balance.
- Multi-class semantic segmentation (sidewalk vs grass vs gravel vs asphalt) — texture, not color.
- Objects whose appearance varies across instances (people, vehicles).
- Motion blur, glare, rain on the lens — learned features generalize, thresholds don't.

**Common HSV pitfalls:**

- OpenCV hue range is `[0, 179]`, not `[0, 359]` (8-bit storage). Writing `H ∈ [0, 60]` for "red-yellow" expecting degrees yields a tiny slice. ([OpenCV docs](https://docs.opencv.org))
- Red wraps around 0 — covers `[0, 10]` AND `[170, 179]`. A single range misses half the reds; use two ranges + `cv::bitwise_or`.
- Lighting kills it. Shadow desaturates, sun saturates to white. Always gate on saturation AND value, not just hue. Calibrate at run time-of-day.
- Camera gamma / WB differ across vendors. Thresholds tuned on RealSense will miss on ZED.
- JPEG artifacts (compressed transport) widen the effective gamut ±5 in HSV — bake in slack.

**Test strategies:**

- Synthetic images: 256×256 mosaic of (H, S, V) tiles, assert mask matches. Catches sign / range bugs.
- Real captures: record 10-second `ros2 bag` per session (sun, overcast, dusk), replay through unit tests, assert IoU >= baseline.
- Threshold sweeps: grid over `H_low, H_high, S_low, V_low`, pick centroid of IoU plateau (not peak — more robust).
- Confusion matrix on a held-out frame — even 5 hand-painted frames give precision/recall.

---

## 9. kiwicampus/semantic_segmentation_layer Integration Contract

The plugin is a Nav2 `Layer` (costmap_2d plugin) that consumes a per-camera segmentation stream and writes graded costs into the costmap. Its contract with upstream perception:

- **Topics per source (`observation_sources: <name>` then `<name>: { ... }`):** ([kiwicampus README](https://github.com/kiwicampus/semantic_segmentation_layer/blob/humble/README.md))
  - `segmentation_topic` — `sensor_msgs/Image`, encoding **`mono8`** (label IDs ∈ `[0, 255]`). QoS: best-effort sensor data.
  - `pointcloud_topic` — `sensor_msgs/PointCloud2`, organized (`height > 1`), **same HxW as mask**, same `header.stamp`, same `header.frame_id`. QoS: best-effort sensor data.
  - `confidence_topic` (optional) — `sensor_msgs/Image`, `mono8`, same shape and stamp as mask.
  - `labels_topic` — `vision_msgs/LabelInfo`, latched (TRANSIENT_LOCAL + RELIABLE).
- **`class_types` MUST be nested inside the per-source block, not the plugin top level.** Schema:
  ```yaml
  semantic_segmentation_layer:
    plugin: "semantic_segmentation_layer::SemanticSegmentationLayer"
    observation_sources: camera
    camera:
      class_types: ["traversable", "danger"]
      traversable:
        classes: ["sidewalk"]
        base_cost: 0
        max_cost: 0
      danger:
        classes: ["grass"]
        base_cost: 254
        max_cost: 254
  ```
  Putting `class_types` at the top level (like Nav2's `obstacle_layer` allows for some keys) makes the plugin log "no class types defined for source X" and silently mark nothing. ([kiwicampus README](https://github.com/kiwicampus/semantic_segmentation_layer/blob/humble/README.md))
- **Per-class-type fields:** `classes` (list of class_name strings — must match `LabelInfo.class_map[].class_name` exactly), `base_cost`, `max_cost` (0–254; 255 reserved for lethal), `mark_confidence`, `samples_to_max_cost`, `dominant_priority`.
- **Other knobs:** `observation_persistence`, `expected_update_rate`, `tile_map_decay_time`, `max_obstacle_distance`, `min_obstacle_distance`, `use_cost_selection`, `visualize_tile_map`.

**Known failure modes** (all silent — no callback, no marks):

- `LabelInfo` with default `VOLATILE` QoS — plugin late-joins, never sees palette.
- Mask and cloud HxW differ (typical ZED bug: 540×960 image vs 256×448 cloud).
- Mask and cloud `stamp` differ by more than synchronizer slop.
- `class_name` in YAML doesn't match `LabelInfo.class_map[].class_name` (case / underscore).
- `class_types` at top level instead of nested under source.
- Cloud unorganized (`height == 1`) — per-pixel projection impossible.

---

## 10. Anti-Patterns (Things Reviewers Should Flag)

- **BGR/RGB confusion** — `bgr8` requested from RGB source, or BGR array tagged `rgb8`. R/B swapped in any HSV mask. ([cv_bridge](https://github.com/ros-perception/vision_opencv/tree/humble/cv_bridge))
- **Dropping stamps** — derived mask with `header.stamp = node.now()` instead of inheriting from source image. Breaks every downstream `ApproximateTime`. ([sensor_msgs/Image.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg))
- **Unorganized cloud** when organized expected (`height = 1`); consumers indexed by `(row, col)` segfault.
- **Wrong frame_id** — `camera_link` instead of `..._optical`. ([REP 103](https://www.ros.org/reps/rep-0103.html))
- **Blocking neural-net or heavy CV in image callback** — rclpy default executor is single-threaded; 200 ms inference blocks all subscriptions including `/tf`. Fix: `MultiThreadedExecutor` + reentrant callback group, or worker thread.
- **No rate limiting** — 30 Hz camera, 10 Hz perception → message_filters queues blow up. Use `qos_profile_sensor_data` (best-effort, depth=5).
- **Ignoring `CameraInfo`** — without intrinsics you cannot project pixels to 3D. ([image_pipeline](https://github.com/ros-perception/image_pipeline/tree/humble))
- **`allow_headerless=True`** on `ApproximateTimeSynchronizer` — hides upstream stamping bugs. ([rclpy bindings](https://github.com/ros2/message_filters/blob/humble/src/message_filters/__init__.py))
- **`passthrough` for color images** — pin `bgr8` or `rgb8` so cv_bridge converts.
- **Hand-rolled URDF for cameras** instead of vendor xacro macro — misses optical-frame rotations.
- **`LabelInfo` published with default `VOLATILE` QoS** — late-joining costmap plugin never sees it.
- **Hardcoding numeric class IDs** instead of looking up by `class_name` from `LabelInfo.class_map` — breaks on model retraining.
- **Same stamp on every message** (`now()` called once at startup) — breaks ApproximateTime entirely. ([ApproximateTime docs](https://docs.ros.org/en/ros2_packages/humble/api/message_filters/doc/index.html))

---

## Quick Checklist for Reviewers

A reviewer comparing a real perception node to upstream standards should confirm each of these. Each should be a clear yes/no.

1. [ ] `LabelInfo` is published with `TRANSIENT_LOCAL` + `RELIABLE` QoS (latched).
2. [ ] `LabelInfo` topic sits at the same namespace level as the mask topic (e.g. `/perception/<cam>/label_info` next to `/perception/<cam>/mask`).
3. [ ] `LabelInfo.class_map` IDs are `uint16` and reserve the max value (`65535` or `255` for `uint8` masks) for `UNLABELED`.
4. [ ] Mask, cloud, (optional) confidence share **bit-identical** `header.stamp` (inherited from the source image).
5. [ ] Mask, cloud, confidence share `header.frame_id` set to the camera **optical** frame (`*_optical` suffix).
6. [ ] Mask and cloud have identical `(height, width)`; cloud is organized (`height > 1`).
7. [ ] Mask is `mono8` encoding, IDs match `class_map`.
8. [ ] Cloud `is_dense` is `false` (or rigorously verified true) and consumers handle NaN.
9. [ ] `ApproximateTimeSynchronizer` `slop` is `≤ 1 / camera_rate`; `queue_size` ≥ `slop * rate + 2`.
10. [ ] `allow_headerless` is `False`.
11. [ ] No two messages on the same topic ever share `header.stamp`.
12. [ ] Image subscribers use `image_transport` (not raw `Image`) when crossing host boundaries.
13. [ ] `cv_bridge` calls request explicit encodings (`bgr8` / `mono8` / `passthrough` for depth), never `passthrough` for color.
14. [ ] Camera URDF includes the vendor xacro macro (`zed_macro.urdf.xacro` / `_d435.urdf.xacro`) — no hand-rolled optical-frame rotations.
15. [ ] ZED launch passes only `camera_name`, `camera_model`, `serial_number` (not both `namespace` and `node_name`); `serial_number` is in `launch_arguments`, not YAML.
16. [ ] ZED v5 topic names used (`rgb/color/rect/image`, `point_cloud/cloud_registered`); no v4 leftovers (`rgb/image_rect_color`).
17. [ ] Image is resized to cloud HxW **before** segmentation runs.
18. [ ] Heavy inference does not block the image callback (multi-threaded executor or worker thread / process).
19. [ ] Perception node respects `qos_profile_sensor_data` (best-effort, depth small) on input image / cloud topics.
20. [ ] Subscriber for `LabelInfo` declares `TRANSIENT_LOCAL` durability or no palette will arrive.
21. [ ] kiwicampus layer config has `class_types` nested **inside** the per-source block, not at plugin root.
22. [ ] `class_name` strings in costmap config exactly match `class_map[].class_name` from `LabelInfo`.
23. [ ] Pipeline tests include synthetic HSV mosaic + real capture + threshold sweep (for HSV pipelines specifically).
24. [ ] `CameraInfo` is consumed (or cached) — never project pixels to 3D without intrinsics.
25. [ ] Numeric class IDs are looked up by name from `LabelInfo`, not hardcoded.
