# avros_lidar: what it is, how it compares to the standard, and where it should live (2026-10-09)

## 1. What `avros_lidar` actually does (read from the code, not assumed)

Confirmed from `src/avros_lidar/` on the Jetson (ament_python package, maintainer Changwe Musonda, Apache-2.0, depends on `velodyne_driver`/`velodyne_pointcloud`/`avros_bringup`):

```
VLP-16 --UDP--> velodyne_driver_node --> velodyne_transform_node --> /velodyne_points
                                                                         |
                                                                         v
                                                              lidar_preprocessor
                                      range/FOV filter -> voxel downsample -> transform to base_link
                                      -> self-footprint removal -> RANSAC ground removal -> height band
                                                                         |
                          /perception/lidar/obstacles, /origin, /health v
                                                                   voxel_mapper
                                      log-odds voxel grid, raycasting (CUDA on Jetson), decay, inflation
                                                                         |
                                                                         v
                                                    /perception/lidar/voxels, voxel_grid, costmap_2d, ...
```

It is a genuinely well-built, complete 3D occupancy mapper, not a toy: Bayesian log-odds occupancy (same family as OctoMap), 3D Bresenham raycasting with an optional numba-CUDA GPU kernel (lazy-imported, falls back to CPU — same convention `avros_perception/pipelines/yolopv2.py` already uses), time-based decay (~0.6 s), inflation, and it publishes a rich set of outputs: raw obstacle points, occupied-voxel centers, inflated voxels, a 2D top-down `OccupancyGrid` (`obstacle_costmap_2d`), the full flattened 3D grid + JSON metadata for programmatic access, and RViz markers.

One detail that matters for the placement question: `voxel_grid.py` already defines `SOURCE_LIDAR`/`SOURCE_DEPTH`/`SOURCE_BOTH` constants — it's architected to fuse a depth camera (the ZED) into the *same* voxel grid alongside lidar, not just lidar alone, even though that fusion isn't wired up yet. That's a "perception/mapping" concern, not a lidar-specific one.

**Current integration state (confirmed, matches earlier findings this session):** it runs as a **standalone ROS 2 node graph**, publishing to `/perception/lidar/*`. It is **not** wired into Nav2 in any way — nothing in `nav2_params_igvc_autonav.yaml` subscribes to its topics. The only consumer referencing it is `avros_sim`.

## 2. The standard this project already uses for the same job

This project's Nav2 local costmap already does 3D lidar obstacle detection today, via **STVL (`spatio_temporal_voxel_layer`)** — confirmed in `CLAUDE.md`'s costmap config and in this session's own memory of STVL-specific incidents (decay tuning, the Velodyne broadcast/flicker root cause, `obstacle_range` kept short to avoid the ramp). STVL is documented by Nav2 itself as *the* standard plugin for exactly this: a 3D voxel world model built from lidar/depth/sonar, with time-based decay via a sensor-model clearing frustum, loaded as a `nav2_costmap_2d` layer plugin (`spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`) ([Nav2 STVL tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_stvl.html), [STVL repo](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)). Nav2 also ships a simpler built-in `voxel_layer` in `nav2_costmap_2d` for the same purpose without the extra dependency, but STVL is the one with real lidar-specific support (it models the VLP-16's hourglass FOV, supports an adjustable `hFOV` for the clearing frustum) and better decay semantics.

**The core fact that should drive this decision: `avros_lidar`'s `voxel_mapper` is pure Python (`rclpy`/`numpy`/`scipy`/`numba`), and Nav2 costmap layers must be C++ pluginlib classes** — confirmed from the Nav2 plugin-tutorial docs: the layer base class is C++, registered via the `PLUGINLIB_EXPORT_CLASS` macro, loaded by `LayeredCostmap` as that C++ base type ([Writing a New Costmap2D Plugin](https://docs.nav2.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html)). There is no Python costmap-layer path in Humble. So `avros_lidar` **cannot become a Nav2 costmap plugin as-is** — the only way to feed its output into the actual navigation costmap is a thin C++ shim layer that subscribes to its `OccupancyGrid`/`PointCloud2` and marks costs in `updateCosts()`, or repurposing `nav2_costmap_2d::StaticLayer`'s topic subscription (built for a SLAM map, not a live rolling local layer — workable but non-standard).

**What this means concretely: `avros_lidar` and STVL are not the same kind of thing and don't compete for the same slot.** STVL is the *safety-critical, Nav2-integrated* 3D obstacle layer already proven on this robot. `avros_lidar` is a *parallel, standalone* 3D perception/mapping node with capabilities STVL doesn't have (full queryable 3D grid, multi-sensor fusion hook, markers) but zero current path into Nav2's actual planning costmap.

## 3. Where lidar processing belongs — the general standard

Checked ROS 2 convention and a mature reference implementation (Autoware):

- **General ROS 2 convention:** drivers publish raw data; a separate package does the processing; hardware-level detail should not leak into consumer packages ([Robotics Stack Exchange discussion](https://answers.ros.org/question/410977/best-practices-for-system-design-and-package-dependencies/), [sensor package discourse thread](https://discourse.openrobotics.org/t/what-makes-a-good-sensor-package-a-list-of-ros-2-sensors/38805)). This project already follows that split: `velodyne_driver`/`velodyne_pointcloud` (apt, raw) feeding into a separate processing package.
- **Autoware** (a much larger, mature full-AV ROS 2 stack) puts lidar point-cloud preprocessing (crop filter → motion-distortion correction → outlier removal → downsampling → transform → concatenation → ground segmentation) in its own dedicated `pointcloud_preprocessor` package under a "sensing" layer, separate from both the driver and the higher-level detection/tracking packages ([Autoware point cloud design doc](https://docs.autoware.org/main/design/autoware-architecture-v1/components/sensing/data-types/point-cloud/)). That's structurally the same shape `avros_lidar` already has.
- **This project's own existing convention** already matches this: one package per concern — `avros_bringup` (launch/config), `avros_control` (actuation), `avros_perception` (camera pipeline), `avros_navigation` (route graph tooling), `avros_webui`, `avros_msgs`. A dedicated `avros_lidar` for lidar-specific processing fits this pattern exactly.

**Conclusion on "should it be its own package": yes — this is already correct and matches both general ROS 2 convention and this project's own established layout.** There's no standard argument for folding it into `avros_perception` (that package is camera/vision-pipeline-specific, with its own `pipelines/` plugin registry for image processing) or into `avros_bringup` (launch/config only, no processing logic belongs there).

## 4. What actually needs a decision (this is the real question, not the package boundary)

The package *boundary* is already right. The open question is **what `avros_lidar` is *for*, relative to STVL**, because that decides what changes next:

| Option | What it means | Tradeoff |
|---|---|---|
| **A. Replace STVL with `avros_lidar`** | Build the C++ shim layer (or repurpose `StaticLayer`) so `voxel_mapper`'s output becomes Nav2's actual local-costmap obstacle source | Re-implements a mature, community-maintained, battle-tested C++ plugin in Python+numba; real maintenance cost and an unproven performance ceiling vs. STVL's native code, for a safety-critical path. Needs a C++ bridge regardless — it can't be a drop-in plugin. |
| **B. Drop `avros_lidar`, keep STVL only** | Leave navigation exactly as it is today | Loses the queryable full 3D grid, the markers, and the lidar+depth fusion hook — capabilities nobody has asked for yet, but they exist in the code already |
| **C. Run both, for different jobs (recommended)** | STVL stays the only thing feeding Nav2's actual costmap (unchanged, zero risk to navigation). `avros_lidar` stays a separate, standalone perception package for things STVL doesn't do: a queryable 3D voxel grid for future semantic/depth fusion, richer visualization/debugging, or a future planner that wants full 3D awareness | No safety-path risk. Keeps the investment in `voxel_mapper` alive for its actual differentiator (3D queryable grid + multi-sensor fusion) instead of racing STVL at STVL's own job |

**Recommendation: C.** Don't touch STVL or the navigation costmap. Keep `avros_lidar` as the separate package it already correctly is, and point it at the problem it's actually good for — the 3D-queryable grid and the lidar+depth fusion hook already stubbed in via `SOURCE_BOTH` — rather than trying to make it replace a proven C++ Nav2 plugin through a Python node that structurally can't be one.

## 5. Concrete layout recommendation

Keep the package split as-is; it's already right. Three structural changes worth making, in order of value:

1. **Decide and document the relationship to STVL explicitly** (this document is a start) — right now nothing says whether `avros_lidar` is meant to replace, feed, or run alongside STVL, and that ambiguity is why it's unwired six+ months in. Put the decision (Option C above, or your call) in the package's own `README.md`.
2. **If any output needs to reach Nav2's planner, go through a thin C++ bridge, not a Python plugin** — e.g. a ~50-line `nav2_costmap_2d::Layer` subclass that subscribes to `/perception/lidar/obstacle_costmap_2d` and marks cells in `updateCosts()`. This is the one piece of new code the "standard" path actually requires, and it doesn't exist yet in any package.
3. **If the depth-camera fusion (`SOURCE_DEPTH`/`SOURCE_BOTH`) is ever pursued, keep it inside `avros_lidar`** (rename consideration aside — it's already the project's designated 3D-voxel-fusion package) rather than creating a second, competing fusion node in `avros_perception`.

## Sources
- [Using an External Costmap Plugin (STVL) — Nav2 docs](https://docs.nav2.org/tutorials/docs/navigation2_with_stvl.html)
- [spatio_temporal_voxel_layer (STVL) repo](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)
- [Writing a New Costmap2D Plugin — Nav2 docs](https://docs.nav2.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html)
- [Best practices for system design and package dependencies — ROS Answers](https://answers.ros.org/question/410977/best-practices-for-system-design-and-package-dependencies/)
- [What Makes a Good Sensor Package? — ROS Discourse](https://discourse.openrobotics.org/t/what-makes-a-good-sensor-package-a-list-of-ros-2-sensors/38805)
- [Autoware point cloud data design](https://docs.autoware.org/main/design/autoware-architecture-v1/components/sensing/data-types/point-cloud/)
