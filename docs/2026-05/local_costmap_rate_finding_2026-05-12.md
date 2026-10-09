# local_costmap publishes at ~2.2 Hz instead of 10 Hz — finding

**Status:** OPEN — observed but not yet root-caused.
**Severity:** MED — MPPI runs at 20 Hz against a costmap that updates 5× slower than configured. Stale-grid behavior is masked at low speed (vx_max=0.5 m/s the robot moves <2 cm between costmap frames) but will show up as lane-clipping or late obstacle reactions when `vx_max` is bumped toward the rule cap (2.235 m/s).

## Observation

On Jetson, fresh launch with:

```
ros2 launch avros_bringup navigation.launch.py \
  enable_velodyne:=false enable_zed_front:=true enable_perception:=true \
  perception_cameras:=front bt_xml:=navigate_igvc_autonav_humble.xml
```

```
$ ros2 topic hz /local_costmap/costmap
average rate: 2.176
        min: 0.456s max: 0.467s std dev: 0.00406s window: 4
```

Configured in `src/avros_bringup/config/nav2_params_humble.yaml`:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 10.0
```

Observed = 22% of configured. Std-dev is tight (4 ms) — this is a sustained rate, not a startup transient.

## What's *not* the cause

- **Not the voxel_layer re-add** (commit `64203d5`, 2026-05-12). Rate was the same (~2.87 Hz) on the previous launch with `plugins: ["semantic_layer", "inflation_layer"]` and no voxel.
- **Not Velodyne load.** The observation above was made with `enable_velodyne:=false` — there's no /velodyne_points traffic.
- **Not LiDAR latency.** Same reason.

So the bottleneck is in the `semantic_layer` ↔ `inflation_layer` ↔ master-costmap-publish chain itself, not in any sensor source.

## Candidate causes (untested, ranked by suspicion)

1. **`semantic_layer.updateBounds` cycle time exceeds 100 ms.** PR3 raytrace-clear (commit `b357882` on `avros-fixes`) iterates every cell in the rolling window per update; at 50×50 cells × 0.2 m (current size) that's 2500 cells × 3 ray-marches each. Patch was tuned for 30×30 m @ 0.2 m windows; we may be exceeding the budget at 50×50. P3.1.5 (reshape to 30×30 m @ 0.1 m = 90 k cells) would make this worse, not better.
2. **`tile_map_decay_time: 0.3 s` purge runs every cycle.** O(N) walk of the `temporal_tile_map_` under the recursive mutex added by PR2 (`76fdf92`). If the tile map has accumulated thousands of entries, the purge is the bottleneck.
3. **kiwicampus "CRITICAL ERROR" log spam for `free`/`unknown` classes.** Fires once per LabelInfo arrival (latched, transient_local — once per subscriber, not once per frame), so probably not the cause. Worth ruling out.
4. **rclcpp executor starvation.** controller_server is single-threaded by default; if `FollowPath`'s MPPI cycle (controller_frequency 20 Hz × time_steps 56) eats more than 50 ms, the costmap update callback gets queued. Looking for this would mean enabling `--use-multi-threaded-executor` on controller_server.
5. **publish_frequency rate limiter, not update_frequency.** Nav2 separates the two — update_frequency drives the internal update loop, publish_frequency drives the topic publication. If only publish is throttled (not update), the planner still sees a 10 Hz costmap internally even if `topic hz` says 2.2. Test: query the master costmap subscribers' reception rate, not the publisher.

## Next steps

Pick one, do not pick all:

- **(A) Profile first.** Drop `time_steps` (MPPI) to 20 and re-measure. If rate jumps, executor starvation is the cause; switch controller_server to multi-threaded.
- **(B) Isolate the semantic_layer.** Set `local_costmap.plugins: ["voxel_layer", "inflation_layer"]` (drop semantic), launch, measure. If rate returns to 10 Hz, kiwicampus is the bottleneck.
- **(C) Investigate publish vs update.** Add a subscriber log on `/local_costmap/costmap_raw` instead of the throttled `/costmap`. If `_raw` is 10 Hz, then `publish_frequency` is the only throttle and there's no actual problem — `topic hz` on `/costmap` is just measuring the publish-side rate.

(C) is the cheapest sanity check and should run first.

## Why this matters for IGVC

At competition speed (`vx_max: 1.5` m/s, eventually bumping toward 2.0+), a 2.2 Hz costmap means MPPI plans on a grid that's 0.7 m behind reality between updates. Lane-as-LETHAL with 0.85 m inflation gives a 0.85 m safety cushion — so we're eating half the cushion in costmap staleness. Either close the rate gap or shrink the safety envelope (worse).

## Verification record

- Workstation HEAD: `64203d5 nav2_humble: re-add voxel_layer to local_costmap plugins (LiDAR online)`
- Jetson HEAD: same.
- Measurement reproducible — same rate observed before voxel fix (~2.87 Hz with semantic+inflation) and after (~2.18 Hz with voxel+semantic+inflation). Voxel adds marginal cost (the layer is loaded but receives no /velodyne_points during this test).
