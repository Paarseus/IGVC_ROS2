# avros_navigation - Review

## Summary

`avros_navigation` is a one-file ament_python package containing a
single offline tool: `scripts/generate_graph.py`. It downloads OSM
data via OSMnx, densifies edges along road curves, shifts to the
campus datum, and writes a `nav2_route` GeoJSON graph. It is
**not** a runtime ROS package.

**The schema is correct.** Nodes carry `frame: 'map'`, edges carry
`cost`, bidirectional roads emit both directions, and the datum
offset is applied in the projected plane so output coordinates
match `navsat_transform_node`'s map frame. **No P0 schema bug.**

**P0 issues are workflow-shaped, not code-shaped.** The committed
graph is for CPP campus via OSMnx; the IGVC AutoNav venue has no
useful OSM data. The script supports neither hand-authoring from
the IGVC course map nor importing a GPS walk. With one month to
competition, a CSV-input mode is the highest-leverage missing
feature. Also: the committed graph has **16,651 nodes / 17,492
edges**, not the "52 nodes, 113 edges" CLAUDE.md claims - sanity
check on Jetson is urgent.

Secondary: third-party deps (`osmnx`, `networkx`, `pyproj`,
`shapely`) are undeclared; the script is not installed via
`setup.py` (only reachable by a path-relative invocation); osmnx
is not version-pinned; there are no unit tests; `package.xml`
declares 7 unused ROS deps.

Against the 18-point ROS-2-Python checklist, this package scores
~12-13/18 - clean skeleton, boilerplate tests pass, but missing
runtime dep declarations, dead deps, and uninstalled script knock
several items off. No rclpy code so the Node/QoS/parameter items
are vacuously fine. **Acceptable for bench testing; needs P0 #1
(CSV input) and P0 #2 (verify graph size) before competition.**

## Per-file findings

### scripts/generate_graph.py

`src/avros_navigation/scripts/generate_graph.py:1-332` is the only
substantive file in the package - an offline OSMnx -> nav2_route
GeoJSON generator, not a ROS node. Pipeline:

1. `build_graph` (lines 57-62) - `ox.graph_from_place(place,
   network_type='drive', simplify=True)`. Default place is the
   entire CPP campus.
2. `densify_graph` (lines 65-154) - projects to UTM via
   `ox.project_graph`, then walks each edge and inserts
   intermediate nodes via `LineString.interpolate(dist)` so the
   straight-line segments between adjacent graph nodes follow the
   road curve. Edges shorter than `1.5 * spacing` are passed
   through.
3. `graph_to_geojson` (lines 157-270) - lifts `(datum_lat,
   datum_lon)` to UTM via pyproj and subtracts that from each
   node's projected XY, producing **map-frame meters from datum**.
   Each Point gets `{id, frame: 'map'}`; each MultiLineString edge
   gets `{id, startid, endid, cost}` plus optional
   `metadata.speed_limit` (mph -> m/s, line 249).
4. `main` (lines 273-328) - argparse, run pipeline, write
   pretty-printed JSON.

The script is executable with a `#!/usr/bin/env python3` shebang.

What is wrong or fragile:

- **OSMnx version drift, no pin.** OSMnx 2.0.0 (Sep 2024) renamed
  many functions and removed positional kwargs. A fresh
  `pip install osmnx` could break the script silently.
- **No empty-graph handling.** If `ox.graph_from_place` returns
  empty (typo, network outage), `max(G_proj.nodes)` on line 89
  raises `ValueError` with no friendly error.
- **No determinism / OSM snapshot.** Live Overpass API; same
  `--place` on different days yields different graphs.
  `date_generated` (line 266) is locale-dependent `%c` and records
  wall-clock, not OSM version.
- **`overridable` flag absent on edges** - recommended by the
  nav2_route schema (`standards_nav2_localization.md` section 7).
- **EPSG header misleading.** Line 264 advertises `EPSG:32611`
  but the coordinates are datum-shifted, so they are NOT in
  EPSG:32611 - they are `EPSG:32611 - datum`. Either drop the
  `crs` block or use a custom URN.
- **Datum vs UTM-grid-north skew.** UTM `+y` is grid-north;
  navsat_transform_node aligns `map` with **true ENU** after
  `magnetic_declination_radians` (per `standards_nav2_
  localization.md` section 3). The graph and live `map` frame
  differ by the local meridian convergence - sub-meter for CPP,
  larger at higher latitudes. Worth rotating UTM -> local ENU
  about the datum at lines 191-192 / 220-221.
- **`int()` on UTM zone** (line 164) - footgun at antimeridian,
  use `math.floor`.
- **`metadata.speed_limit` is dead code for IGVC** - the
  competition course has no OSM `maxspeed` tags.
- **`street_count=2` on intermediate nodes** (line 131) is
  misleading - OSMnx `street_count` means "intersections", not
  defined for an interpolated point.
- **No `--bbox` / `--polygon` flags** - the only place selector is
  `--place` (Nominatim string).
- **No runtime check that `--datum-lat / --datum-lon` matches
  `navsat.yaml`.** Lines 40-41 hard-code the datum with a comment
  saying "must match" but no enforcement.
- **JSON not git-stable across re-runs.** MultiDiGraph parallel-edge
  iteration order is implementation-defined; sort features by `id`
  before `json.dump` (lines 325-326).

### package.xml / setup.py / setup.cfg

`package.xml` (`src/avros_navigation/package.xml:1-26`) - format 3,
ament_python, MIT license. Declares `<depend>` on `rclpy`,
`sensor_msgs`, `nav_msgs`, `geometry_msgs`, `nav2_msgs`,
`robot_localization`, `avros_msgs`. **Every one of those is unused**
- the package contains zero rclpy imports. The actual third-party
deps the script uses (`osmnx`, `networkx`, `pyproj`, `shapely`) are
**undeclared** anywhere in package metadata - they only appear in
the script's docstring at line 25. Note `python3-osmnx` is not a
rosdep key on Humble (osmnx is pip-only); proper fix is to declare
`python3-networkx`, `python3-pyproj`, `python3-shapely` as
`<exec_depend>` and document the osmnx pip step in a
`requirements-graph-tool.txt`. Test deps and build_type are
correctly declared.

`setup.py` (`src/avros_navigation/setup.py:1-29`) -
`find_packages(exclude=['test'])` correct. `data_files=` only
installs `package.xml` and the resource marker; **the
`scripts/generate_graph.py` file is NOT installed** by any glob or
entry_point. The `setup.cfg` declares
`script_dir=$base/lib/avros_navigation` (canonical), but no actual
script ends up there because nothing tells setuptools to put it
there. The script is reachable only by a path-relative invocation
(`python3 src/avros_navigation/scripts/generate_graph.py`); it
cannot be run as `ros2 run avros_navigation generate_graph`.
Acceptable as authoring-time tooling, but undocumented.
`entry_points={'console_scripts': []}` is empty.

`setup.cfg` follows the canonical ROS 2 ament_python pattern
(`standards_ros2_python.md` section 1). Correct as-is.

### tests / __init__ / resource

`avros_navigation/__init__.py` is **empty**. The package has no
Python module surface; everything lives in `scripts/`.
`resource/avros_navigation` is present as the empty ament-index
marker. `test/test_copyright.py` is the upstream-default
**skipped** test (line 20 `@pytest.mark.skip(reason='No copyright
header has been placed...')`); the script does indeed lack a
copyright header so the skip is honest, but `<test_depend>
ament_copyright</test_depend>` then becomes dead weight.
`test_flake8.py` and `test_pep257.py` are unskipped defaults that
will run under `colcon test` against `generate_graph.py` -
likely-passing but unverified here.

**Critical gap: no unit test for `generate_graph.py`.** A 30-line
test mocking `osmnx.graph_from_place` and asserting (a) every Point
has `frame == 'map'`, (b) every MultiLineString has `cost > 0`,
(c) IDs are unique, (d) datum-shift is applied (re-run with a
shifted datum and check coords delta) would catch every schema
breakage before competition. This is the single highest-leverage
missing artifact in the package.

## Schema-correctness analysis (does the output match nav2_route's expectations?)

Empirical sample of node 0: `{"properties": {"id": 0, "frame": "map"},
"geometry": {"type": "Point", "coordinates": [-163.269, -633.825]}}`.
Empirical edge 16651: `{"id": 16651, "startid": 0, "endid": 551,
"cost": 4.72, "metadata": {"speed_limit": 13.41}}`.

| nav2_route field | Emitted? |
|---|---|
| Node `id`, `coordinates`, `frame` (recommended) | yes (`'map'`) |
| Edge `id`, `startid`, `endid`, `cost` | yes |
| Edge `overridable` (recommended) | **no** |
| Edge metadata (optional) | partial (1857/17492 edges) |

Schema is almost fully correct. The missing `overridable` is
recommended-not-required; `GeoJsonGraphFileLoader` will default.
Not a P0.

**Bidirectional edges**: confirmed empirically (17,492 features
over 16,651 nodes). The dedup at lines 207-215 keys on
`(u_seq, v_seq)`, so `(u,v)` and `(v,u)` hash differently and both
directions of two-way OSM roads emit as separate features.

**Coordinate frame**: the script subtracts the datum's UTM
coordinates from each node's projected XY (lines 191-192, 220-221).
This is **the same operation `navsat_transform_node` performs**
when configured with `wait_for_datum: true` (`standards_nav2_
localization.md` section 3). Output and live `map` frame agree to
within local UTM grid convergence (~0.5 deg at CPP, sub-meter
cross-track over a 1 km course). **No P0 coordinate-system mismatch.**

**Magnitude vs CLAUDE.md**: committed file is 16,651 nodes /
17,492 edges, not "52 nodes, 113 edges". `--spacing 5.0` over the
whole campus produces ~300x what CLAUDE.md describes - route_server
load and per-query A* runtime will be substantially larger than
the docs imply. Sanity check on Jetson is urgent.

## Cross-cutting issues

- **Package name misleading.** `package.xml` describes a "Route
  planner" but contains zero runtime code. Rename to
  `avros_route_graph`, or fold actual runtime nav glue (rerouting
  service, graph reload service) in here to justify the dead
  rclpy / nav2_msgs deps.
- **OSMnx is not a competition-day asset.** IGVC AutoNav is a
  roped enclosure on grass with no OSM data. The script supports
  neither hand-authoring from the organizer course map nor
  importing a GPS walk. A CSV-input mode would close the gap in
  one afternoon.
- **No producer/consumer link.** The committed
  `src/avros_bringup/config/cpp_campus_graph.geojson` is the live
  consumer; neither package references this script as its
  producer. Default `--output` writes to CWD, not the install
  tree. CLAUDE.md gives the workaround command.
- **OSM data not pinned.** Without `ox.graph_from_xml` + cached
  `.osm.pbf`, every regeneration mutates the 16k-feature file
  unpredictably.
- **No runtime regenerate path.** A ROS service to rebuild the
  graph against a polygon parameter would be ~80 LOC.

## Punch list

### P0 - Must fix before competition

1. **No way to author the IGVC course graph.** OSM has no data for the
   IGVC AutoNav venue. Add a CSV-input mode to `generate_graph.py`
   (`--from-csv waypoints.csv` with `id,lat,lon[,neighbor_ids]`
   columns) so a course graph can be hand-authored in 30 minutes
   from the IGVC organizer-provided map or a GPS walk. Without this,
   the route_server cannot be used at the competition (it's tied to
   a CPP-only dataset). `scripts/generate_graph.py:273-310`.

2. **Stale or oversized graph file checked in.** The committed
   `cpp_campus_graph.geojson` is 16,651 nodes / 17,492 edges, not the
   "52 nodes, 113 edges" CLAUDE.md claims. Either regenerate with a
   tighter `--place` (or a `--bbox`) and commit a small graph, or
   update CLAUDE.md. If this file is what `route_server` actually
   loads at competition, expect non-trivial query latency on Jetson
   (Dijkstra/A* over 17k edges per goal). Cross-cuts P0 for compute
   budget.

### P1 - Should fix

3. **No unit test for the schema generator.** Add a 20-line
   fixture mocking osmnx that asserts schema invariants (Point
   `frame == 'map'`, every edge has `cost`, IDs unique, datum
   offset applied). Single highest-leverage missing artifact.
4. **Undeclared third-party Python deps + dead ROS deps.**
   `package.xml:10-16` declares 7 unused ROS deps; `osmnx`,
   `networkx`, `pyproj`, `shapely` are undeclared. Replace ROS
   deps with `<exec_depend>python3-{networkx,pyproj,shapely}</exec_depend>`
   and add `requirements-graph-tool.txt` for pip-only osmnx.
5. **Script not installed.** `setup.py:9-13` doesn't include
   `glob('scripts/*.py')` or an entry_point. Either add a
   `data_files` glob or a console script so it's runnable via
   `ros2 run avros_navigation generate_graph`.
6. **OSMnx version not pinned** (lines 33). `osmnx>=2.0` broke
   many APIs; pin `osmnx<2,>=1.6` or migrate.
7. **Datum vs UTM-grid-north skew.** Rotate UTM -> local ENU about
   the datum at lines 191-192 / 220-221 to align with what
   `navsat_transform_node` publishes (with `magnetic_declination_radians`).
   Sub-meter for CPP, larger at higher latitudes.
8. **No `overridable` on edges** (lines 225-240). Set to `True`
   so scorer plugins (especially `CostmapScorer`) can dynamically
   modify edge cost.
9. **Empty osmnx query crashes** at line 89 (`max(G_proj.nodes)`).
   Wrap line 60 + line 89 in try/except, print friendly error.
10. **Datum default not validated against `navsat.yaml`** (lines
    40-41). `yaml.safe_load` and assert at startup.
11. **Copyright skip is dead code.** Either add license headers to
    `generate_graph.py:1` and unskip `test_copyright.py:20`, or drop
    `ament_copyright` from `<test_depend>`.

### P2 - Nice-to-have

12. **Locale-dependent `date_generated`** (line 266). Use ISO-8601 UTC.
13. **Misleading EPSG header** (lines 164-169, 259-265). Coordinates
    are datum-shifted, not raw EPSG; rename or drop the `crs` block.
14. **`int()` on UTM zone** (line 164) - use `math.floor`.
15. **`street_count=2` on intermediate nodes** (line 131) is
    misleading OSMnx metadata. Drop or add a comment.
16. **Output JSON not git-stable** across re-runs (lines 325-326).
    Sort features by `id` before `json.dump`.
17. **No `--cache-dir` / `--bbox` / `--polygon` flags.** OSMnx
    `ox.settings.use_cache=True` would make builds reproducible.
18. **Empty `avros_navigation/__init__.py`** is harmless but odd
    given there's no library surface. Drop the package dir + the
    `find_packages` call, or document the placeholder.

## Positives

- **GeoJSON schema is correct.** Point with `frame: 'map'`,
  MultiLineString with `id / startid / endid / cost` - matches
  `nav2_route`'s `GeoJsonGraphFileLoader` per
  `standards_nav2_localization.md` section 7. No P0 schema bug.
- **Datum offset applied after UTM projection** (lines 191-192,
  220-221), producing map-frame meters from datum that match
  `navsat_transform_node`'s output - the exact coordinate-system
  trap the task brief warned about, and it is avoided.
- **Densification follows road curves** rather than chord-cutting
  them - uses Shapely's `LineString.interpolate(dist)` (lines 105,
  126) along OSM geometry, and short edges are kept as-is when
  shorter than `1.5 * spacing` (line 108). This is the same
  approach as the original `AV2.1-API/planning/navigator.py`.
- **Bidirectional edges preserved** - both directions of two-way
  OSM roads emit as separate features with swapped
  `startid`/`endid`. Verified empirically.
- **`mph -> m/s` unit conversion is correct** (constant `0.44704`,
  line 249).
- **Idempotent pipeline** - builds a new graph rather than
  mutating the input.
- **Boilerplate is canonical**: `setup.cfg`, format-3
  `package.xml`, `resource/` marker, three lint tests
  (`standards_ros2_python.md` section 1+5). Script has proper
  shebang and executable bit.
