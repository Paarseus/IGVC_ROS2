# Cross-cutting: Build System & Workspace Layout — Review

## Summary

The IGVC_ROS2 workspace is one ament_cmake interface package (`avros_msgs`) plus six
ament_python packages (`avros_bringup`, `avros_control`, `avros_webui`, `avros_navigation`,
`avros_perception`, `avros_sim`), with five upstream-dependency clones managed via a single
`avros.repos` vcstool manifest (`/home/mspacman/IGVC_ROS2/avros.repos:1-32`). All seven
packages declare `<package format="3">` and the build types match REP 149 expectations.
Resource markers and `setup.cfg` files are present in every ament_python package.
`avros_msgs/CMakeLists.txt` declares `find_package(std_msgs REQUIRED)` plus
`rosidl_generate_interfaces(... DEPENDENCIES std_msgs)`, which gives colcon enough
dependency information to schedule it before everything else. So the *skeleton* of the
workspace is correct.

What is not correct is everything that lives between the skeleton bones. There are at
least eight P0 issues that will bite a fresh checkout on a fresh machine:

1. `scripts/deploy.sh:11` deploys to `~/AVROS` on the Jetson, the deprecated path
   explicitly forbidden by CLAUDE.md ("the live Jetson workspace lives at `~/IGVC/`. ...
   `~/AVROS/` directory ... do **not** use it").
2. `requirements.txt` carries seven pip-only deps (osmnx, networkx, pyproj, shapely,
   fastapi, uvicorn, websockets) that are not declared as `<exec_depend>` in any
   `package.xml`, so `rosdep install --from-paths src` cannot satisfy them. There is no
   automated step that guarantees `pip install -r requirements.txt` happens before
   `colcon build`. (Cross-references: `package_avros_navigation.md` P1 #4, `package_avros_webui.md` P1 #6.)
3. The kiwicampus `semantic_segmentation_layer` is referenced everywhere (CLAUDE.md
   "Known Issues" table, `nav2_params_humble.yaml` plugin string, perception node
   integration tests, four-patch stack) but is **not** in any package's
   `<exec_depend>` block. `rosdep` will not install it; it must be cloned by `vcs import`
   first.
4. CLAUDE.md instructs operators to run `scripts/apply_kiwicampus_patches.sh` after
   `vcs import` to apply four stacked patches; that script does not exist
   (`/home/mspacman/IGVC_ROS2/scripts/` has only `deploy.sh` + `diagnose_sim.py`). The
   four patches are now baked into the `Paarseus/semantic_segmentation_layer` fork on
   the `avros-fixes` branch (`avros.repos:28-31`), so the doc is stale, but a new
   contributor following CLAUDE.md verbatim will hit a missing-file error.
5. `avros_bringup/package.xml` does not declare `<exec_depend>` on six of the upstream
   packages it actually launches: `xsens_mti_ros2_driver`, `zed_wrapper`, `nav2_route`,
   `nav2_lifecycle_manager`, `kiwicampus/semantic_segmentation_layer`,
   `avros_perception` (the bringup launch's `navigation.launch.py` includes perception
   conditionally). `rosdep install` therefore silently leaves these holes; `ros2 launch`
   then fails at runtime with "package not found." (Cross-reference: `package_avros_bringup.md`
   line 36-43.)
6. `avros_control/package.xml` declares `<depend>python3-serial</depend>` AND
   `avros_control/setup.py:14` declares `install_requires=['setuptools', 'pyserial']`.
   The two are reconciled today (apt and pip both install the same module under the
   same import name), but the second-source pattern is footgun-prone — bumping pyserial
   in one place doesn't bump it in the other.
7. `avros_navigation/package.xml:10-16` declares seven `<depend>` entries
   (`rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `nav2_msgs`,
   `robot_localization`, `avros_msgs`) for a package that imports **none** of them —
   the package contains zero rclpy code; only `scripts/generate_graph.py` which uses
   pyproj/shapely/networkx. The four pip-only deps that the script *does* use are not
   declared anywhere except `requirements.txt`. (Cross-reference:
   `package_avros_navigation.md` lines 109-121.)
8. `avros_msgs/CMakeLists.txt:16` lists `srv/PlanRoute.srv` in
   `rosidl_generate_interfaces` but the entire workspace has zero references to it —
   it is dead IDL code that forces every dependent package to rebuild on unrelated
   changes. (Cross-reference: `package_avros_msgs.md` lines 21, 91-102.)

Less severe but pervasive: every package version is `0.0.0`, all share the
`avlab@cpp.edu` maintainer, six of seven use the `<depend>` shorthand where
`<exec_depend>` is preferred for ament_python, three packages lack a LICENSE file
despite manifest claim of MIT (avros_webui, avros_perception, avros_sim), no
top-level repo `LICENSE`, no CI infrastructure, and the `numpy<2` pin from
CLAUDE.md is documented in prose nowhere encoded.

## Inter-package dependency map

Reconciles "manifest" vs "Python imports" vs "launch-file invocations" for each package.
Discrepancies are either (a) missing `<exec_depend>` (rosdep leaves hole, runtime fail) or
(b) dead `<depend>` (build bloat).

**`avros_msgs` (ament_cmake).** `package.xml:13` declares `<depend>std_msgs</depend>`;
`member_of_group` and `<exec_depend>rosidl_default_runtime</exec_depend>` correctly split
per REP 149. CMakeLists' `find_package(std_msgs REQUIRED)` + `rosidl_generate_interfaces(...
DEPENDENCIES std_msgs)` close the loop. **No drift.** Only build-system gripe is dead
`srv/PlanRoute.srv`.

**`avros_bringup` (ament_python launch metapackage).** Declares 5 `<depend>` and 10
`<exec_depend>`. No rclpy imports — should be all `<exec_depend>`. Holes vs what's
launched:

| Process / executable launched | Source package | Declared in bringup `package.xml`? |
|---|---|---|
| `velodyne_driver_node` / `velodyne_transform_node` | `velodyne_driver`, `velodyne_pointcloud` | yes (lines 20-21) |
| `realsense2_camera_node` | `realsense2_camera` | yes (line 22) |
| `xsens_mti_node` | `xsens_mti_ros2_driver` | **no** (P0) |
| `ntrip_client` | `ntrip` | yes (line 23) |
| `zed_camera.launch.py` | `zed_wrapper` | **no** (P0) |
| `actuator_node` | `avros_control` | yes (line 16) |
| `webui_node` | `avros_webui` | yes (line 25) |
| `perception_node` (conditional) | `avros_perception` | **no** (P0) |
| `controller_server`, `planner_server`, `behavior_server`, `smoother_server`, `bt_navigator`, `velocity_smoother` | `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_smoother`, `nav2_bt_navigator`, `nav2_velocity_smoother` | partial — `nav2_bringup` is declared (line 19), but nav2_bringup is a launch metapackage; the individual servers should be listed if you want strict rosdep coverage |
| `route_server` | `nav2_route` | **no** (P0) |
| `lifecycle_manager` | `nav2_lifecycle_manager` | **no** (P0) |
| `static_transform_publisher` | `tf2_ros` | **no** (P1) |
| `foxglove_bridge` | `foxglove_bridge` | **no** (P1) |
| `teleop_twist_keyboard` | `teleop_twist_keyboard` | yes (line 24) |
| `semantic_segmentation_layer` (Nav2 plugin) | `kiwicampus/semantic_segmentation_layer` | **no** (P0) |
| `robot_localization` (EKFs + navsat) | `robot_localization` | yes (line 18) |

"yes" rows correct; "no" rows leave `rosdep install` holes → runtime failures.

**`avros_control`.** Imports `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`,
`avros_msgs`, `serial`. Manifest declares all six. `setup.py:14` *also* declares
`install_requires=['setuptools', 'pyserial']` — duplicates `<depend>python3-serial</depend>`.
Pick one (rosdep entry preferred). **P2.**

**`avros_webui`.** Imports `rclpy`, `avros_msgs`, `fastapi`, `uvicorn`, `websockets`.
Manifest declares the first two, plus an unused `<depend>std_msgs</depend>`, and zero
pip deps. Humble has `python3-fastapi`/`python3-uvicorn`/`python3-websockets` rosdep
keys — declaring them as `<exec_depend>` would let rosdep handle them. CLAUDE.md
currently tells operators to `pip install` manually. **Missing pip exec_depends, P0.**

**`avros_navigation`.** Worst manifest hygiene in the workspace. `package.xml:10-16`
declares seven runtime deps (rclpy/sensor_msgs/nav_msgs/geometry_msgs/nav2_msgs/
robot_localization/avros_msgs). Empty `__init__.py`; only `scripts/generate_graph.py`
exists, which imports osmnx/networkx/pyproj/shapely — **zero ROS deps**. All seven
declared `<depend>` are dead; the actual pip deps are undeclared. **All wrong-direction, P1.**

**`avros_perception`.** Imports rclpy/sensor_msgs/vision_msgs/std_msgs/cv_bridge/
message_filters/numpy/cv2/yaml/ament_index_python. Manifest declares the first nine;
`ament_index_python` is transitive but should be explicit. Missing
`<test_depend>launch_pytest</test_depend>` despite test importing it. Closest to
standards-correct. **Minor drift, P2.**

**`avros_sim`.** Declares `<depend>rclpy/geometry_msgs/webots_ros2_driver>` and
`<exec_depend>` on six packages. Driver uses `tf2_ros`, `nav_msgs`, `sensor_msgs` —
none declared. Sim launch xacro-includes `zed_wrapper/urdf/zed_macro.urdf.xacro` →
`zed_wrapper` should be `<exec_depend>`. Currently transitively present; one upstream
removal from breaking. **Missing transitives, P1.**

**Summary of dead and missing exec_deps:**

| Package | Dead exec_depends to remove | Missing exec_depends to add |
|---|---|---|
| avros_msgs | (none) | (none) |
| avros_bringup | (none) | xsens_mti_ros2_driver, zed_wrapper, avros_perception, nav2_route, nav2_lifecycle_manager, semantic_segmentation_layer, tf2_ros, foxglove_bridge |
| avros_control | (none) | (none — pyserial duplication is hygiene) |
| avros_webui | std_msgs | python3-fastapi, python3-uvicorn, python3-websockets |
| avros_navigation | rclpy, sensor_msgs, nav_msgs, geometry_msgs, nav2_msgs, robot_localization, avros_msgs | python3-networkx, python3-pyproj, python3-shapely (osmnx is pip-only) |
| avros_perception | (none) | launch_pytest (test_depend) |
| avros_sim | (none) | tf2_ros, nav_msgs, sensor_msgs, zed_wrapper |

## Workspace meta

### avros.repos

`avros.repos:1-32` pins five repositories: `realsense-ros@4.56.4` (upstream tag),
`xsens_mti` (Paarseus fork, branch `ros2`), `zed-ros2-wrapper@v5.2.2` (upstream tag,
with a `# TODO(jetson)` comment about SDK-major.minor matching),
`semantic_segmentation_layer` (Paarseus fork, branch `avros-fixes`).

Two notes. The `semantic_segmentation_layer` pin (lines 17-31) is **gold-standard
fork-pinning** — header comment lists the four stacked patches, names which are merged
upstream (PR #1, PR #5) and which are open, cross-references
`docs/CHANGELOG_2026-04-29.md`. Future maintainers can audit deltas without `git log`.

In contrast, the `xsens_mti` Paarseus-fork pin (lines 6-9) has zero explanation of why
a fork is required vs upstream — no patch list, no comment. Rebasing onto upstream is
risky because no one knows what's preserved. **P1 hygiene** — copy the
`semantic_segmentation_layer` comment style.

**Doc drift:** CLAUDE.md "Build & Test" tells operators to run
`scripts/apply_kiwicampus_patches.sh` after `vcs import`. That script does not exist
(`/home/mspacman/IGVC_ROS2/scripts/` has only `deploy.sh` + `diagnose_sim.py`). The
patch-stacking moved into the fork; CLAUDE.md was not updated. **P0 doc fix** —
either create a stub script or remove the reference.

### .gitignore

10 lines. Standard ROS 2 / colcon ignores (`build/`, `install/`, `log/`, `__pycache__/`,
`*.pyc`, `*.egg-info/`, `.colcon/`) plus four `src/<pkg>/` paths for the vcs-imported
trees. **Correct minimal pattern** — the `src/<pkg>/` lines mean a contributor who
runs `vcs import` does not accidentally commit ~700 MB of upstream code into our
repo. Also nothing pyc-y or build-y can sneak in.

What's missing: `*.swp`, `.DS_Store`, `.idea/`, `.vscode/`, `*.bak`, `*~` editor/OS
turds; the perception package's `.pytest_cache/` (already exists at
`src/avros_perception/.pytest_cache/` per `ls`); coverage reports
(`.coverage`, `htmlcov/`); any `*.csv` produced by firmware bring-up scripts
(per `package_firmware.md` cross-cutting issues — "Bring-up CSVs may not be in
.gitignore"). **P2 hygiene.**

### requirements.txt

7 lines, all pip-only deps for two packages: `osmnx`/`networkx`/`pyproj`/`shapely`
(navigation graph tool) and `fastapi`/`uvicorn`/`websockets` (webui). Three concerns:

1. **Of these, six have rosdep keys on Humble** (`python3-networkx`,
   `python3-pyproj`, `python3-shapely`, `python3-fastapi`, `python3-uvicorn`,
   `python3-websockets`). Should be `<exec_depend>` in the respective package.xml
   files, not pip-only. Only osmnx is genuinely pip-only on Humble — that one belongs
   in a `requirements-graph-tool.txt`. **P0.**
2. **Floors only, no caps.** `osmnx>=1.3.0` will pull osmnx 2.x which broke many APIs
   (cross-ref `package_avros_navigation.md` lines 67-71). CLAUDE.md's `numpy<2`
   constraint is documented but encoded nowhere. Add `osmnx<2,>=1.6`, `numpy<2,>=1.21`.
   **P0 reproducibility.**
3. **Single root-level file** rather than per-package requirements files or
   `extras_require=` integration. Minor — webui-only contributors install navigation
   deps unnecessarily. **P2.**

### scripts/deploy.sh

61 lines bash. Three P0-shaped issues:

1. **Hardcoded `~/AVROS`** (line 11) — deprecated path per CLAUDE.md ("live workspace
   is `~/IGVC/`"). Script last touched April 12; migration happened later. Every
   field-deploy overwrites the wrong tree. **P0.**
2. **`colcon build ... 2>&1 | tail -5`** (line 44) masks exit code. With `set -e` but
   no `set -o pipefail`, failed builds are treated as success and the launch step
   runs against stale `install/`. **P0 reliability.**
3. **Selective build of 3 packages only** (line 44 `--packages-select avros_bringup
   avros_control avros_webui`); changes to msgs/perception/navigation/sim go silently
   un-rebuilt. **P1.**

P2 issues: kill-everything-with-"ros"-in-cmdline (line 15) is footgun on shared boxes;
SSH session blocks the launched stack with no detach. **Positive:** correctly exports
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI` (lines 49, 52, 55),
addressing CLAUDE.md's known FastDDS/CycloneDDS interop bug.

### scripts/diagnose_sim.py

170 lines, Python 3, ROS 2. Subscribes to 6 topics + a TF lookup, prints a structured
report every 2 s. The script is at workspace root, not inside `avros_sim/`, so it is
not installed by any setup.py and is not reachable via `ros2 run`. The docstring
(line 12) advertises `ros2 run avros_sim diagnose_sim` which **does not work**
(cross-reference `package_avros_sim.md` lines 79-87, P1 #14). Standalone invocation
via `python3 scripts/diagnose_sim.py` does work.

The script itself is fine — it follows the standards-doc Node skeleton (Node subclass,
`try/finally` cleanup, `BEST_EFFORT` QoS for sensor streams). The only code-level
concern is the bare `except Exception` on the tf2 lookup at line 135 (should catch the
three tf2 exceptions specifically per `standards_ros2_python.md` §6). Workspace-level
concern: this is the only file at `scripts/` other than `deploy.sh`, and it's not even
referenced by `setup.py`/`package.xml`. Either move into `avros_sim/scripts/` and add
to `setup.py` `data_files`/`entry_points`, or keep it at root and document it as a
standalone diagnostic. **P1 — pick a story and stick to it** (cross-ref
`package_avros_sim.md` cross-cutting issue "Documentation drift").

## package.xml / setup.py drift

Side-by-side of the seven manifests:

| Field | msgs | bringup | control | webui | nav | perception | sim |
|---|---|---|---|---|---|---|---|
| format | 3 | 3 | 3 | 3 | 3 | 3 | 3 |
| version | 0.0.0 | 0.0.0 | 0.0.0 | 0.0.0 | 0.0.0 | 0.0.0 | 0.0.0 |
| license | MIT | MIT | MIT | MIT | MIT | MIT | MIT |
| maintainer email | avlab@cpp.edu (×7) ||||||| 
| LICENSE at pkg root | yes | yes | yes | **no** | yes | **no** | **no** |
| build_type | ament_cmake | ament_python (×6) ||||||
| Resource marker / setup.cfg | n/a | yes | yes | yes | yes | yes | yes |
| `<depend>` shorthand | yes | yes | yes | yes | yes | yes | mixed |
| `entry_points` | n/a | empty | actuator_node | webui_node | empty | perception_node | empty |
| `install_requires` | n/a | st | st, **pyserial dup** | st | st | st | st |

**Cross-cutting findings.**

1. **Every version is `0.0.0`** — uniformly applied scaffold marker. Bump to `0.1.0`.
   **P2.**
2. **All seven share `avlab@cpp.edu`** maintainer email; verify reachability. **P2.**
3. **Three packages missing `LICENSE` at package root** despite manifest claim of MIT
   (`avros_webui`, `avros_perception`, `avros_sim`); no top-level `LICENSE` either.
   `ros2 pkg license` returns claim with no text; ament_copyright skips silently. **P1.**
4. **Six of seven packages use `<depend>` shorthand** for runtime-only deps where the
   standards doc §1 prefers `<exec_depend>` for ament_python. `<depend>` expands to
   `build + build_export + exec`, the first two meaningless for pure Python. Not a
   defect today; pervasive. **P2.**
5. **`setup.cfg` is uniform** across all six ament_python packages (`script_dir =
   $base/lib/<pkg>`). **Positive.**
6. **`avros_bringup/setup.py:17` uses exotic glob** `*launch.[pxy][yma]*` which works
   today but is footgun-prone (a `myimage.pyma` would install as a launch file).
   Tighten to `*.launch.py`. Same pattern in `avros_sim/setup.py:17`. **P2.**
7. **`avros_bringup/setup.py:19,21,23` use broad `glob('config/*')`, `glob('urdf/*')`,
   `glob('rviz/*')`** — picks up editor swap files, backups, subdirectories. **P2.**
8. **`avros_navigation/setup.py:9-13` does NOT install `scripts/generate_graph.py`.**
   `find_packages` finds the empty Python package; the script lives in `scripts/` and
   is referenced in neither `data_files` nor `entry_points`. With `--symlink-install`
   this is workable; non-symlink install drops the script silently. **P1.**
9. **`avros_control/setup.py:14`** declares `install_requires=['setuptools',
   'pyserial']` duplicating `<depend>python3-serial</depend>`. Pick one source of
   truth. **P2.**
10. **`avros_perception` is the cleanest** of the seven — manifest matches imports,
    setup.py installs everything correctly. Use as the model for the other packages.
    **Positive.**
11. **`avros_msgs/package.xml:13` uses `<depend>std_msgs</depend>`** instead of the
    `<build_depend>` + `<exec_depend>` pair conventional for IDL packages
    (`common_interfaces`, `nav2_msgs`). Works today; not idiomatic. **P2.**
12. **No package declares `<author>`** (REP 149 §2.4 optional but recommended). **P2.**

## Build-order + symlink-install verification

CLAUDE.md prescribes a two-step build:

```
colcon build --symlink-install --packages-select avros_msgs
colcon build --symlink-install
```

This is **defensive but unnecessary.** Four packages declare `<depend>avros_msgs</depend>`
or `<exec_depend>avros_msgs</exec_depend>`: `avros_control` (`package.xml:14`),
`avros_navigation` (`package.xml:16`), `avros_webui` (`package.xml:11`), `avros_sim`
(`package.xml:14`). For ament_python `<depend>` expands to `<build_depend>`, which colcon
uses for scheduling. A clean build correctly prints `Starting >>> avros_msgs` first.
The two-step recipe defensively serializes against a rare race (a Python package starting
its build before message stubs are generated), but the dependency metadata is sufficient.

**Other inter-package build-time risks:**

- `avros_sim/launch/sim.launch.py` xacro-includes `zed_wrapper/urdf/zed_macro.urdf.xacro`
  at launch time (not build time). Missing `<exec_depend>zed_wrapper</exec_depend>` →
  runtime failure on next `ros2 launch`, not build failure. Brittle.
- `avros_perception/test/test_perception_launch.py` imports `launch_pytest` (test_dep
  missing per `package_avros_perception.md` P1 #8) — affects `colcon test`, not build.

**Symlink install:** works for editing existing `.py`/`.yaml`/launch files in place.
**Does not work for newly-added files** because they aren't in the glob's snapshot at
build time. Standard colcon limitation; flag for new contributors.

**Files NOT installed by setup.py:**

- `avros_navigation/scripts/generate_graph.py` — P1 above. Reachable only via
  path-relative invocation.
- `scripts/diagnose_sim.py` — workspace-root, no setup.py references it. Docstring
  advertises `ros2 run avros_sim diagnose_sim` which fails (P1 above).
- `requirements.txt` — pip-only, no colcon integration. Manual `pip install -r` is
  not gated by the build.
- `avros_msgs/srv/PlanRoute.srv` — installed but dead (P0 above).

**Build cache pitfalls.** `build/`/`install/` correctly in `.gitignore`. CLAUDE.md does
not document `colcon clean` (`rm -rf build install log`); after renaming a launch file
the old name persists in `install/` until manually deleted. **P2 — document clean-slate
gesture.**

## CI / version pinning gaps

The workspace has **zero CI infrastructure** — no `.github/workflows/`, no
`.gitlab-ci.yml`, no `Dockerfile`, no `pre-commit` config, no `tox.ini` / `pyproject.toml`,
no `Makefile`. The only "test" is "did `colcon build` succeed on the Jetson when the
developer last ran `./scripts/deploy.sh`." Per-package linter tests
(`test_copyright.py`/`test_flake8.py`/`test_pep257.py`) run only when a contributor
remembers to invoke `colcon test`.

For a competition codebase one month from event day, this is **acceptable risk** — but
it's also the single biggest reason missing `<exec_depend>` drift accumulated unobserved.
**P1 post-competition tracking item: GitHub Actions workflow** that does
`vcs import src < avros.repos` → `pip install -r requirements.txt` → `rosdep install
--from-paths src --ignore-src -y` → `colcon build --symlink-install` →
`colcon test`. Even just the build step would catch every missing-exec_depend in this
report on the next PR.

**Version pinning gaps:**

CLAUDE.md "Known Issues" table (the row for "numpy binary incompatibility on Jetson")
documents the constraint `numpy<2`. This constraint is enforced **nowhere**:

- Not in `requirements.txt` (would be `numpy<2,>=1.21`).
- Not in `avros_perception/setup.py` (`install_requires` doesn't list numpy).
- Not in `avros_perception/package.xml` — `<depend>python3-numpy</depend>` (line 16)
  has no version constraint and rosdep can install whatever apt has, which on Humble
  is numpy 1.21 by default but on a system with custom pip overlay could be numpy 2.x.
- Not in any pip lockfile (there is none).

So if a contributor updates the Jetson's numpy via pip (or another package upgrades
it transitively), the perception node breaks at import time and only CLAUDE.md prose
explains why. **P0 reproducibility** — add `numpy<2` to a constraints file or the
package's `<exec_depend>python3-numpy</exec_depend>` entry (rosdep YAML edit).

Other reproducibility gaps worth flagging:

- **OSMnx version not pinned to `<2`** despite breaking changes (cross-ref
  `package_avros_navigation.md` lines 67-71). `requirements.txt:1` has `osmnx>=1.3.0`
  with no upper bound. **P0 reproducibility for the route-graph regeneration path.**
- **No Jetson-specific `requirements-jetson.txt`** — pip on the Jetson must use
  arm64 wheels, sometimes via `--index-url
  https://pypi.jetson-ai-lab.dev/jp6/cu126`. Documented nowhere. CLAUDE.md
  "Required deps" prose mentions a few apt packages but not the pip wheel resolver
  setup.
- **No SDK version capture for ZED.** `avros.repos:13-16` pins
  `zed-ros2-wrapper@v5.2.2`, requiring SDK 5.2.x at `/usr/local/zed`. The SDK install
  itself is described in CLAUDE.md prose but not encoded as an automated step. A new
  Jetson bring-up requires manual SDK installation, no automation.
- **No firmware version capture.** The Teensy `.ino` file has no `__DATE__ __TIME__`
  banner (cross-ref `package_firmware.md` cross-cutting issues). A field operator with
  a misbehaving chassis cannot determine which firmware revision is loaded without
  reading PID gains and matching to FINDINGS.md. **P2.**
- **No CycloneDDS version constraint.** The behavior depends on which Cyclone is
  installed (apt vs source), and `cyclonedds.xml` may carry version-specific tags.
  Encode as `<exec_depend>` if possible.

**Lockfile pattern entirely absent.** No `Pipfile.lock`, no `poetry.lock`, no
`requirements.lock` (pip-tools output). Every `pip install -r requirements.txt`
resolves dynamically. For a competition codebase one bad pypi metadata change away
from breaking the chassis, this is brittle. **P1 — generate a
`requirements.lock` once and commit it.**

## Consolidated build punch list

### P0 (won't build / fails on fresh checkout)

1. **`scripts/deploy.sh:11` deploys to `~/AVROS`** — deprecated path per CLAUDE.md.
   Update to `~/IGVC`. Every "field deploy" today overwrites the wrong tree.
2. **`scripts/deploy.sh:44` masks `colcon` exit code** via `colcon build ... 2>&1 | tail -5`
   without `set -o pipefail`. Failed builds treated as success; launch step runs
   against stale `install/`. Add `set -o pipefail` + explicit exit-code check.
3. **`avros_bringup/package.xml` missing six `<exec_depend>` entries** for packages it
   launches: `xsens_mti_ros2_driver`, `zed_wrapper`, `nav2_route`,
   `nav2_lifecycle_manager`, `avros_perception`, `semantic_segmentation_layer`. Plus
   `tf2_ros` and `foxglove_bridge` (P1). `rosdep install` leaves these holes; launch
   fails at runtime. (Cross-ref `package_avros_bringup.md` P0 line 36-43.)
4. **`avros_webui` missing pip-deps in `package.xml`** — `python3-fastapi`,
   `python3-uvicorn`, `python3-websockets` (valid Humble rosdep keys). CLAUDE.md tells
   operators to `pip install` manually, no automated check. (Cross-ref
   `package_avros_webui.md` P1 #6.)
5. **`avros_navigation/package.xml` declares 7 dead ROS deps and zero pip deps.**
   Drop `<depend>` for `rclpy/sensor_msgs/nav_msgs/geometry_msgs/nav2_msgs/
   robot_localization/avros_msgs` (none imported). Add `<exec_depend>` for
   `python3-networkx/python3-pyproj/python3-shapely`. osmnx pip-only (no rosdep on
   Humble) → `requirements-graph-tool.txt`. (Cross-ref `package_avros_navigation.md` P1 #4.)
6. **`requirements.txt` versions have floors but no caps.** `osmnx>=1.3.0` pulls
   osmnx 2.x which broke many APIs; CLAUDE.md's `numpy<2` is prose-only. Add
   `osmnx<2,>=1.6` and `numpy<2,>=1.21`.
7. **`scripts/apply_kiwicampus_patches.sh` referenced by CLAUDE.md does not exist.**
   Patches now baked into the fork (`avros.repos:28-31`); update CLAUDE.md.
8. **`avros_msgs/srv/PlanRoute.srv` is dead IDL.** Remove from `CMakeLists.txt:16`
   and delete the file. (Cross-ref `package_avros_msgs.md` P1 line 21.)

### P1 (works on lucky machines, breaks elsewhere)

9. **`avros_navigation/setup.py` does not install `scripts/generate_graph.py`.**
   Add `glob('scripts/*.py')` to `data_files` or an `entry_points` entry. Currently
   reachable only by path-relative invocation. (Cross-ref `package_avros_navigation.md` P1 #5.)
10. **`scripts/diagnose_sim.py` docstring claims `ros2 run avros_sim diagnose_sim`
    works.** It doesn't — file is at workspace root. Move it into `avros_sim/scripts/`
    + setup.py, or fix the docstring. (Cross-ref `package_avros_sim.md` P1 #14.)
11. **`scripts/deploy.sh` selectively builds 3 packages**; doesn't warn when you've
    changed others (e.g. msgs, perception). Use `--packages-up-to` or build all seven.
12. **`avros_perception/package.xml` missing `<test_depend>launch_pytest</test_depend>`.**
    `test_perception_launch.py` imports it; `colcon test` skips silently otherwise.
13. **`avros_sim/package.xml` missing `<exec_depend>` for `tf2_ros`, `nav_msgs`,
    `sensor_msgs`, `zed_wrapper`.** First three imported by sim driver; zed_wrapper
    needed for sim URDF xacro inflation. Transitively present today; brittle.
14. **No CI/CD anywhere.** Drift like the missing exec_deps accumulated because
    nothing checks. Post-competition tracking item.
15. **`avros.repos:6-9` pins `xsens_mti` to a Paarseus fork branch `ros2`** without
    a comment explaining why a fork is needed. Compare to the
    `semantic_segmentation_layer` block which has a detailed patch-list comment.
16. **`requirements.txt` has no lockfile.** Generate `requirements.lock` via
    `pip freeze` or `pip-tools compile` and commit it.
17. **Three packages missing LICENSE file at package root** (avros_webui,
    avros_perception, avros_sim) despite manifest claim of MIT. Either copy
    `avros_msgs/LICENSE` into each, or add a single repo-root LICENSE.
18. **Top-level repo-root `LICENSE` missing.** GitHub will display "no license found."
19. **`avros_control` has duplicated `pyserial` dep** in `package.xml`
    `<depend>python3-serial</depend>` AND `setup.py` `install_requires=[..., 'pyserial']`.
    Pick one source of truth (rosdep entry preferred per convention).
20. **`.gitignore` missing common turds** — `*.swp`, `.DS_Store`, `.idea/`,
    `.vscode/`, `*.bak`, `*~`, `.coverage`, `.pytest_cache/`, firmware-bringup
    `*.csv` files. The `.pytest_cache/` is already in the tree at
    `src/avros_perception/.pytest_cache/`.
21. **No documented `colcon clean` workflow** in CLAUDE.md. After renaming a launch
    file, the old name persists in `install/` until manually deleted. Document
    `rm -rf build install log`.

### P2 (style / hygiene)

22. All seven packages have `<version>0.0.0</version>`. Bump to `0.1.0` post-fixes.
23. All seven share `avlab@cpp.edu` maintainer; verify reachability; add `<author>`
    tags.
24. Six of seven packages use `<depend>` shorthand for runtime-only deps in
    ament_python packages where `<exec_depend>` is preferred (standards §1).
25. `avros_bringup/setup.py` uses exotic glob `*launch.[pxy][yma]*` and broad
    `glob('config/*')`. Tighten to `*.launch.py` and per-extension config globs.
    Same exotic launch glob in `avros_sim/setup.py`.
26. `avros_msgs/CMakeLists.txt:1` uses `cmake_minimum_required(VERSION 3.8)`.
    `3.14` is more idiomatic on Humble for a fresh IDL package.
27. `avros_msgs/CMakeLists.txt:4-6` declares `-Wall -Wextra -Wpedantic` — dead
    noise for an interface-only package.
28. No firmware-version banner (`__DATE__ __TIME__`) on Teensy boot.
29. No SDK install automation for ZED — manual prose only.
30. `avros_navigation/avros_navigation/__init__.py` is empty (no library surface).
    Either delete the Python package directory or document the placeholder.
31. `scripts/deploy.sh` does kill-everything-with-"ros"-in-cmdline — footgun on a
    shared dev machine.
32. `scripts/deploy.sh` blocks the SSH session for the launched stack's duration
    (no detach). Acceptable but undocumented.
33. Four-patch stack on `Paarseus/semantic_segmentation_layer:avros-fixes` is
    fragile. Two patches are merged upstream (PR #1, PR #5); could open patches
    (#2 lock, #4 wall-clock) be proposed upstream so the fork shrinks?

## Positives

- **Every package uses `<package format="3">`** with correct build types (ament_cmake
  for IDL, ament_python for the six others). Resource markers and uniform `setup.cfg`
  files (`script_dir = $base/lib/<pkg>`) exist in every ament_python package.
- **`avros.repos` is well-curated** — five pinned repositories, clear `# TODO(jetson)`
  comment on the ZED pin, and an outstanding patch-list comment block on the
  `semantic_segmentation_layer` fork that names PR numbers and cross-references the
  changelog. Gold-standard fork-pinning for the kiwicampus block.
- **`avros_msgs` build-time deps are correctly wired.** `find_package(std_msgs)` plus
  `rosidl_generate_interfaces(... DEPENDENCIES std_msgs)` give colcon enough information
  to schedule the IDL package first automatically — CLAUDE.md's two-step recipe is
  defensive but unnecessary.
- **`scripts/deploy.sh` correctly exports `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI`**
  on every launch (lines 49, 52, 55), addressing the known FastDDS/CycloneDDS interop
  bug from CLAUDE.md.
- **`.gitignore` excludes the four upstream-cloned source trees**, preventing accidental
  commits of hundreds of MB of upstream code.
- **`avros_perception/setup.py` is the model setup.py for the workspace** — specific
  globs (`glob('config/*.yaml')`, `glob('launch/*.launch.py')`), correct entry_points,
  manifest matches imports. The other packages should copy this pattern.
- **The exec_depends that ARE present are mostly correct.** Velodyne, RealSense, ntrip,
  robot_localization, nav2_bringup, teleop, and the inter-package avros_* deps all map
  to real packages. The gaps documented above are **omissions**, not wrong entries —
  easier to fix.
- **`avros_msgs` is correctly consumed by every dependent package** — `avros_control`,
  `avros_webui`, `avros_navigation`, `avros_sim` all declare it, and downstream Python
  imports (`from avros_msgs.msg import ActuatorCommand, ActuatorState`) resolve.
- **Firmware tree is build-system-isolated** — `firmware/` is Arduino C++ + standalone
  Python scripts; it does not interfere with colcon's view of the workspace.
- **Symlink-install works for the steady-state edit-and-rerun loop.** Editing existing
  Python / YAML / launch files does not require a rebuild.
