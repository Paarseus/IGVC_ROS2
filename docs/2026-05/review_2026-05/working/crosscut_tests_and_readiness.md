# Cross-cutting: Test Coverage & IGVC Readiness — Review

## Part 1: Test coverage

### Test inventory by package

The workspace has nine packages plus the `firmware/` tree. One package (`avros_perception`)
carries real behavioral tests; every other package has either three boilerplate ament-
linter files or nothing at all. There is no `.github/workflows/` directory and no CI
runner of any kind, so even the boilerplate linters only run when a developer manually
executes `colcon test`.

| Package | `test/` contents | Real behavioral tests | Source LOC | Test LOC | Coverage assessment |
|---|---|---|---|---|---|
| `avros_msgs` | (no `test/` directory) | 0 | 0 (msg/srv only) | 0 | N/A — message package, no Python code to test. |
| `avros_bringup` | 3 boilerplate (copyright/flake8/pep257) | 0 | 1167 | 73 | None. 9 launch files, 16 YAMLs, 16 651-node GeoJSON — entirely uncovered by `launch_testing` smoke checks. |
| `avros_control` | 3 boilerplate | 0 | 456 | 73 | **Worst gap in the workspace.** A 426-line safety-critical control node with diff-drive inverse, slew limiter, IMU heading-hold, and watchdog logic — zero behavioral assertions. `test_copyright.py` is `pytest.mark.skip`. |
| `avros_navigation` | 3 boilerplate | 0 | 360 | 73 | The 332-LOC `generate_graph.py` (offline OSMnx → GeoJSON producer) has no schema test. `__init__.py` is empty so `find_packages` finds nothing for the linters to grade. |
| `avros_perception` | 3 boilerplate + 8 unit + 1 launch | 9 | 727 | 1018 | Workspace high-water mark. Test:source ratio > 1.0; pioneers `launch_pytest`, mock `ApproximateTimeSynchronizer`, SHA-256 threshold-drift guard. |
| `avros_webui` | 3 boilerplate | 0 | 246 | 29 | The disconnect → e-stop safety contract — webui's single most important property — is untested. FastAPI `TestClient` + WebSocket close + assert publish-estop is ~20 LOC. |
| `avros_sim` | (no `test/` directory) | 0 | 480 | 0 | No tests at all. `<test_depend>` lines exist in `package.xml` but no test directory; `colcon test` finds nothing to run. |
| `firmware/` | (no test infrastructure) | 0 | ~2 280 (mostly Python bring-up) | 0 | No `arduino-cli compile` CI hook. The 9 phase scripts run successfully but only print — they do not assert. The bring-up CSV baselines are not loaded by any regression script. |

**Workspace totals.** ~3 436 LOC of source code (Python; Arduino C++ excluded) against
~1 264 LOC of test code, of which 1 018 LOC (81%) live in a single package. Subtracting
boilerplate, the workspace has 1 018 LOC of real behavioral test code total, all of it
covering one node in one package.

The `<test_depend>` block in `package.xml` is uniformly correct — every package declares
`ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`. `avros_perception`
should additionally declare `launch_pytest` (its only existing integration test imports
that module); see the `avros_perception` per-package report.

### avros_perception as template

`avros_perception/test/` is the only example of modern ROS 2 Humble Python testing in
this workspace. Six patterns are directly copyable to every other package:

1. **`conftest.py` for determinism** (`test/conftest.py:17-20`). Pins
   `cv2.setNumThreads(1)` and `np.random.seed(0)` before any test runs, eliminating
   a class of flake; the `load_image` fixture skips on missing files so the field
   corpus can land later. Every package should adopt this.
2. **`launch_pytest` integration test** (`test/launch/test_perception_launch.py`).
   Spawns the real node in a subprocess, late-joins a `_TopicWatcher` with the exact
   `transient_local + reliable` QoS the production subscriber uses, asserts the
   latched message arrives within 10 s. Every node should have an equivalent smoke
   check; for `avros_control` this would be ~50 LOC against a `socat`-created PTY.
3. **SHA-256 threshold-drift guard** (`test/unit/test_hsv_thresholds.py`, 78 LOC).
   Hashes the HSV tuning tuple and asserts against a checked-in `EXPECTED_HASH`.
   Forces every threshold change through PR review. Apply the same pattern to
   `actuator_params.yaml` (PID gains, slew caps) and the costmap inflation YAMLs —
   PR-gated parameter governance the rest of the workspace lacks.
4. **Mock `ApproximateTimeSynchronizer` tests** (`test/unit/test_sync_slop.py`, 149
   LOC). Pure-Python mock subscribers push synthetic stamped messages through the
   real `message_filters` synchronizer; six cases cover within/beyond/replacement/
   order/boundary slop behavior. No ROS process required. Reusable for any node
   using time-sync.
5. **Live-tuning callback test** (`test/unit/test_hsv_live_tuning.py`, 115 LOC).
   Proves the shared mutable params dict + atomic validate-then-mutate pattern —
   the only place in the workspace this canonical Humble pattern is actually
   exercised. Becomes the test template once `actuator_node` adds its missing
   `add_on_set_parameters_callback`.
6. **Behavior tests on synthetic numpy frames** (`test/unit/test_hsv_pipeline.py`,
   11 cases). Hand-crafted frames where the correct answer is known by
   construction. Eliminates the fixture-corpus dependency. The same pattern works
   for `avros_control` (synthetic Twist+Imu → `_target_v`/`_slew_v`),
   `avros_navigation` (monkey-patched `osmnx` → schema invariants),
   `avros_webui` (FastAPI TestClient + synthetic joystick JSON).

A single afternoon per package gets `conftest.py` + 5–10 unit tests. Every other
Python package can crib `conftest.py` verbatim.

### Test gaps by severity

The most consequential gap is `avros_control`. 426 LOC of safety-critical Python code
controls a 1.5 m/s tracked vehicle with zero behavioral assertions. The Phase 2 report
calls out 14 failure modes in its safety table; none are guarded. The minimum test
set is pure-function and writable in ~120 LOC of one file (`test_kinematics.py`):
three `yaw_from_quaternion` cases, three `wrap_angle` cases, two diff-drive inverse
cases (pure translation, pure rotation), four slew-limiter cases (accel-clamped,
decel-toward-zero, sign-change, at-target), and two midpoint-pose cases. Per Phase 2
the slew limiter has subtle correctness ("it took three reads to convince myself")
and the midpoint integrator was a regression fix in CHANGELOG_2026-04-28 — no
regression guard exists. **This is the single highest-leverage missing artifact in
the workspace.**

`firmware/teensy_diff_drive.ino` is Arduino C++ — no rclpy. Three minimum hooks: an
`arduino-cli compile` invocation (catches syntax errors before they ship); a
`dump_progmem.py` helper that reports flash+RAM and compares against last commit
(catches accidental `String` use); a regression runner that loads Phase 4 / Phase 6c
CSV baselines and asserts max-RPM and PID tracking within ±5 %. The bring-up scripts
already produce the data; they just don't load it back. This is the "calibration
drift detection" pattern from `standards_firmware_safety.md` § 8.

`avros_navigation/scripts/generate_graph.py` needs a 30-LOC schema test that
monkey-patches `osmnx.graph_from_place` to a 4-node graph and asserts every Point has
`frame == 'map'`, every edge has `cost > 0`, IDs are unique, and datum shift moves
coordinates by the expected delta. Per the package review, "the single highest-
leverage missing artifact in the package."

`avros_bringup` has nine launch files and no smoke checks. A single ~30-LOC
`launch_testing` test for `sensors.launch.py` (all `enable_*=false`, assert
`/robot_state_publisher` and `/tf_static` come up) catches most launch-file
regressions: missing data_files glob, broken `LaunchConfiguration` interpolation,
`xacro` failure on a clean install.

`avros_sim` has no `test/` directory at all; `colcon test` is silently a no-op.
Adding the three boilerplate linters is the minimum to catch typos in the ~535 LOC
of sim code.

`avros_webui` has 29 LOC of test (boilerplate only). The single most valuable test
is "WebSocket disconnect publishes e-stop" (~20 LOC with FastAPI's `TestClient`).
The webui's *only* safety property is currently untested.

### Boilerplate cost-benefit

Every package except `avros_msgs` and `avros_sim` carries the canonical three-file
ament boilerplate. The verdict:

- `test_copyright.py` is universally `pytest.mark.skip` (same skip-comment in every
  package — source files lack headers). `<test_depend>ament_copyright</test_depend>`
  is dead weight in every manifest. Fix is either `ament_copyright --add-missing 'AV Lab'
  apache2` once and remove the skip, or drop the dep entirely. The current "declare
  the dep, skip the test" state is the worst of both worlds.
- `test_flake8.py` runs but checks PEP-8 only. Catches missing imports, bad
  whitespace, line-length, unused variables — no behavioral bug, but enough
  churn-shaped regressions to justify keeping.
- `test_pep257.py` is the lowest-value of the three; docstring style on self-
  contained ROS nodes pays minimal dividends.

**The bigger cost is false confidence.** Six packages have green linter suites and
zero behavioral coverage. A developer running `colcon test` and seeing all-green
assumes there's real coverage; there isn't. Recommendation: keep the linters as
scaffolding, fix the copyright headers in one shot, then land the behavioral tests
in the punch list below. For `avros_sim` (no test directory), add the boilerplate so
`colcon test` isn't silently a no-op.

### CI / automation gap

**No `.github/workflows/` directory exists.** No GitHub Actions, no GitLab CI, no
Jenkins. `git push` triggers nothing. Broken code lands on `main` silently until the
next manual `colcon build` on the Jetson — gap on the order of days to weeks.

The 2026-04-29 changelog captures one such silent regression: the kiwicampus PR3
patch migration to `Paarseus/avros-fixes` required a manual `vcs import` step;
anyone who didn't pull the new `avros.repos` got a build failure with no root-cause
signal.

Minimum viable CI for IGVC competition (~half a day's work):

1. **GitHub Action: `colcon build` on every PR.** ~30 LOC of YAML using
   `osrf/ros:humble-desktop`. Catches missing `<exec_depend>`, broken `setup.py`
   globs, syntax errors, `xacro` failure.
2. **GitHub Action: `colcon test`.** Runs after build. Today catches near-zero
   regressions because tests are concentrated in `avros_perception`; becomes
   valuable as the punch list lands.
3. **GitHub Action: `arduino-cli compile`** for the firmware. ~10 LOC. Prevents
   the next `KP<garbage>`-class parse bug from shipping. The Teensy sketch has
   never been built outside the developer's laptop.
4. **Pre-commit hook for the SHA-256 hash guard.** ~5 LOC; refuses commits that
   change HSV thresholds without bumping `EXPECTED_HASH`.

No launch file is structured for headless smoke testing; the closest is
`scripts/diagnose_sim.py` (human-driven, reports to stdout). A field-grade E2E
test would replay a recorded ROS bag through `navigation.launch.py` and assert
`/odometry/filtered` reaches a target pose — ~200 LOC plus a fixture bag, P2
unless the team gets ahead of schedule.

### Test-coverage punch list

#### P0 (would mask a competition-blocking bug)

1. **`avros_control/test/test_kinematics.py`** — pure-function regression suite,
   ~120 LOC: `yaw_from_quaternion` (3 cases), `wrap_angle` (3), diff-drive inverse
   (2), asymmetric slew limiter (4), midpoint-pose integrator (2). Every line of
   safety-critical code with no current regression guard. Forward-Euler pose was
   already a regression once (CHANGELOG_2026-04-28); slew limiter has subtle
   correctness ("took three reads to convince myself"). One afternoon.
2. **CI: `colcon build` GitHub Action on every PR.** ~30 LOC of YAML. Catches
   missing `<exec_depend>`, broken `setup.py` glob, `xacro` failure, `vcs import`
   regressions. Half a day.
3. **CI: `arduino-cli compile` for the firmware.** ~10 LOC. Catches the next
   `KP<garbage>`-class parse bug. Add CSV-baseline regression (Phase 4/6c → ±5 %)
   for calibration-drift detection.
4. **`avros_webui/test/test_disconnect_estop.py`** — ~20 LOC, FastAPI `TestClient`.
   Asserts `WebSocketDisconnect` triggers `publish_estop` (`webui_node.py:161-165`).
   **Webui's only safety property is currently untested.**

#### P1 (would mask a quality regression)

5. **Extend `test_perception_launch.py` with synthetic image+cloud publishers**
   (~50 LOC). Currently asserts only LabelInfo arrival; `_on_synced` (HxW resize,
   header propagation, mask publish) has zero E2E coverage.
6. **`test_hsv_hue_wraparound.py`** — synthetic H=178 and H=2 both classified as
   barrel. Active perception P0; no regression guard exists.
7. **`avros_navigation/test/test_generate_graph_schema.py`** — ~30 LOC, mock osmnx
   → 4-node graph, assert `frame == 'map'`, `cost > 0`, unique IDs, datum-shift
   delta. "The single highest-leverage missing artifact in the package."
8. **`avros_bringup/test/test_sensors_launch_smokes.py`** — ~30 LOC, all
   `enable_*=false`, assert `/robot_state_publisher` and `/tf_static` come up.
9. **`avros_sim/test/`** — add the three boilerplate linters at minimum; `colcon
   test --packages-select avros_sim` is silently a no-op today.
10. **`test_on_synced_header_propagation.py`** — kiwicampus contract regression.
    "No test confirms stamp/frame_id flows onto outputs."
11. **Pre-commit hook for `test_hsv_thresholds.py`.** ~5 LOC; refuses commits that
    change HSV thresholds without bumping `EXPECTED_HASH`.
12. **`firmware/test/regression_check.py`** — ~80 LOC; load Phase 4/6c CSV
    baselines and re-run. Standards § 8 "calibration drift detection."

#### P2 (style / coverage polish)

13. Add `launch_pytest` to `<test_depend>` of `avros_perception/package.xml`.
14. Resolve `test_copyright.py` skip across all five Python packages.
15. SHA-256 hash guard for `actuator_params.yaml` PID gains and slew caps.
16. Save subscription / timer handles on `self` in `actuator_node.py:158-163, 198-199`.
17. `launch_testing` smoke checks for `actuator.launch.py`, `webui.launch.py`,
    `localization.launch.py`, `navigation.launch.py`.
18. `firmware/CHANGELOG_BUILD_FOOTPRINT.md` — flash + RAM use per commit.
19. Cross-reference test for `class_map.yaml` ↔ `nav2_params*.yaml class_types` ↔
    `hsv.py:_DEFAULT_CLASS_IDS`.
20. `docs/TESTING.md` — how to run `colcon test`.

## Part 2: IGVC AutoNav competition readiness

### IGVC rules requirements (quoted)

The team's track is **IGVC AutoNav Challenge only** (project memory). Self-Drive is
out of scope, which removes the sign-detection requirement.

The rule citations from `standards_firmware_safety.md` § 5:

> **"Vehicle E-Stops must be hardware based and not controlled through software."**
> Software-only e-stop fails inspection.
>
> **Mechanical e-stop must be located on the center rear of the vehicle, between 2 ft
> and 4 ft high.**
>
> **A wireless e-stop is also required**, range checked at a minimum of **50 ft**
> (older revisions required 100 ft). During AutoNav and SD challenges the wireless
> e-stop is held by the judges.
>
> The competition checks both mechanical placement and wireless range during
> inspection — a software-only or out-of-spec mechanical e-stop is a disqualifier.

Per the same doc § 4, command-loss timeout > 500 ms is "a smell for a moving vehicle"
(robot travels 0.5 m at 1 m/s). The implementation has `cmd_timeout_s = 0.5` and
firmware `WATCHDOG_MS = 300` — within tolerance, upper end.

Other AutoNav requirements (synthesized from the team's standards doc + CLAUDE.md):

- **Lane following** (white paint) — HSV `lane_white` class 1; partial coverage.
- **GPS waypoint navigation** — route_server + GeoJSON graph (currently CPP campus,
  not IGVC course — must regenerate).
- **Obstacle avoidance** — orange barrels and white-on-grass potholes. HSV classes
  `barrel_orange` (2) and `pothole` (3).
- **Course time limit, vehicle dimensions, payload** — team should verify against
  current IGVC rules document; not yet captured in CLAUDE.md.
- **Sustained speed** — CLAUDE.md caps 1.5 m/s; field-tested 1.0 m/s due to
  power-rail brown-out (`firmware/.../FINDINGS.md` "Power rail ceiling identified").

### Capability inventory (have / partial / missing)

| Capability | Status | Risk |
|---|---|---|
| Mechanical e-stop (mushroom, center rear, 2–4 ft) | UNKNOWN — not in software / firmware | **DISQUALIFIER on inspection.** |
| Wireless e-stop (≥ 50 ft, judge-held) | UNKNOWN | **DISQUALIFIER.** |
| Software e-stop on `/avros/actuator_command` | HAVE | Layer-5 only; latching auto-clears on next non-estop command (Phase 2 control P0 #3). |
| Command-loss watchdog (≤ 500 ms) | HAVE | Firmware trip leaves `ctrl_mode = MODE_VELOCITY` (firmware P0 #2); vehicle rolls on slope. |
| Lane following (white paint) | PARTIAL | Concrete misclassified as pothole at ~14 % in outdoor tests (TODO.md). Outdoor lighting risk. |
| Barrel detection (orange) | PARTIAL | **Hue wraparound at sunset misses pixels** (perception P0 #2). |
| Pothole / hazard detection | PARTIAL | Bounds nearly identical to lane; adaptive-V gate disagrees (perception P1 #4). |
| Sign detection | NOT IN SCOPE (AutoNav-only) | None. |
| GPS waypoint navigation (route_server) | PARTIAL | **Graph is CPP campus, not IGVC course** (navigation P0 #1). Generator has no CSV-input mode. |
| IGVC course graph | MISSING | Hand-authoring + CSV import is ~1 afternoon once `--from-csv` lands. |
| RTK GNSS | PARTIAL | NTRIP streaming; outdoor RTK FIXED unverified (TODO.md). |
| Local costmap with LiDAR | BROKEN on Humble | voxel_layer not in `plugins:` list (bringup P0). LiDAR is unused. |
| Collision monitor | MISSING | No backstop > 1 m/s (bringup P0). |
| Costmap inflation | PARTIAL | Humble `0.5 m` < robot radius `0.8 m`; Jazzy `1.8 m` (bringup P0 — pick one). |
| Diff-drive kinematics + slew + heading-hold | HAVE | Multiple P0 robustness gaps (control review): no IMU staleness, no serial liveness, e-stop infinite-jerk. |
| Wheel odometry → EKF #1 | HAVE | `m_per_rev` uncalibrated (TODO.md 5 m test). |
| GPS-anchored EKF #2 + navsat_transform | PARTIAL | Two EKFs share one `ekf.yaml` (sim P0; same defect likely in `avros_bringup`). End-to-end untested. |
| Power rail (compute / motor separation) | MISSING | **Blocks autonomous field testing** (TODO.md). |
| Webui authentication | MISSING | Spectator on venue Wi-Fi can drive (webui P0). |
| NTRIP credentials in source | COMMITTED | Account compromise risk (bringup P0). |
| Sensor mount measurements (URDF) | MISSING | All sketch-derived; GNSS lever arm `[0,0,0]` (bringup P0). |
| Sim-real parity | PARTIAL | Ackermann sim vs diff-drive real; camera frames missing in sim TF (sim P0). |
| CI / build automation | MISSING | Broken code lands silently. |
| Field-test E2E run history | PARTIAL | EKF #1 verified 14 m drive; full Nav2 stack untested (blocked by power rail). |

### Top 10 P0s for the next month

Ranked by what would block the team at judges' inspection or prevent completion of
the AutoNav course. Each item links back to the Phase 2 source.

1. **Hardware mechanical e-stop + wireless e-stop wired into firmware GPIO and a
   battery-to-motor contactor.** Quoted IGVC rule: "Vehicle E-Stops must be hardware
   based and not controlled through software. ... mechanical e-stop must be located
   on the center rear of the vehicle, between 2 ft and 4 ft high. ... wireless e-stop
   ... range checked at a minimum of 50 ft." Phase 2 firmware P0 #1; Phase 2 webui P0 #2.
   **Without this the team fails inspection on day 0.** Half a day of firmware work
   (`pinMode(ESTOP_PIN, INPUT_PULLUP)` + latched-fault state machine); hardware team
   scope for the mushroom button + wireless receiver + contactor wiring.

2. **Dedicated Jetson power rail (48 V → 19 V buck, separate from motor rail).**
   TODO.md: "Currently both Jetson and SparkMAXes share one 48 V→12 V buck; motor
   inrush sags the rail and brown-outs the Jetson. **Blocks safe field testing.**"
   `firmware/teensy_diff_drive/FINDINGS.md` "Power rail ceiling identified" confirms
   sustained ground speed is 1.0 m/s because of this. **Until this is fixed, the
   vehicle cannot autonomously drive a full course at speed without random
   reboots.** Hardware team scope.

3. **`cpp_campus_graph.geojson` is the wrong graph + generator has no IGVC-course
   path.** Phase 2 navigation P0 #1: "OSM has no data for the IGVC AutoNav venue. Add
   a CSV-input mode to `generate_graph.py` ... so a course graph can be hand-authored
   in 30 minutes from the IGVC organizer-provided map or a GPS walk." Plus P0 #2:
   the committed graph is 16 651 nodes / 17 492 edges, not the "52 / 113" CLAUDE.md
   claims; `route_server` query latency on Jetson is unverified. **Without a real
   course graph the team cannot use route_server at the competition.** ~1 afternoon
   for the CSV importer; ~30 minutes per course graph thereafter.

4. **NTRIP credentials committed to git.** Phase 2 bringup P0 SECURITY: "NTRIP
   credentials are committed to git in `config/ntrip_params.yaml`." If the repo is
   public, the rtk2go.com / earthscope.org password is leaked and the account can
   be compromised. Rotate credentials, move to env var or `.gitignore`d local
   override file. ~1 hour.

5. **Latched e-stop in `actuator_node` + bounded e-stop deceleration.** Phase 2
   control P0 #3 + #4: "Once `_estop = True`, do **not** auto-clear on the next
   non-estop ActuatorCommand. Add either a service (`/avros/clear_estop`) or a
   specific `clear_estop` field." And: "Replace the instant zero-step on `_estop`
   ... with a slew-limited ramp using a dedicated `estop_decel_mps2` parameter (e.g.
   3.0 m/s²)." Currently a webui reconnect silently un-stops the motors and an
   e-stop infinite-decel can brown-out the rail (which itself has not been fixed —
   see #2). ~45 minutes.

6. **Serial-link liveness + IMU staleness in `actuator_node`.** Phase 2 control P0
   #1 + #2: "If the cable is unplugged at runtime the `_serial_reader` thread hits
   `SerialException` ... the dead handle stays dead, and the control loop ... keeps
   writing into a dead handle." And: "The IMU-fresh flag is set true on the first
   IMU message and never reset: if the Xsens dies mid-run, the node keeps applying
   heading-hold against a stale yaw forever." Both have ~30 minute fixes (timestamp
   the last message, expire after 200 ms, force `_estop = True`).

7. **Firmware watchdog `ctrl_mode = MODE_VELOCITY` bug.** Phase 2 firmware P0 #2:
   "`teensy_diff_drive.ino` lines 360–361 zero `cmd_rpm` and `cmd_duty` but don't
   change `ctrl_mode`. Means a host-loss event running velocity commands continues
   sending `setVelocity(_, 0)`, which is the exact failure mode the `S` command was
   rewritten to avoid ... On a slope, vehicle rolls." One-line fix, but it requires
   reflashing the Teensy.

8. **Local costmap on Humble has no LiDAR layer.** Phase 2 bringup P0: "Humble local
   costmap has NO obstacle layer at all — purely semantic + inflation. This means
   **LiDAR returns are not used by the local costmap** in Humble. For competition
   this is a serious issue: a hard obstacle the camera fails to classify will not
   appear on the costmap." The voxel_layer block exists in YAML but isn't in the
   `plugins:` list. Half a day to wire in alongside the kiwicampus semantic layer
   without breaking the existing fork.

9. **Webui WebSocket authentication.** Phase 2 webui P0: "no authentication on the
   control WebSocket — anyone reachable to TCP 8000 (vehicle LAN, Tailscale, IGVC
   venue Wi-Fi) can drive the chassis." Bind to localhost or Tailscale-only +
   token check. ~1 hour.

10. **HSV barrel hue wraparound.** Phase 2 perception P0 #2: "Single-range
    `cv2.inRange` on `H ∈ [5, 25]` ... misses pixels when sunset shifts orange
    toward red across `H=0/179`. Add second-range pair (`barrel_low_2`,
    `barrel_high_2`) and OR the masks." The IGVC course is run outdoors in
    arbitrary lighting; this is the single perception bug most likely to bite at
    competition. ~2 hours including the regression test (P1 #6 above).

**Items 1, 2, and 3 are the structurally biggest** (hardware scope, hardware scope,
field walk + CSV authoring). Items 4–10 are software fixes the team can land in a
single concentrated week.

### Risk register

| # | Risk | Likelihood | Impact | Mitigation if not fixed |
|---|---|---|---|---|
| 1 | Mechanical / wireless e-stop missing | Certainty | **Disqualification** | None — team fails inspection. |
| 2 | Jetson brown-out under motor load | High (already observed) | Mid-run reset, run forfeit | Speed-cap to 0.7 m/s; accept slower time. |
| 3 | Wrong route graph / no IGVC graph | Certainty (if route_server is BT path) | No goal-able plan | Use `navigate_to_pose_simple_humble.xml` instead; lose pre-built waypoint logic. |
| 4 | NTRIP credentials leaked | Medium (repo visibility unknown) | Account banned mid-competition | Rotate + env-var. ~1 hour. |
| 5 | Webui reconnect silently un-stops motors | Medium (Wi-Fi blip is common) | Vehicle resumes after operator believed stopped | Train operators to use hardware e-stop. |
| 6 | Serial link to Teensy fails mid-run | Medium (USB-C is known failure mode) | Vehicle keeps moving until 300 ms firmware watchdog | Visual monitoring + hardware e-stop. |
| 7 | Watchdog leaves vehicle in MODE_VELOCITY on slope | Low (course is flat) | Rolls after host crash | Flat course mitigates; hardware e-stop is ground truth. |
| 8 | LiDAR not used in Humble local costmap | High (any non-colored obstacle the camera misses) | Vehicle plows into hard obstacle | Speed-cap reduces severity; no software mitigation. |
| 9 | Spectator drives robot via webui | Low (requires venue Wi-Fi) | Run aborted | Bind to localhost on the day. ~1 minute change. |
| 10 | HSV misses barrels at sunset | Medium (afternoon AutoNav slot) | Vehicle hits barrel | Schedule near solar noon; live-tune at venue. |

**Compounding risk:** items 5, 6, 7, 9 all assume hardware e-stop works. Without it
(item 1), every layered defense collapses. The first three items are the
foundation everything else relies on.

Items the register doesn't quantify well: operational ergonomics (re-arming after
e-stop, recovering from localization loss, mid-course bag recording — walk through
"what does the operator do when X" before competition day) and AutoNav time-on-
course limits (verify against current rules).

### Realistic 1-month timeline

Today is 2026-05-01. The plan assumes a four-person team (mechanical/power, perception,
controls/firmware, nav/integration leads).

**Week 1 — Foundations.** Remove items that block all subsequent field testing.
Mechanical lead orders + installs hardware e-stop (P0 #1) and dedicated Jetson buck
(P0 #2); both during the same shop day. Controls lead lands the four `actuator_node`
software fixes (latched estop, serial liveness, IMU staleness, watchdog mode bug —
P0 #5–7) plus `test_kinematics.py`. Nav lead rotates NTRIP credentials (P0 #4) and
sets up `colcon build` GitHub Action. Perception lead lands hue wraparound fix +
regression test (P0 #10), then the `pothole` HSV tightening from TODO.md. **End-of-
week gate:** 50 m teleop drive with no brown-out + verified mechanical e-stop. If
brown-out persists, escalate — this is the structural blocker.

**Week 2 — Course-level integration; first autonomous run.** Nav lead adds CSV-input
mode to `generate_graph.py` (P0 #3) and pre-authors an AutoNav arena graph; wires
the voxel_layer into Humble local costmap (P0 #8). Controls lead lands the
parameter callback (P1 #9 from `package_avros_control`) + Phase 7 BURN verification.
Perception lead finalizes pothole vs lane separation, re-tightens
`max_obstacle_distance` to 5–8 m, runs `perception_test.launch.py` at the bench
course. Mechanical lead measures sensor mounts (ZED, Velodyne, IMU, GNSS lever arm)
and runs 5 m `m_per_rev` calibration. **End-of-week gate:** first autonomous
30 m navigation run via `navigation.launch.py`.

**Week 3 — Robustness and operational practice.** Run a mock IGVC AutoNav course on
a quad/field with taped lanes + barrels + 5 waypoints; ≥ 3 runs daily, rosbags
captured. Controls lead lands P1 polish (IMU QoS, measured-dt slew, watchdog
state on actuator_state). Perception lead lands integration test that publishes
synthetic image+cloud (test P1 #5) and live-tunes for venue lighting. Nav lead
adds webui auth (P0 #9) + `collision_monitor`. Daily 15-min stand-ups; every
failure into TODO.md. **End-of-week gate:** 5 consecutive autonomous course runs
without operator intervention.

**Week 4 — Polish + dress rehearsal.** Freeze main; any change requires code review
*and* a successful mock-course run. Mechanical lead preps spares (batteries, USB
cables, pre-flashed Teensy). Perception lead compiles a "lighting playbook" with
HSV ranges per time-of-day. Nav lead writes operator runbook, pre-stages the IGVC
course graph, verifies RTK FIXED outdoors. Two days before competition: full dress
rehearsal on the vehicle; no software changes after.

**Buffer.** This schedule has no buffer for "route_server times out at 16k nodes"-
class surprises. Build buffer by descoping P1s from week 2 onward. **The hardest
early decision** — make in week 1 — is whether to use the route_server BT path or
skip it for `navigate_to_pose_simple_humble.xml` with hand-clicked goals. Deciding
early removes the riskiest dependency.

## Positives

The picture above is not "this team is in trouble" — it's "a few high-stakes items to
land in a tight timeline, and the rest of the codebase is better than typical IGVC
work." Things to preserve:

- **`avros_perception` test infrastructure is the workspace high-water mark.** SHA-256
  threshold-drift guard, `launch_pytest` integration test with late-joining
  TRANSIENT_LOCAL subscriber, mock `ApproximateTimeSynchronizer` slop tests, live-
  tunable parameter regression tests. Every pattern transfers to every other package
  in an afternoon.
- **Firmware bring-up trail is exemplary.** Phase scripts + CSV archive +
  CLAUDE.md/BRING_UP.md/FINDINGS.md trio. PID tune is competition-grade (99–100 %
  tracking, 0.83 % L/R sync delta). Multiple subtle SparkMAX FW 26.x protocol bugs
  caught and fixed pre-bring-up.
- **kiwicampus contract correctness in `avros_perception`.** Latched LabelInfo
  (TRANSIENT_LOCAL + RELIABLE), bit-identical stamp/frame_id propagation, mono8
  mask, organized HxW-matched cloud, image resize before pipeline. Every silent
  failure mode the standards doc warns about is guarded. Remaining P0s are 2-hour
  fixes.
- **`avros_control` architecture is conceptually correct** despite hardening gaps.
  dt-aware slew limits with separate accel/decel caps, midpoint pose integration,
  last-message-wins command priority that makes webui and Nav2 produce identical
  chassis behavior, IMU heading-hold gating with the right double-conditional.
- **Nav2 stack assembled with deliberate engineering choices.** ZED wrapper gotchas
  avoided, dual-EKF + navsat_transform pattern matches upstream
  `nav2_gps_waypoint_follower_demo`, cross-distro launch logic, RewrittenYaml two-
  stage merge for sim overrides.
- **`vcs import` source management is professional.** Heavy drivers pinned in
  `avros.repos`; the `Paarseus/avros-fixes` fork stacks four kiwicampus patches as
  commits on a branch — no `git am` step at install time.
- **Documentation is honest.** CLAUDE.md "Known Issues & Fixes" reads like a post-
  incident log. TODO.md is priority-organized with session-delta references.
  FINDINGS.md preserves wrong-then-right reasoning trails. Persistent journald +
  bring-up CSVs mean post-failure forensics will always have data.
- **Per-package Phase 2 review trail exists** — every package has a technical-debt
  inventory with file:line pointers. New contributors can ramp fast.

The team is one structurally hard hardware problem (power rail), one rule-required
hardware item (e-stop), and one missing course graph short of a viable competition
entry. **Land the P0 list, don't break anything else, and the team has a real shot
at a successful AutoNav run.**
