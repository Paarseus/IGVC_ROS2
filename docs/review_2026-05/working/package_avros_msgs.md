# avros_msgs — Review

## Summary

`avros_msgs` is the workspace's interface package (build type `ament_cmake`, package format 3) defining two messages and one service:

- `msg/ActuatorCommand.msg` — webui / external commands to the actuator stack (`/avros/actuator_command`)
- `msg/ActuatorState.msg` — actuator telemetry @ 20 Hz (`/avros/actuator_state`)
- `srv/PlanRoute.srv` — a route-planning service that is **not used anywhere** in the workspace (planning has migrated to nav2_route's `ComputeRoute` action)

Files reviewed: `CMakeLists.txt`, `package.xml`, `LICENSE`, `msg/ActuatorCommand.msg`, `msg/ActuatorState.msg`, `srv/PlanRoute.srv`. There is no `test/` directory.

Overall verdict: the package generally follows the `ros2 pkg create --build-type ament_cmake` boilerplate correctly, and the ActuatorCommand/State pair is wired up and consumed correctly by `avros_control` and `avros_webui`. However, the messages have several professional-quality gaps: undocumented mode strings instead of typed enum constants, missing units on a velocity-derived state vector, no documentation comments on most fields, an unused dead service (`PlanRoute.srv`), a placeholder version (`0.0.0`), an `<exec_depend>` on `std_msgs` that should be elevated for IDL generation correctness, and a license/manifest mismatch (manifest says `MIT` but the bundled `LICENSE` is missing the copyright/notice line that real MIT requires).

---

## Per-file findings

### CMakeLists.txt

- **[P1] CMakeLists.txt:16 — dead service in generation list.** `srv/PlanRoute.srv` is included in `rosidl_generate_interfaces`, but `grep -rn PlanRoute src/` returns only this file and `CMakeLists.txt`. There is no client, no server, no `.action` registration, and `avros_navigation` uses `nav2_route`'s `ComputeRoute` action (see `src/avros_navigation/scripts/generate_graph.py:7`). Either delete `srv/PlanRoute.srv` + this line, or document the field semantics and add a real consumer. Stale interfaces inflate ABI surface and force every dependent package to rebuild on unrelated changes.
- **[P1] CMakeLists.txt — no `find_package(builtin_interfaces REQUIRED)`.** `std_msgs/Header` transitively depends on `builtin_interfaces/Time`. While `rosidl` will resolve this through `std_msgs`, upstream message packages declare `builtin_interfaces` explicitly when any field is `std_msgs/Header`-rooted (see `geometry_msgs/CMakeLists.txt`). Not strictly required for build but conventional.
- **[P2] CMakeLists.txt:1 — `cmake_minimum_required(VERSION 3.8)`.** ROS 2 Humble's official template now uses `3.8` so this is fine, but `3.14` (matching `rosidl_default_generators` minimum on Humble) is more idiomatic for a fresh interface package and prevents subtle policy issues with `IMPORTED` targets that arise on 3.8.
- **[P2] CMakeLists.txt:4-6 — `add_compile_options(-Wall -Wextra -Wpedantic)`** is unused noise for an interface-only package — there are no C++ sources outside the generated stubs, which `rosidl` builds with its own flags. Harmless but dead.
- **[P2] CMakeLists.txt:26-30 — disabled lint shims.** `set(ament_cmake_copyright_FOUND TRUE)` and `set(ament_cmake_cpplint_FOUND TRUE)` are the `ros2 pkg create` boilerplate that suppresses the copyright + cpplint linters. Standard ros2/demos packages either delete these lines (after adding headers) or leave them as-is. Given the workspace's small size and `LICENSE` file in place, adding a one-line copyright header to each `.msg` (lines starting with `#`) and removing these shims would let the linter actually run.
- **[P2] CMakeLists.txt:20 — `ament_export_dependencies(rosidl_default_runtime)`** is correct, but the surrounding boilerplate is missing the equally idiomatic line `ament_export_dependencies(std_msgs)` — without it, downstream C++ packages that include the generated headers may not transitively pull in `std_msgs`'s targets. Most rosidl pipelines work anyway because consumers `find_package(std_msgs)` themselves, but exporting it is the safer pattern.

### package.xml

- **[P1] package.xml:5 — `<version>0.0.0</version>`.** Placeholder version. Even if you don't intend to release on ROS Index, `0.1.0` or `0.0.1` signals "real" rather than "scaffolded." Versioning is required by REP 149 and cmake's `bloom-generate` will reject `0.0.0` for releases.
- **[P1] package.xml:7 — placeholder maintainer email.** `<maintainer email="avlab@cpp.edu">AV Lab</maintainer>` — verify the email actually receives mail (this looks like a TODO holdover). REP 149 §2 mandates the email be valid and reachable; a dead email is a CI/release blocker.
- **[P1] package.xml:13 — `<depend>std_msgs</depend>` is wrong polarity for an IDL package.** REP 149 + the upstream pattern in `common_interfaces/std_msgs/package.xml` and `geometry_msgs/package.xml` use `<build_depend>` + `<exec_depend>` (not `<depend>`) for message-package dependencies, because the IDL-generated headers need it at build time AND the runtime libraries link against it. The `<depend>` shortcut also expands to `<build_export_depend>`, which is fine but not idiomatic for interface packages — most upstream IDL packages enumerate the four explicit tags. This passes today; flag for cleanup if you ever release upstream.
- **[P1] package.xml:7 — license inconsistency.** `<license>MIT</license>` is declared, but `LICENSE` (lines 1-17) is the **MIT body without the leading `Copyright (c) <year> <holder>` line and without the conditions sentence**. A real MIT license has a 3-block structure (copyright notice, permission grant, warranty disclaimer); this file is missing block 1. Either fix the LICENSE file to include `Copyright (c) 2026 Cal Poly Pomona AV Lab` (or whoever holds copyright) at the top, or change `<license>` to match what's actually written. Some package indexers reject manifests with mismatched license metadata.
- **[P2] package.xml:18-19 — only `<test_depend>ament_lint_auto</test_depend>` + `ament_lint_common`.** No `<test_depend>` for `ros2_pytest`, `launch_pytest`, or per-language linters (`ament_cmake_gtest` for any future C++ helper). This is fine for a pure-IDL package — there's nothing to test beyond linting — but if you ever add even a single `.hpp` constants file (recommended below for the mode enum), you'll want gtest scaffolding.
- **[P2] package.xml — no `<author>` tag.** REP 149 §2.4 lists `<author>` as optional but recommended; the workspace's other packages mostly don't have one either, so this is consistent at least.

### msg/ActuatorCommand.msg

```
std_msgs/Header header
bool estop
float32 throttle      # 0.0-1.0
string mode           # N, D, S, R
float32 brake         # 0.0-1.0
float32 steer         # -1.0-1.0 (normalized, + = CCW / left, per REP-103)
```

- **[P0] line 4 — `string mode` instead of `uint8` constants.** This is the single biggest design smell. The valid set is documented in a comment as `N, D, S, R` (Neutral, Drive, Sport?, Reverse?), but a `string` field:
  - allows arbitrary sender-side typos (`"d"`, `"drive"`, `"DRIVE"`, `""`) that the receiver must defensively parse,
  - costs ~40 bytes of serialization overhead per message vs 1 byte for `uint8`,
  - cannot be auto-validated by `vision_msgs`-style introspection tools,
  - cannot be exhaustively switched in C++ without a `std::map<std::string, enum>`.

  ROS 2 conventions for enums in `.msg` files use ALL-CAPS constants. Replace with:
  ```
  uint8 MODE_NEUTRAL = 0
  uint8 MODE_DRIVE   = 1
  uint8 MODE_SPORT   = 2
  uint8 MODE_REVERSE = 3
  uint8 mode
  ```
  Reference upstream: `sensor_msgs/NavSatStatus.msg` uses exactly this `int8 STATUS_NO_FIX = -1` + `int8 status` pattern. Then `actuator_node.py:326` becomes `msg.mode = ActuatorState.MODE_DRIVE if not self._estop else ActuatorState.MODE_NEUTRAL` which is type-checkable.
- **[P1] line 1 — `std_msgs/Header header` is set on send (webui_node.py:77) but **not consumed** by the receiver.** Search of `actuator_node.py` shows zero references to `msg.header` — it accepts every command without staleness or frame check. Either (a) wire the receiver to reject commands with `now - msg.header.stamp > timeout` (currently done with `_last_actuator_cmd_t = self.get_clock().now()` instead, which uses receive time, not send time — vulnerable to network buffering), or (b) drop the header entirely. Keeping a populated-but-ignored header is misleading.
- **[P1] lines 3-6 — no field documentation beyond inline ranges.** Real upstream messages use `#`-prefixed multi-line block comments above each field explaining semantics, units, and valid ranges. Compare `sensor_msgs/NavSatFix.msg` or `geometry_msgs/Twist.msg`. Specifically:
  - `throttle` and `brake`: the comment says `0.0-1.0` but doesn't say what happens at the endpoints (full throttle = `_max_v`?), nor whether they're mutually exclusive (`actuator_node.py:220` does `(throttle - brake) * max_v` so they ARE allowed simultaneously and result in `throttle - brake`).
  - `steer`: documents the sign convention but not the magnitude semantics (`±1.0` = `±max_w`).
  - `estop`: is this latching or one-shot? (`actuator_node.py:212` makes it latching: once `estop=true`, the next `false` clears it. Document this.)
- **[P2] line 5 — semantically odd to mix `mode` (a discrete state) with `throttle/brake/steer` (continuous setpoints) in the same message.** Real automotive control APIs separate them (mode is sent on transitions; pedals/wheel are streamed). Not a refactor for IGVC week but flag for any post-competition cleanup.
- **[P2] no `Twist`-shaped representation.** Since `actuator_node.py:220-221` immediately maps `(throttle, brake, steer) → (v, ω)` the same way it maps `cmd_vel.linear.x`/`angular.z`, an alternative design is `geometry_msgs/Twist setpoint` + `bool estop` + `uint8 mode` — but that diverges from the webui's "joystick UI" mental model, so this is a stylistic note only.

### msg/ActuatorState.msg

```
std_msgs/Header header
bool estop
float32 throttle
string mode
float32 brake
float32 steer
bool watchdog_active
```

- **[P0] lines 3-6 — no units / ranges documented at all.** ActuatorCommand has at least a comment per field; ActuatorState has none. The actuator node populates these by **inverse-mapping wheel RPM through `max_v`** (actuator_node.py:319-322) which is a different semantic than ActuatorCommand: in Command they're "what I want," in State they're "approximated from measured RPM." This is non-obvious from the `.msg` alone and trips up new contributors. Add comments stating "telemetry derived from measured wheel velocity, not the commanded value." Consider renaming to `measured_throttle` etc., or split into `cmd_throttle` + `meas_throttle`.
- **[P1] no measured kinematic state.** State messages from real platforms include the measured `linear_x_mps`, `angular_z_rps`, per-wheel velocities, motor currents, voltages, faults, etc. This package's ActuatorState reports only the back-calculated normalized values. The actuator node already has `_l_meas_rpm`, `_r_meas_rpm`, bus voltage from `D` line, etc. — exposing those would make the message useful for diagnostics, replay, and mechanical tuning. For IGVC field debugging this matters.
- **[P1] line 4 — `string mode`.** Same P0 enum problem as ActuatorCommand. Apply the same fix.
- **[P1] line 7 — `bool watchdog_active` is always `false`.** `actuator_node.py:327` literally hard-codes `msg.watchdog_active = False` — there is no watchdog state ever set. Either implement the watchdog (the node does have `_cmd_timeout` logic that could feed this) or remove the field. Publishing a boolean that is constant `false` is misleading telemetry.
- **[P2] line 2 — `bool estop` echoes the commanded estop, not a measured one.** A safer ActuatorState exposes both (`estop_commanded` and `estop_engaged`) so the operator can see if the hardware actually obeyed. With the current single field a stuck SparkMAX would not show up in the state. Future work.

### srv/PlanRoute.srv

```
float64 destination_lat
float64 destination_lon
---
bool success
string message
float64 distance_meters
uint32 num_waypoints
```

- **[P1] entire file is dead code.** Already noted under CMakeLists. No producer, no consumer, no test. Nav2 navigation goes through `nav2_route`'s `ComputeRoute` action (see `src/avros_navigation/scripts/generate_graph.py:7-10` and `CLAUDE.md` "Route Server" section). **Action: delete `srv/` directory, drop `srv/PlanRoute.srv` from CMakeLists.txt:16, and delete the `srv` mention from any documentation.**
- **[P2] design concern (if this ever gets resurrected).** Goal coordinates as a flat `(lat, lon)` pair without a frame is also a smell — real services use `geometry_msgs/PoseStamped` or `geographic_msgs/GeoPose` so the frame, timestamp, and orientation travel with the request. Compare `nav2_msgs/srv/IsPathValid` or `nav_msgs/action/ComputePathToPose` for canonical signatures.

---

## Cross-cutting issues

- **No `test/` directory.** `ament_lint_auto` is declared as a `<test_depend>` but there's no `test/` subdir, so `colcon test --packages-select avros_msgs` runs zero tests. For an IDL package this is mostly fine — there's nothing to unit-test — but a stub `test/test_copyright.py`, `test_flake8.py`, `test_pep257.py` ensures CI still flags formatting regressions on the `.msg` files (line-trailing whitespace, etc.). Standard `ros2 pkg create` scaffolds these.
- **No copyright header in any of the `.msg` / `.srv` / `CMakeLists.txt` / `package.xml` files.** Even with `set(ament_cmake_copyright_FOUND TRUE)` suppressing the linter, professional review would expect at least one header line per file: `# Copyright 2026 Cal Poly Pomona AV Lab\n# Licensed under MIT...`. This is the "license matches license file matches headers" rule from the standards doc Quick Checklist item #8.
- **Single source of truth for mode enum is missing.** The strings `"N"`, `"D"`, `"S"`, `"R"` appear:
  - in `ActuatorCommand.msg` line 4 comment,
  - in `ActuatorState.msg` line 4 (no comment),
  - in `webui_node.py:82, 92` (`msg.mode = mode` and `msg.mode = 'N'`),
  - in `actuator_node.py:326` (`msg.mode = 'D' if not self._estop else 'N'`),
  - in `static/app.js:101` (`btn.dataset.mode`).

  Five separate spellings of the same enum with no central declaration means any new mode (e.g., `"L"` for low gear) requires touching at least 3 ROS 2 files plus the JS. Encoding it as `uint8` constants in the `.msg` (see ActuatorCommand finding) gives the JS something to reference (`/avros_msgs/msg/ActuatorState` introspection exposes constants over the wire).
- **No header time consumed anywhere.** Both messages have `Header header` populated by the publisher but neither consumer reads `msg.header.stamp` or `msg.header.frame_id`. If the team is not going to use it for staleness rejection or rosbag replay alignment, drop it — empty headers cost serialization time and clutter logs. If you DO want replayability (recommended), add receive-side staleness checks in `actuator_node._on_actuator_cmd` based on `msg.header.stamp`, not arrival time.

---

## Punch list

### P0 (IGVC blockers)

- **Replace `string mode` with `uint8 mode` + constants** in both `ActuatorCommand.msg` and `ActuatorState.msg`. Eliminates a class of "typo'd mode silently ignored" bugs that could surface during the AutoNav run when the operator UI re-mounts. Touch points: 2 `.msg` files, `webui_node.py`, `actuator_node.py`, `static/app.js`. Roughly 30 minutes of work.
- **Document units/ranges/semantics on every field** in both `.msg` files (especially ActuatorState, which has none). Without this, a judge or post-competition reviewer cannot tell whether a `0.5` throttle in a rosbag means "50% of max speed," "50% of motor max RPM," or "50% commanded vs measured." 15 minutes.

### P1 (professional quality)

- **Delete `srv/PlanRoute.srv` + its CMakeLists entry.** Dead code. 2 minutes.
- **Fix LICENSE file** — add the missing `Copyright (c) 2026 ...` line at the top so it matches the manifest's `<license>MIT</license>` declaration. 1 minute.
- **Bump `<version>` from `0.0.0` to `0.1.0`** to signal "real, in use." 30 seconds.
- **Verify `<maintainer email="avlab@cpp.edu">`** — confirm the address actually receives mail and someone reads it; otherwise change to a person's real email per REP 149.
- **Remove `bool watchdog_active`** (or wire it to the actual `_cmd_timeout` watchdog logic in `actuator_node.py:233-242`). Currently constant `false`.
- **Decide and document Header semantics.** Either drop `Header header` from both messages, or wire the receiver in `actuator_node._on_actuator_cmd` to reject commands with `(now - msg.header.stamp) > stale_threshold`. Ignored fields are technical debt.
- **Add `<exec_depend>std_msgs</exec_depend>` + `<build_depend>std_msgs</build_depend>`** explicitly instead of the `<depend>` shorthand, matching upstream message packages (`common_interfaces`, `nav2_msgs`).
- **Expand `ActuatorState`** to include actual measured kinematics: `float32 measured_linear_mps`, `float32 measured_angular_rps`, `float32 left_wheel_rpm`, `float32 right_wheel_rpm`, `float32 bus_voltage_v`. The actuator node already computes these for control; surfacing them aids field debugging during IGVC.

### P2 (nice-to-have)

- Bump `cmake_minimum_required` to `3.14` for parity with `rosidl_default_generators` Humble.
- Drop `add_compile_options(-Wall -Wextra -Wpedantic)` from CMakeLists.txt:4-6 — no C++ in this package.
- Add a stub `test/test_copyright.py` + `test_flake8.py` + `test_pep257.py` so `colcon test` does something on this package.
- Add a one-line `# Copyright 2026 ...` header to each `.msg` and `.srv` file, then remove the `set(ament_cmake_copyright_FOUND TRUE)` shim.
- Add `ament_export_dependencies(std_msgs)` next to the existing `ament_export_dependencies(rosidl_default_runtime)` — better transitive-deps hygiene.
- Consider splitting `ActuatorCommand` into a discrete `mode_command` topic + a streamed `pedals` topic; `Twist`-shaped setpoint is also worth considering for symmetry with `cmd_vel`. Post-competition refactor.
- Add `<author>` tag for accreditation.

---

## Positives

- Correctly uses `format="3"` package manifest with `<build_type>ament_cmake</build_type>` in `<export>` — matches REP 149 and the standards doc Quick Checklist item #1.
- `member_of_group rosidl_interface_packages` is present (package.xml:15), required for downstream consumers' interface generation to discover this package.
- `rosidl_generate_interfaces` with `DEPENDENCIES std_msgs` (CMakeLists.txt:13-18) is the modern idiomatic form; not the legacy `add_message_files` + `generate_messages` pattern.
- `<exec_depend>rosidl_default_runtime</exec_depend>` + `<buildtool_depend>rosidl_default_generators</buildtool_depend>` are correctly split (package.xml:11, 16) — this is exactly the pattern from `ros2/example_interfaces/package.xml`.
- The package is consumed correctly: `avros_control` and `avros_webui` both declare `<depend>avros_msgs</depend>` and import via `from avros_msgs.msg import ActuatorCommand, ActuatorState` — no half-broken consumers.
- Both messages embed `std_msgs/Header header` as the first field, matching the upstream convention even if the field is currently unused.
- File layout (`msg/`, `srv/`, `package.xml`, `CMakeLists.txt`, `LICENSE`) matches the standard `ros2 pkg create --build-type ament_cmake` template — no surprises for a reviewer familiar with ROS 2.
- `ActuatorCommand.msg:6` documents the steer-sign convention with explicit reference to REP-103 ("+ = CCW / left, per REP-103") — that's the kind of prose comment that prevents subtle sign-flip bugs and shows the author was thinking about it.
