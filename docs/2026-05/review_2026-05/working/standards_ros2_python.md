# Standards Reference: ROS 2 Humble Python Packages

This document captures what professional / upstream ROS 2 Humble looks like for `ament_python` packages. It is intended as the rubric for downstream Phase 2 reviewers comparing the IGVC_ROS2 codebase against community standards. Every claim is sourced inline. No code from this repo has been read while writing it.

---

## 1. ament_python Package Layout

The canonical layout for a pure-Python ROS 2 package.

- **Build type is `ament_python`** declared in `package.xml` under `<export><build_type>ament_python</build_type></export>` ([Creating Your First ROS 2 Package — Humble](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)). Use `ament_cmake_python` only when mixing C/C++ and Python in the same package ([Ament CMake Python — Humble](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html)).
- **Required files / directories:** `package.xml`, `setup.py`, `setup.cfg`, `resource/<package_name>` marker file, `<package_name>/__init__.py`, and a `test/` directory ([Humble Creating-Your-First-ROS2-Package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)).
- **`setup.cfg`** is required for `ros2 run` to find executables and must contain ([Humble Creating-Your-First-ROS2-Package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)):
  ```ini
  [develop]
  script_dir=$base/lib/<package-name>
  [install]
  install_scripts=$base/lib/<package-name>
  ```
- **`setup.py`** mirrors `package.xml` metadata, declares console-script entry points, and installs non-Python data files. Canonical pattern from upstream `demo_nodes_py` setup.py ([ros2/demos demo_nodes_py/setup.py @ humble](https://github.com/ros2/demos/blob/humble/demo_nodes_py/setup.py)):
  ```python
  data_files=[
      ('share/ament_index/resource_index/packages',
       ['resource/' + package_name]),
      ('share/' + package_name, ['package.xml']),
  ],
  install_requires=['setuptools'],
  entry_points={
      'console_scripts': [
          'talker = demo_nodes_py.topics.talker:main',
      ],
  },
  ```
- **Installing non-Python data files** (launch / config / urdf / rviz) goes through `data_files=` with `glob`. Canonical idiom — Humble docs and `lifecycle_py` reference setup.py ([Integrating launch files into ROS 2 packages — Humble](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html), [ros2/demos lifecycle_py/setup.py @ humble](https://github.com/ros2/demos/blob/humble/lifecycle_py/setup.py)):
  ```python
  import os
  from glob import glob
  data_files=[
      ('share/ament_index/resource_index/packages',
          ['resource/' + package_name]),
      ('share/' + package_name, ['package.xml']),
      (os.path.join('share', package_name, 'launch'),
          glob('launch/*.launch.py')),
      (os.path.join('share', package_name, 'config'),
          glob('config/*.yaml')),
      (os.path.join('share', package_name, 'urdf'),
          glob('urdf/*.xacro')),
      (os.path.join('share', package_name, 'rviz'),
          glob('rviz/*.rviz')),
  ]
  ```
- **Launch file naming:** files must end in `*.launch.py` for `ros2 launch` autocomplete to find them ([Humble Launch-system](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)).
- **Install destinations** ([Humble Developing-a-ROS-2-Package](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)):
  - executables: `lib/<package>` (set by `setup.cfg`)
  - shared resources: `share/<package>/{launch,config,urdf,rviz,...}`
  - package metadata: `share/<package>/package.xml`
  - resource marker: `share/ament_index/resource_index/packages/<package>`
- **`package.xml` format 3** (current, supported on Humble — format 2 is also valid but format 3 is what new packages use; format declared as `<package format="3">`) ([REP 149 — Package Manifest Format Three Specification](https://ros.org/reps/rep-0149.html)).
- **Dependency tags semantics** ([REP 149](https://ros.org/reps/rep-0149.html)):
  | Tag | Meaning |
  |---|---|
  | `<buildtool_depend>` | Tool needed to build (e.g., `ament_cmake`, `rosidl_default_generators`). For ament_python this is normally `ament_python`. |
  | `<build_depend>` | Compile-time dependency (rare for pure Python). |
  | `<exec_depend>` | Runtime dependency. **For ament_python this is the dominant tag** — `rclpy`, message packages, other ROS packages your nodes import at runtime. |
  | `<depend>` | Shorthand for `build_depend` + `build_export_depend` + `exec_depend`. Common for C++; for Python prefer the more precise `<exec_depend>` ([ros2/demos demo_nodes_py/package.xml @ humble](https://github.com/ros2/demos/blob/humble/demo_nodes_py/package.xml)). |
  | `<test_depend>` | Test-only deps: `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`. |
  | `<buildtool_export_depend>` / `<build_export_depend>` / `<exec_depend>` etc. | Less common; `_export_` variants are for packages that consume yours. |
- **License conventions:** `<license>` field in package.xml takes a SPDX-ish identifier. `Apache-2.0` (written as `Apache License 2.0` in many older manifests) is the de facto ROS 2 default — every official `ros2/demos` package uses it ([demo_nodes_py/package.xml](https://github.com/ros2/demos/blob/humble/demo_nodes_py/package.xml)). A matching `LICENSE` file at package root or repo root is expected; the `ament_copyright` linter will flag missing copyright headers in `.py` files.
- **`resource/<package_name>`** is an empty marker file (touched, zero bytes) that the ament index uses to discover the package after install ([Humble Creating-Your-First-ROS2-Package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)).

---

## 2. rclpy.Node Anatomy

The recommended Python node structure on Humble.

- **Standard skeleton** ([Writing a simple publisher and subscriber (Python) — Humble](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)):
  ```python
  import rclpy
  from rclpy.node import Node

  class MyNode(Node):
      def __init__(self):
          super().__init__('my_node')
          self.declare_parameter('rate_hz', 10.0)
          self.pub = self.create_publisher(Msg, 'topic', 10)
          self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

      def tick(self):
          self.get_logger().info('hello')

  def main(args=None):
      rclpy.init(args=args)
      node = MyNode()
      try:
          rclpy.spin(node)
      except KeyboardInterrupt:
          pass
      finally:
          node.destroy_node()
          rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```
- **Parameter declaration** is mandatory in Humble — undeclared `get_parameter()` raises `ParameterNotDeclaredException` unless `allow_undeclared_parameters=True` is set on the node, which is **discouraged** ([Using parameters in a class (Python) — Humble](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)). Declare with a default value and (optionally) a `ParameterDescriptor` for type, range, and read-only constraints.
- **Reading parameters** uses the verbose form `self.get_parameter('name').get_parameter_value().double_value` or the shorter `.value` ([Humble Using-Parameters-In-A-Class-Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)).
- **Parameter callbacks**: register with `add_on_set_parameters_callback(callback)`. The callback receives `List[Parameter]` and **must return a `rcl_interfaces.msg.SetParametersResult`** with `successful: bool` and optional `reason: str` ([rclpy Parameters cookbook](https://github.com/mikeferguson/ros2_cookbook/blob/main/rclpy/parameters.md), [The Robotics Back-End: rclpy Parameter Callback](https://roboticsbackend.com/ros2-rclpy-parameter-callback/)):
  ```python
  from rcl_interfaces.msg import SetParametersResult
  def _on_set(self, params):
      for p in params:
          if p.name == 'rate_hz' and p.value <= 0:
              return SetParametersResult(successful=False, reason='rate_hz must be > 0')
      return SetParametersResult(successful=True)
  self.add_on_set_parameters_callback(self._on_set)
  ```
  Validate first, mutate state second; the callback fires *before* the new value is committed.
- **Logger usage**: always `self.get_logger().info/warn/error/debug(...)` — this routes through `/rosout` and is namespaced to the node. Do not use bare `print()` ([rclpy Node API](https://docs.ros.org/en/rolling/p/rclpy/rclpy.node.html)). Prefer `_throttle` and `_once` variants for hot loops to avoid log spam.
- **Lifecycle vs plain Node** ([Node lifecycle design — design.ros2.org](https://design.ros2.org/articles/node_lifecycle.html)): Lifecycle nodes give a state machine (`unconfigured → inactive → active → finalized`) with explicit `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`, `on_shutdown`, `on_error` transitions. Use Lifecycle when:
  - the node is part of a coordinated bring-up (Nav2 servers all use Lifecycle);
  - other nodes must wait until this one is "ready";
  - you need clean restart-without-process-kill semantics.
  Use a plain `Node` for everything else — sensor drivers, simple controllers, teleop UIs.
- **Executors** ([About Executors — Humble](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)):
  - `SingleThreadedExecutor` (the default `rclpy.spin()` uses this) processes callbacks sequentially. Default and correct choice for >90% of nodes.
  - `MultiThreadedExecutor` runs callbacks across N worker threads. Required when you have callbacks that legitimately need to run concurrently (e.g., a service handler that calls another service, or a long-running action callback that must coexist with periodic timers).
  - `StaticSingleThreadedExecutor` is an optimization for nodes whose entity set is fixed at init.
  - **Callback groups** ([Executors — Humble](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)): `MutuallyExclusiveCallbackGroup` (default; prevents parallel execution of callbacks in the group) vs `ReentrantCallbackGroup` (allows parallel execution). Use Reentrant only with thread-safe callbacks. Mixing a long-running timer and a service in the default group will deadlock under MultiThreaded if the service waits on the timer's state.
- **Shutdown / signal handling** ([Humble Writing-A-Simple-Py-Publisher-And-Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)): wrap `spin()` in `try/except KeyboardInterrupt` and always pair `node.destroy_node()` with `rclpy.shutdown()` in a `finally`. `rclpy.init(args=args)` parses `--ros-args` from `sys.argv` ([ROS Command Line Arguments design](https://design.ros2.org/articles/ros_command_line_arguments.html)).

---

## 3. QoS Profiles

- **Predefined profiles in rclpy** ([About Quality of Service Settings — Humble](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html), [QoS design — design.ros2.org](https://design.ros2.org/articles/qos.html)):
  | Profile | Reliability | Durability | History | Depth | Use for |
  |---|---|---|---|---|---|
  | `qos_profile_default` | Reliable | Volatile | Keep last | 10 | Generic comms — control commands, planner output, default for `create_publisher(_, _, 10)` |
  | `qos_profile_sensor_data` | Best effort | Volatile | Keep last | 5 | High-rate lossy sensor streams — IMU, LiDAR, cameras |
  | `qos_profile_services_default` | Reliable | Volatile | Keep last | 10 | Service request/reply (set automatically by `create_service` / `create_client`) |
  | `qos_profile_parameters` | Reliable | Volatile | Keep last | 1000 | Parameter service (set automatically) |
  | `qos_profile_system_default` | RMW default | RMW default | RMW default | RMW default | Avoid — values vary by RMW implementation |
- **`TRANSIENT_LOCAL` + `RELIABLE` = "latched" topic** ([Humble About-Quality-of-Service-Settings](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)). The publisher's outgoing-DataWriter cache replays the last N messages to any late-joining subscriber. Required for:
  - `/map` (nav_msgs/OccupancyGrid) — published once, must survive subscriber restart.
  - `/robot_description` (std_msgs/String) — `robot_state_publisher` publishes URDF once and latches it.
  - `/tf_static` — already latched by tf2.
  - `vision_msgs/LabelInfo`, semantic class maps, calibration-info-style topics.
  - Static configuration that downstream consumers must see exactly once.
- **TRANSIENT_LOCAL pitfalls**:
  - **Both endpoints must agree.** A transient_local publisher and a default-volatile subscriber are still QoS-compatible (subscriber accepts the stricter publisher offer), but a volatile publisher and a transient_local subscriber **do not match** — the subscriber refuses the weaker offer ([QoS design — design.ros2.org](https://design.ros2.org/articles/qos.html)). Symptom: silent topic with both ends listed by `ros2 topic info`.
  - **Depth matters.** `KEEP_LAST` with depth 1 + transient_local replays only the most recent message. Setting depth too low for a topic that publishes a sequence (like a class-info series) means late joiners miss earlier entries.
  - **Reliability must also match** — transient_local without reliable is rare and usually a bug; the durable cache is meant to be replayed reliably.
  - The plugin-side check on `kiwicampus/semantic_segmentation_layer` (and other Nav2 layers) is a textbook example: the plugin loads after the publisher, so without `transient_local + reliable` the latched LabelInfo is lost.
- **`KEEP_LAST` depth choices**:
  - Sensor streams: 1–5 — old data is useless, stale samples just clog memory ([sensor_data profile](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)).
  - Commands (cmd_vel, ActuatorCommand): 1 — only the latest setpoint matters; queueing them causes lag spirals.
  - Diagnostics, status: 10 (default).
  - Mission-critical event topics: tune up to 100 if you genuinely need history; otherwise prefer durability + small depth.
- **Common mistakes for sensor pipelines**:
  - Default QoS (reliable, depth 10) on a 100 Hz IMU stream — the reliability buffer fills, publisher blocks, latency spikes. Use `qos_profile_sensor_data`.
  - Mismatched QoS between rosbag-replayed data (often best_effort) and a reliable subscriber — bag plays but subscriber sees nothing. Match the bag's profile or use compatible best_effort.
  - Using `qos_profile_system_default` for anything cross-RMW. Different RMW implementations (CycloneDDS, FastDDS, RMW Connext) produce different defaults; always pick an explicit profile.

---

## 4. Launch Files

- **Authoritative source**: launch tutorials on docs.ros.org ([Creating a launch file — Humble](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html), [Using substitutions — Humble](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html), [Using event handlers — Humble](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Event-Handlers.html), [launch architecture](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)).
- **`DeclareLaunchArgument` + `LaunchConfiguration`** is the correct way to expose user-tunable values:
  ```python
  DeclareLaunchArgument('use_sim_time', default_value='false',
                         description='Use /clock from a simulator')
  use_sim_time = LaunchConfiguration('use_sim_time')
  Node(..., parameters=[{'use_sim_time': use_sim_time}])
  ```
  `LaunchConfiguration` is a *substitution* — it's a deferred reference, not a Python string. You cannot do `if use_sim_time == 'true':` in module-level code ([ros2/launch architecture](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)).
- **`OpaqueFunction`** is the escape hatch when you genuinely need a Python value at launch description time (e.g., load a YAML, branch on contents) ([Humble Using-Substitutions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html), [ros2/launch issue #599](https://github.com/ros2/launch/issues/599)):
  ```python
  def _setup(context, *args, **kwargs):
      camera = LaunchConfiguration('camera').perform(context)  # now a str
      cfg = yaml.safe_load(open(f'config/{camera}.yaml'))
      return [Node(package='avros_perception', executable='perception_node',
                   parameters=[cfg])]
  return LaunchDescription([
      DeclareLaunchArgument('camera', default_value='front'),
      OpaqueFunction(function=_setup),
  ])
  ```
  Don't reach for `OpaqueFunction` if a normal substitution can do the job — it bypasses the introspectable launch description and disables `ros2 launch --show-args` for anything inside it.
- **`IncludeLaunchDescription`** for composing multi-file trees:
  ```python
  IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          PathJoinSubstitution([FindPackageShare('avros_bringup'),
                                'launch', 'sensors.launch.py'])),
      launch_arguments={'enable_zed_front': 'true'}.items(),
  )
  ```
  `launch_arguments` keys must be string keys with string values. Use `PathJoinSubstitution` and `FindPackageShare` rather than `os.path.join` so install paths resolve correctly under colcon's symlink-install ([ros2/launch architecture](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)).
- **Conditionals — `IfCondition` / `UnlessCondition`** ([Humble Using-Event-Handlers](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)):
  ```python
  from launch.conditions import IfCondition, UnlessCondition
  from launch.substitutions import PythonExpression
  Node(..., condition=IfCondition(LaunchConfiguration('enable_zed_front')))
  Node(..., condition=IfCondition(PythonExpression([
      "'", LaunchConfiguration('camera'), "' == 'front'"])))
  ```
  Both conditions evaluate at launch time. Multi-term expressions go through `PythonExpression([...])` and substitute via list elements (each `LaunchConfiguration` slot becomes a separate element).
- **`GroupAction` + `PushRosNamespace`** is how you scope a subtree to a namespace without each `Node()` repeating it:
  ```python
  GroupAction([PushRosNamespace('zed_front'), Node(...), Node(...)])
  ```
- **Environment variables**: use `SetEnvironmentVariable('NAME', 'value')` (or `AppendEnvironmentVariable`) at the top of `LaunchDescription` ([launch architecture](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)). Common pattern for ROS 2:
  ```python
  SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
  SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
  SetEnvironmentVariable('CYCLONEDDS_URI', cyclonedds_uri),
  SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
  ```
- **Namespaces vs node names** — these are *separate* concepts but the `Node` action ties them together by concatenating into the fully-qualified node name ([ros2/launch_ros launch_ros/actions/node.py @ humble](https://github.com/ros2/launch_ros/blob/humble/launch_ros/launch_ros/actions/node.py)):
  - `namespace='/zed_front'` + `name='zed_node'` → fully-qualified node `/zed_front/zed_node` and topics under `/zed_front/...`.
  - **`IncludeLaunchDescription` + child launch file:** when you include a launch file that already passes `namespace=` to its Nodes, do not also pass a wrapping namespace from the parent unless the child file is namespace-aware. The zed-ros2-wrapper bug (recorded in this project's CLAUDE.md) is the canonical example: passing both `camera_name` and `namespace`/`node_name` to `zed_camera.launch.py` causes `node_name` to be silently overwritten by `camera_name`. Read the included launch file's argument list before wrapping.
  - In a parent launch, prefer `GroupAction([PushRosNamespace(ns), IncludeLaunchDescription(...)])` over passing `namespace=` to each child Node.

---

## 5. Test Patterns

- **Boilerplate `test_copyright.py`, `test_flake8.py`, `test_pep257.py`** are still standard for new ament_python packages on Humble — they are auto-generated by `ros2 pkg create --build-type ament_python` ([Ament Lint CLI Utilities — Humble](https://docs.ros.org/en/humble/Tutorials/Advanced/Ament-Lint-For-Clean-Code.html), [ament_flake8](https://index.ros.org/p/ament_flake8/), [ament_copyright](https://index.ros.org/p/ament_copyright/), [ament_pep257](https://index.ros.org/p/ament_pep257/)). What each enforces:
  - `test_copyright.py` — every `.py` file has a copyright + license header. Run `ament_copyright --add-missing 'Author Name' apache2` to fix.
  - `test_flake8.py` — PEP-8 conformance via Flake8 with ROS-specific config.
  - `test_pep257.py` — docstring style per PEP 257.
  - All three are gated behind `<test_depend>` in package.xml; a missing dep silently skips the test.
- **`test_copyright.py` is often skipped in upstream packages** that don't enforce it (it's commented out in many `ros2/demos` packages because the copyright check is heavy on third-party / contributor-pushed code). Skipping is acceptable but the file should at least compile and `pytest.skip()` cleanly, not error out.
- **`pytest` is the test runner** and pytest is required as a `<test_depend>` (`python3-pytest`) ([Writing Basic Tests with Python — Humble](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html)). Tests live in `test/` (or `tests/`) at package root with names `test_*.py`; functions named `test_*` are auto-discovered.
- **Unit test layout** ([Humble Writing-Basic-Tests-Python](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html)):
  ```
  my_pkg/
    my_pkg/
      __init__.py
      foo.py
    test/
      test_copyright.py    # boilerplate
      test_flake8.py       # boilerplate
      test_pep257.py       # boilerplate
      test_foo.py          # actual unit tests
  ```
- **Running tests:** `colcon test --packages-select <pkg>` then `colcon test-result --verbose`. Verbose console output: `colcon test --event-handlers console_cohesion+` ([Humble Writing-Basic-Tests-Python](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html)).
- **Integration tests via `launch_pytest`** ([launch_pytest — index.ros.org](https://index.ros.org/p/launch_pytest/), [Writing Basic Integration Tests with launch_testing — Humble](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Integration.html)):
  - File-name convention: `test/test_<thing>_launch.py` or `test_*.py` containing both a launch description fixture and tests.
  - The `launch_pytest` plugin manages a `LaunchService` lifetime via fixtures; you write a `launch_description` fixture and tests that use it.
  - Use `@launch_pytest.fixture` to declare the launch and `def test_...(launch_service, ...)` for active tests; `@launch_testing.post_shutdown_test()` for tests that run after node teardown (typical use: assert clean exit codes).
  - For pure rclpy in-process testing, you can bypass launch entirely: `rclpy.init()` in a fixture, instantiate the Node, drive it with `rclpy.spin_once(node, timeout_sec=0.1)`, and assert on captured publisher output via a sibling subscriber node.
- **Upstream demos as reference** — `ros2/demos`, `ros2/examples`, and `ros2/system_tests` repos are the canonical places to see real test patterns (`launch_testing`, integration scenarios with multiple processes, post-shutdown asserts) ([ros2/demos @ humble](https://github.com/ros2/demos/tree/humble)).

---

## 6. Common Anti-Patterns

Things to flag when reviewing real code. Sources cited where the rule is doctrinal.

- **`get_parameter('foo')` without prior `declare_parameter('foo')`** — raises `ParameterNotDeclaredException` in Humble; if `allow_undeclared_parameters=True` is set to "fix" this, the node loses static type inference and command-line parameter validation ([Humble Using-Parameters-In-A-Class-Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)).
- **Hard-coded magic numbers that should be parameters** — speed limits, timeouts, frame names, thresholds. The discriminator: anything you might tune between bench, sim, and field deployment must be a parameter.
- **`time.sleep()` inside a callback** ([rclpy executors](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)) — under `SingleThreadedExecutor` this blocks the entire node; no other callback runs, no incoming messages are processed, no parameter changes take effect. Use timers or async patterns. The same applies to blocking `service.call()` from a callback — call asynchronously with `call_async()` and chain `.add_done_callback()`.
- **Synchronous `service.call()` while spinning** — deadlocks under SingleThreaded because the call awaits a response that another callback in the same executor is supposed to deliver. Use `call_async()` or move to a `MultiThreadedExecutor` with the service in a separate `ReentrantCallbackGroup` ([Humble About-Executors](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)).
- **No exception handling around `tf_buffer.lookup_transform`** — raises `LookupException`, `ConnectivityException`, `ExtrapolationException` (all from `tf2_ros`); the canonical pattern catches all three and either logs throttled and returns or retries ([rclpy tf2 cookbook](https://github.com/mikeferguson/ros2_cookbook/blob/main/rclpy/tf2.md)). Specifically:
  ```python
  try:
      tf = self.buffer.lookup_transform(target, source, rclpy.time.Time())
  except (LookupException, ConnectivityException, ExtrapolationException) as e:
      self.get_logger().warn(f'tf {source}->{target}: {e}', throttle_duration_sec=1.0)
      return
  ```
- **Mutable default arguments** — generic Python footgun, doubly painful in callbacks: `def cb(self, msg, history=[]):` keeps state across instances. Use `history=None` + check inside.
- **Bare `print()` instead of `get_logger()`** — bypasses /rosout, skips per-level filtering, breaks colored output and log file capture.
- **No QoS profile passed to `create_publisher` / `create_subscription`** when interacting with sensor topics or latched topics — defaults to reliable + volatile + depth 10, which silently fails to match the upstream sensor's `qos_profile_sensor_data` (best_effort) or a latched publisher's `transient_local`. Always explicit when crossing a QoS boundary ([Humble About-Quality-of-Service-Settings](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)).
- **Spinning a sub-node from inside a callback** — calling `rclpy.spin_until_future_complete(other_node, fut)` from a callback of the *same* executor deadlocks. Use a different executor instance or `call_async` with a done-callback.
- **Setting parameters without re-validating** — a parameter callback that just stores the new value into `self.foo = p.value` without bounds-checking lets a user `ros2 param set ... rate_hz -5` and crash the node. Validate every parameter every time ([rclpy parameters cookbook](https://github.com/mikeferguson/ros2_cookbook/blob/main/rclpy/parameters.md)).
- **Forgetting `node.destroy_node()` and `rclpy.shutdown()`** — leaks DDS entities and CycloneDDS shared-memory segments; reproducible "couldn't bind port" errors on relaunch.
- **Not setting `frame_id` / `stamp` on outgoing messages** — `Header` left default means consumers downstream can't transform or temporally correlate. Standard practice: set both at publish time, fail loudly if a source frame is unknown.
- **Putting network I/O, file I/O, or large inference in a callback without offloading** — same blocking problem as `time.sleep`. Heavy work should run in a worker thread or a separate process; the callback should hand off and return.
- **Constructing a node-internal subscriber without storing it on `self`** — `self.create_subscription(...)` returns the subscription handle; assigning it to a local variable lets it be garbage-collected and silently die. Always `self.sub = self.create_subscription(...)`.
- **Passing both `namespace` and `name` to a child launch file's `Node`** when the child file already wraps both — see the zed-ros2-wrapper case in this repo's CLAUDE.md ([ros2/launch_ros node.py @ humble](https://github.com/ros2/launch_ros/blob/humble/launch_ros/launch_ros/actions/node.py)).
- **`Node`s constructed at launch-file module top level** instead of inside `generate_launch_description()` — top-level construction runs at launch-file *import* time, before the LaunchService is ready, and breaks introspection.

---

## Quick Checklist for Reviewers

Boolean items downstream Phase 2 agents can score per package. Aim for ≥ 14/18.

1. [ ] `package.xml` uses `format="3"` and declares `<build_type>ament_python</build_type>`.
2. [ ] `setup.py` installs `package.xml` and the resource marker into `share/`.
3. [ ] `setup.py` installs all launch files via `glob('launch/*.launch.py')` (or equivalent) — no orphan launch files in source-only.
4. [ ] All YAML/URDF/RViz config files are installed via `data_files=` glob entries.
5. [ ] `setup.cfg` has the `[develop]`/`[install]` `script_dir` lines so `ros2 run` finds executables.
6. [ ] Every entry in `entry_points['console_scripts']` resolves to a real `main()` function.
7. [ ] Runtime dependencies live under `<exec_depend>`; test deps live under `<test_depend>`.
8. [ ] License is declared in `package.xml` and matched by a top-level `LICENSE` file; source files have copyright headers (or `test_copyright` is intentionally skipped).
9. [ ] `test/` contains the three boilerplate linters (`test_copyright`, `test_flake8`, `test_pep257`) and they pass under `colcon test`.
10. [ ] Every `Node` subclass calls `super().__init__('name')` and uses `self.get_logger()` (not `print()`).
11. [ ] Every parameter is declared via `declare_parameter(...)` with a default before being read; `allow_undeclared_parameters` is **not** set.
12. [ ] If parameters are runtime-tunable, an `add_on_set_parameters_callback` validates them and returns `SetParametersResult`.
13. [ ] Sensor-stream subscribers use `qos_profile_sensor_data` (or an explicit best_effort + small depth profile) — not the default.
14. [ ] Latched topics (map, robot_description, calibration, LabelInfo) use `transient_local` + `reliable` on **both** publisher and subscriber.
15. [ ] Callbacks contain no `time.sleep()`, no synchronous `service.call()`, no blocking I/O without an offload mechanism.
16. [ ] Every `tf_buffer.lookup_transform` is wrapped in a `try/except` for the three tf2 exceptions, with throttled warning logging.
17. [ ] Launch files use `DeclareLaunchArgument` + `LaunchConfiguration` for tunables; conditional inclusion uses `IfCondition`/`UnlessCondition`; environment variables use `SetEnvironmentVariable`.
18. [ ] Launch files do **not** pass both `namespace=` and `name=` to a child launch file's `Node` when the child already manages those (see zed-ros2-wrapper rule).

---

## Sources (consolidated)

- ROS 2 Humble — Creating Your First ROS 2 Package: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- ROS 2 Humble — Ament CMake Python: https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html
- ROS 2 Humble — Developing a ROS 2 Package: https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html
- ROS 2 Humble — Integrating launch files into ROS 2 packages: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html
- ROS 2 Humble — Using parameters in a class (Python): https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html
- ROS 2 Humble — Writing a simple publisher and subscriber (Python): https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- ROS 2 Humble — About QoS Settings: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- ROS 2 Humble — About Executors: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html
- ROS 2 Humble — Creating a launch file: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
- ROS 2 Humble — Using substitutions: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html
- ROS 2 Humble — Using event handlers: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
- ROS 2 Humble — Writing Basic Tests with Python: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html
- ROS 2 Humble — Writing Integration Tests with launch_testing: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Integration.html
- ROS 2 Humble — Ament Lint CLI Utilities: https://docs.ros.org/en/humble/Tutorials/Advanced/Ament-Lint-For-Clean-Code.html
- design.ros2.org — QoS: https://design.ros2.org/articles/qos.html
- design.ros2.org — Node lifecycle: https://design.ros2.org/articles/node_lifecycle.html
- design.ros2.org — ROS Command Line Arguments: https://design.ros2.org/articles/ros_command_line_arguments.html
- REP 149 — Package Manifest Format Three Specification: https://ros.org/reps/rep-0149.html
- ros2/demos demo_nodes_py setup.py @ humble: https://github.com/ros2/demos/blob/humble/demo_nodes_py/setup.py
- ros2/demos demo_nodes_py package.xml @ humble: https://github.com/ros2/demos/blob/humble/demo_nodes_py/package.xml
- ros2/demos lifecycle_py setup.py @ humble: https://github.com/ros2/demos/blob/humble/lifecycle_py/setup.py
- ros2/launch architecture: https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst
- ros2/launch_ros Node action @ humble: https://github.com/ros2/launch_ros/blob/humble/launch_ros/launch_ros/actions/node.py
- ros2/launch issue #599 (OpaqueFunction context.perform): https://github.com/ros2/launch/issues/599
- mikeferguson/ros2_cookbook rclpy/parameters: https://github.com/mikeferguson/ros2_cookbook/blob/main/rclpy/parameters.md
- mikeferguson/ros2_cookbook rclpy/tf2: https://github.com/mikeferguson/ros2_cookbook/blob/main/rclpy/tf2.md
- The Robotics Back-End — rclpy Parameter Callback: https://roboticsbackend.com/ros2-rclpy-parameter-callback/
- index.ros.org — ament_flake8: https://index.ros.org/p/ament_flake8/
- index.ros.org — ament_copyright: https://index.ros.org/p/ament_copyright/
- index.ros.org — ament_pep257: https://index.ros.org/p/ament_pep257/
- index.ros.org — launch_pytest: https://index.ros.org/p/launch_pytest/
- rclpy Node API (Rolling, applies to Humble): https://docs.ros.org/en/rolling/p/rclpy/rclpy.node.html
