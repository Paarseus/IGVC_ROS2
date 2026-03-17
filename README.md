# AVROS — Autonomous Vehicle ROS 2 Platform

**Author: Parsa Ghasemi**

A ROS 2 (Jazzy/Humble) autonomous vehicle stack for a custom drive-by-wire platform, featuring GPS-based route planning, LiDAR obstacle avoidance, and full Nav2 integration.

## Demo

![AVROS Navigation Demo](docs/avros_nav_demo.gif)

*Autonomous navigation on a simulated campus road network. The vehicle receives a GPS destination, computes a route via the campus road graph, and follows the path using Regulated Pure Pursuit with real-time LiDAR-based obstacle avoidance.*

## Features

- **GPS waypoint navigation** — Send a destination as GPS coordinates; the route planner finds the optimal path through a pre-built campus road graph
- **Nav2 integration** — SmacPlannerHybrid (Ackermann-aware), Regulated Pure Pursuit controller, and behavior tree orchestration
- **Sensor fusion** — Dual EKF (local + global) fusing IMU, GPS, and wheel odometry via `robot_localization`
- **LiDAR obstacle avoidance** — Velodyne VLP-16 with VoxelLayer costmaps for real-time 3D obstacle detection
- **Drive-by-wire control** — PID speed control with Ackermann steering, communicating over UDP to a Teensy actuator MCU
- **Web UI** — Phone-based joystick controller (FastAPI + WebSocket) for manual bench testing
- **Webots simulation** — Full campus environment with Car PROTO, simulated sensors, and Nav2 for development and validation

## Architecture

| Package | Description |
|---------|-------------|
| `avros_msgs` | Custom message/service definitions (ActuatorCommand, ActuatorState, PlanRoute) |
| `avros_bringup` | Launch files, URDF, YAML configs, Nav2 params, RViz config |
| `avros_control` | Actuator node: `cmd_vel` → PID → Ackermann inverse kinematics → Teensy UDP |
| `avros_webui` | Phone joystick WebSocket → direct actuator control |
| `avros_navigation` | Offline campus road graph generator (OSMnx → GeoJSON) |
| `avros_sim` | Webots simulation with campus world, vehicle driver, and sensor emulation |

## Hardware

- **Compute:** NVIDIA Jetson Orin (JetPack 6)
- **LiDAR:** Velodyne VLP-16
- **Camera:** Intel RealSense D455
- **IMU/GNSS:** Xsens MTi-680G with RTK corrections (NTRIP)
- **Actuator:** Teensy MCU over CAN bus (steering, throttle, brake, gear)

## Quick Start

```bash
# Build
cd ~/AVROS
colcon build --symlink-install --packages-select avros_msgs
colcon build --symlink-install

# Launch simulation
ros2 launch avros_sim sim_navigation.launch.py

# Send a GPS navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 108.09, y: 198.93}}}}"
```

## Author

**Parsa Ghasemi**
GitHub: [@Paarseus](https://github.com/Paarseus)
