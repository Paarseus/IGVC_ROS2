# diff_drive_controller

Controller for mobile robots with differential drive.

As input it takes velocity commands for the robot body, which are translated
to wheel commands for the differential drive base.

Odometry is computed from hardware feedback and published.

For an introduction to mobile robot kinematics and the nomenclature used here,
see [Wheeled Mobile Robot
Kinematics](../../doc/mobile_robot_kinematics.html#mobile-robot-kinematics).

## Other features

>   * Realtime-safe implementation.
>
>   * Odometry publishing
>
>   * Task-space velocity, acceleration and jerk limits
>
>   * Automatic stop after command time-out
>
>

## Description of controller’s interfaces

### References

(the controller is not yet implemented as chainable controller)

### Feedback

As feedback interface type the joints’ position
(`hardware_interface::HW_IF_POSITION`) or velocity
(`hardware_interface::HW_IF_VELOCITY`,if parameter `position_feedback=false`)
are used.

### Output

Joints’ velocity (`hardware_interface::HW_IF_VELOCITY`) are used.

## ROS 2 Interfaces

### Subscribers

~/cmd_vel [geometry_msgs/msg/TwistStamped]

    

Velocity command for the controller, if `use_stamped_vel=true`. The controller
extracts the x component of the linear velocity and the z component of the
angular velocity. Velocities on other components are ignored.

~/cmd_vel_unstamped [geometry_msgs::msg::Twist]

    

Velocity command for the controller, if `use_stamped_vel=false`. The
controller extracts the x component of the linear velocity and the z component
of the angular velocity. Velocities on other components are ignored.

### Publishers

~/odom [nav_msgs::msg::Odometry]

    

This represents an estimate of the robot’s position and velocity in free
space.

/tf [tf2_msgs::msg::TFMessage]

    

tf tree. Published only if `enable_odom_tf=true`

~/cmd_vel_out [geometry_msgs/msg/TwistStamped]

    

Velocity command for the controller, where limits were applied. Published only
if `publish_limited_velocity=true`

### Parameters

This controller uses the
[generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)
to handle its parameters. The parameter [definition file located in the src
folder](https://github.com/ros-
controls/ros2_controllers/blob/humble/diff_drive_controller/src/diff_drive_controller_parameter.yaml)
contains descriptions for all the parameters used by the controller.

left_wheel_names (string_array)

    

Names of the left side wheels’ joints

Default: {}

Constraints:

>   * parameter is not empty
>
>

right_wheel_names (string_array)

    

Names of the right side wheels’ joints

Default: {}

Constraints:

>   * parameter is not empty
>
>

wheel_separation (double)

    

Shortest distance between the left and right wheels. If this parameter is
wrong, the robot will not behave correctly in curves.

Default: 0.0

Constraints:

>   * greater than 0.0
>
>

wheels_per_side (int)

    

Number of wheels on each side of the robot. This is important to take the
wheels slip into account when multiple wheels on each side are present. If
there are more wheels then control signals for each side, you should enter
number for control signals. For example, Husky has two wheels on each side,
but they use one control signal, in this case ‘1’ is the correct value of the
parameter.

Default: 0

wheel_radius (double)

    

Radius of a wheel, i.e., wheels size, used for transformation of linear
velocity into wheel rotations. If this parameter is wrong the robot will move
faster or slower than expected.

Default: 0.0

Constraints:

>   * greater than 0.0
>
>

wheel_separation_multiplier (double)

    

Correction factor when the actual wheel separation differs from the nominal
value in the `wheel_separation` parameter.

Default: 1.0

left_wheel_radius_multiplier (double)

    

Correction factor when radius of left wheels differs from the nominal value in
`wheel_radius` parameter.

Default: 1.0

right_wheel_radius_multiplier (double)

    

Correction factor when radius of right wheels differs from the nominal value
in `wheel_radius` parameter.

Default: 1.0

tf_frame_prefix_enable (bool)

    

Enables or disables appending tf_prefix to tf frame id’s.

Default: true

tf_frame_prefix (string)

    

(optional) Prefix to be appended to the tf frames, will be added to odom_id
and base_frame_id before publishing. If the parameter is empty, controller’s
namespace will be used.

Default: “”

odom_frame_id (string)

    

Name of the frame for odometry. This frame is parent of `base_frame_id` when
controller publishes odometry.

Default: “odom”

base_frame_id (string)

    

Name of the robot’s base frame that is child of the odometry frame.

Default: “base_link”

pose_covariance_diagonal (double_array)

    

Odometry covariance for the encoder output of the robot for the pose. These
values should be tuned to your robot’s sample odometry data, but these values
are a good place to start: `[0.001, 0.001, 0.001, 0.001, 0.001, 0.01]`.

Default: {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

twist_covariance_diagonal (double_array)

    

Odometry covariance for the encoder output of the robot for the speed. These
values should be tuned to your robot’s sample odometry data, but these values
are a good place to start: `[0.001, 0.001, 0.001, 0.001, 0.001, 0.01]`.

Default: {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

open_loop (bool)

    

If set to true the odometry of the robot will be calculated from the commanded
values and not from feedback.

Default: false

position_feedback (bool)

    

Is there position feedback from hardware.

Default: true

enable_odom_tf (bool)

    

Publish transformation between `odom_frame_id` and `base_frame_id`.

Default: true

cmd_vel_timeout (double)

    

Timeout in seconds after which a command received on the `cmd_vel` topic is
considered stale. A value of 0.0 disables the timeout.

Default: 0.5

publish_limited_velocity (bool)

    

Publish limited velocity value.

Default: false

velocity_rolling_window_size (int)

    

Size of the rolling window for calculation of mean velocity use in odometry.

Default: 10

use_stamped_vel (bool)

    

Use stamp from input velocity message to calculate how old the command
actually is.

Default: true

publish_rate (double)

    

Publishing rate (Hz) of the odometry and TF messages.

Default: 50.0

linear.x

    

Joint limits structure for the linear `x`-axis. The limiter ignores position
limits. For details see `joint_limits` package from ros2_control repository.

linear.x.has_velocity_limits (bool)

    

Default: false

linear.x.has_acceleration_limits (bool)

    

Default: false

linear.x.has_jerk_limits (bool)

    

Default: false

linear.x.max_velocity (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

linear.x.min_velocity (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

linear.x.max_acceleration (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

linear.x.min_acceleration (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

linear.x.max_jerk (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

linear.x.min_jerk (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

angular.z

    

Joint limits structure for the rotation about `z`-axis. The limiter ignores
position limits. For details see `joint_limits` package from ros2_control
repository.

angular.z.has_velocity_limits (bool)

    

Default: false

angular.z.has_acceleration_limits (bool)

    

Default: false

angular.z.has_jerk_limits (bool)

    

Default: false

angular.z.max_velocity (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

angular.z.min_velocity (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

angular.z.max_acceleration (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

angular.z.min_acceleration (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

angular.z.max_jerk (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

angular.z.min_jerk (double)

    

Default: std::numeric_limits<double>::quiet_NaN()

An example parameter file for this controller can be found in [the test
directory](https://github.com/ros-
controls/ros2_controllers/blob/humble/diff_drive_controller/test/config/test_diff_drive_controller.yaml):

    
    
    test_diff_drive_controller:
      ros__parameters:
        left_wheel_names: ["left_wheels"]
        right_wheel_names: ["right_wheels"]
    
        wheel_separation: 0.40
        wheels_per_side: 1  # actually 2, but both are controlled by 1 signal
        wheel_radius: 0.02
    
        wheel_separation_multiplier: 1.0
        left_wheel_radius_multiplier: 1.0
        right_wheel_radius_multiplier: 1.0
    
        odom_frame_id: odom
        base_frame_id: base_link
        pose_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        twist_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
        position_feedback: false
        open_loop: true
        enable_odom_tf: true
    
        cmd_vel_timeout: 0.5 # seconds
        publish_limited_velocity: true
        velocity_rolling_window_size: 10
    
        linear.x.has_velocity_limits: false
        linear.x.has_acceleration_limits: false
        linear.x.has_jerk_limits: false
        linear.x.max_velocity: 0.0
        linear.x.min_velocity: 0.0
        linear.x.max_acceleration: 0.0
        linear.x.max_jerk: 0.0
        linear.x.min_jerk: 0.0
    
        angular.z.has_velocity_limits: false
        angular.z.has_acceleration_limits: false
        angular.z.has_jerk_limits: false
        angular.z.max_velocity: 0.0
        angular.z.min_velocity: 0.0
        angular.z.max_acceleration: 0.0
        angular.z.min_acceleration: 0.0
        angular.z.max_jerk: 0.0
        angular.z.min_jerk: 0.0
    

