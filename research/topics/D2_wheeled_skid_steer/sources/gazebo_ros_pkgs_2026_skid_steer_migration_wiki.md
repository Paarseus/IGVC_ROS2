# ROS 2 Migration: Skid Steer drive

Jump to bottom

Leander Stephen D'Souza edited this page Jun 26, 2022 * [3 revisions](/ros-
simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Skid-Steer-drive/_history)

# Overview

This pages describes the changes for skid steer drive plugin in
`gazebo_plugins` for ROS 2, including a migration guide. The skid steer drive
is integrated into diff drive.

From ROS 2, `gazebo_ros_diff_drive` can be made to behave like skid steer
drive by specifying extra joints and kinematic properties. This way, a single
plugin works for both differential drive and skid steer.

### Summary

  * All SDF parameters are now `snake_cased`
  * Use remapping argument to change default topics (`cmd_vel` and `odom`)
  * Skid steer drive is no longer limited to 4 wheels. Any number of wheels can be specified using `num_wheel_pairs`. The number of wheels including left and right will be double the `num_wheel_pairs`.
  * To specify the left joints, (instead of `leftFrontJoint` and `leftRearJoint`) `left_joint` has to be specified `num_wheel_pairs` of times. The first `left_joint` would be taken as the front one. The second `left_joint` would be the second from front and so on.
  * Similarly right joints have to be specified.
  * Instead of a common `wheelSeparation`, each wheel pair can be assigned a different `wheel_separation`. `wheel_separation` can be specified `num_wheel_pairs` times. The first `wheel_separation` refers to the front most wheel pair. The second `wheel_separation` refers to the second from front and so on.
  * Similarly, different `wheel_diameter`s can be specified.
  * Instead of `broadcastTF`, `publish_wheel_tf` and `publish_odom_tf` can be used to specify independently whether to publish 'wheels TF' or 'odom TF'.

### SDF parameters

ROS 1 | ROS 2  
---|---  
`odometryFrame` | `odometry_frame`  
`updateRate` | `update_rate`  
`torque` | `max_wheel_torque`  
`covariance_x` | `covariance_x`  
`covariance_y` | `covariance_y`  
`covariance_yaw` | `covariance_yaw`  
`commandTopic` | `<ros><remapping>cmd_vel:=custom_cmd_vel</remapping></ros>`  
`odometryTopic` | `<ros><remapping>odom:=custom_odom</remapping></ros>`  
`robotBaseFrame` | `robot_base_frame`  
`wheelSeparation` | `wheel_separation`  
`wheelDiameter` | `wheel_diameter`  
`broadcastTF` |  `publish_wheel_tf` / `publish_odom_tf`  
`leftFrontJoint` | 1st `left_joint`  
`rightFrontJoint` | 1st `right_joint`  
`leftRearJoint` | 2nd `left_joint`  
`rightRearJoint` | 2nd `right_joint`  
  
### Example Migration

#### ROS1

    
    
        <model name='vehicle'>
          ...
    
          <joint name='left_wheel_joint0' type='revolute'>
            ...
          </joint>
    
          <joint name='right_wheel_joint0' type='revolute'>
            ...
          </joint>
    
          <joint name='left_wheel_joint1' type='revolute'>
            ...
          </joint>
    
          <joint name='right_wheel_joint1' type='revolute'>
            ...
          </joint>
    
          <plugin name='skid_steer_drive' filename='libgazebo_ros_skid_steer_drive.so'>
            <updateRate>500</updateRate>
    
            <leftFrontJoint>left_wheel_joint0</leftFrontJoint>
            <rightFrontJoint>right_wheel_joint0</rightFrontJoint>
            <leftRearJoint>left_wheel_joint1</leftRearJoint>
            <rightRearJoint>right_wheel_joint1</rightRearJoint>
    
            <wheelSeparation>1.25</wheelSeparation>
            <wheelDiameter>0.3</wheelDiameter>
    
            <torque>20</torque>
    
            <commandTopic>cmd_demo</commandTopic>
            <odometryTopic>odom_demo</odometryTopic>
            <odometryFrame>odom_demo</odometryFrame>
            <broadcastTF>true</broadcastTF>
          </plugin>
    
        </model>

#### ROS2

    
    
        <model name='vehicle'>
          ...
    
          <joint name='left_wheel_joint0' type='revolute'>
            ...
          </joint>
    
          <joint name='right_wheel_joint0' type='revolute'>
            ...
          </joint>
    
          <joint name='left_wheel_joint1' type='revolute'>
            ...
          </joint>
    
          <joint name='right_wheel_joint1' type='revolute'>
            ...
          </joint>
    
          <plugin name='skid_steer_drive' filename='libgazebo_ros_diff_drive.so'>
            <ros>
              <!-- Set namespace -->
              <namespace>/demo</namespace>
    
              <!-- Remap default topics -->
              <remapping>cmd_vel:=cmd_demo</remapping>
              <remapping>odom:=odom_demo</remapping>
            </ros>
    
            <!-- Update rate -->
            <update_rate>500</update_rate>
    
            <!-- Number of wheel pairs -->
            <num_wheel_pairs>2</num_wheel_pairs>
    
            <!-- wheels0 -->
            <left_joint>left_wheel_joint0</left_joint>
            <right_joint>right_wheel_joint0</right_joint>
    
            <!-- wheels1-->
            <left_joint>left_wheel_joint1</left_joint>
            <right_joint>right_wheel_joint1</right_joint>
    
            <!-- kinematics -->
            <wheel_separation>1.25</wheel_separation>
            <wheel_separation>1.25</wheel_separation>
    
            <wheel_diameter>1.0</wheel_diameter>
            <wheel_diameter>0.6</wheel_diameter>
    
            <!-- limits -->
            <max_wheel_torque>20</max_wheel_torque>
            <max_wheel_acceleration>1.0</max_wheel_acceleration>
    
            <!-- output -->
            <publish_odom>true</publish_odom>
            <publish_odom_tf>true</publish_odom_tf>
            <publish_wheel_tf>true</publish_wheel_tf>
    
            <odometry_frame>odom_demo</odometry_frame>
            <robot_base_frame>chassis</robot_base_frame>
    
          </plugin>
    
        </model>

