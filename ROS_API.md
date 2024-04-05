# ROSbot XL - Software

Detailed information about content of rosbot_xl package for ROS2.

## Package Description

### `rosbot_xl`

Metapackage that contains dependencies to other repositories. It is also used to define whether simulation dependencies should be used.

### `rosbot_xl_bringup`

Package that contains launch, which starts all base functionalities with the microros agent. Also configs for `robot_localization` and `laser_filters` are defined there.

**Available Launch Files:**

- `bringup.launch.py` - is responsible for activating all logic related to the robot's movement and processing of sensory data.
- `combined.launch.py` - launches `bringup.launch.py` ​​and communication with the firmware allows you to control the robot.

**Launch Params:**

| PARAMETER                | DESCRIPTION                                                       | VALUE (_type_)          |
| ------------------------ | ----------------------------------------------------------------- | ----------------------- |
| **camera_model**         | Add camera model to the robot URDF                                | **"None"**\* (_string_) |
| **lidar_model**          | Add LiDAR model to the robot URDF                                 | **"None"**\* (_string_) |
| **include_camera_mount** | Whether to include camera mount to the robot URDF                 | **False** (_bool_)      |
| **mecanum**              | Whether to use mecanum drive controller, otherwise use diff drive | **False** (_bool_)      |
| **namespace**            | Namespace for all topics and tfs                                  | **""** (_string_)       |

> \* - you can check all available options using `-s`/`--show-args` flag. (e.g. `ros2 launch rosbot_bringup bringup.launch.py -s`).

### `rosbot_xl_controller`

ROS2 hardware controller for ROSbot XL. It manages inputs and outputs data from ROS2 control, forwarding it via ROS topics to be read by microROS. The controller.launch.py file loads the robot model defined in rosbot_xl_description along with ROS2 control dependencies from [rosbot_hardware_interfaces](https://github.com/husarion/rosbot_hardware_interfaces).

### `rosbot_xl_description`

URDF model used for both simulation and as a source of transforms on physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

Available models:

| MODEL            | DESCRIPTION                                                                                                                                                                                  |
| ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `rosbot_xl`      | Final configuration of rosbot_xl with ability to attach external hardware.                                                                                                                   |
| `rosbot_xl_base` | Base of rosbot prepared to be included into preexisting configuration. Meant to be compatible with concept of ROS Industrial ability for manipulators to have interchangeable end effectors. |

### `rosbot_xl_gazebo`

Launch files for Ignition Gazebo working with ROS2 control.

**Available Launch Files:**

- `simulations.launch.py` - running a rosbot in Gazebo simulator and simulate all specified sensors.

**Launch Params:**

| PARAMETER                | DESCRIPTION                                                       | VALUE (_type_)                                            |
| ------------------------ | ----------------------------------------------------------------- | --------------------------------------------------------- |
| **camera_model**         | Add camera model to the robot URDF                                | **"None"**\* (_string_)                                   |
| **lidar_model**          | Add LiDAR model to the robot URDF                                 | **"None"**\* (_string_)                                   |
| **include_camera_mount** | Whether to include camera mount to the robot URDF                 | **False** (_bool_)                                        |
| **mecanum**              | Whether to use mecanum drive controller, otherwise use diff drive | **False** (_bool_)                                        |
| **namespace**            | Namespace for all topics and tfs                                  | **""** (_string_)                                         |
| **world**                | Path to SDF world file                                            | `husarion_office_gz/worlds/husarion_world.sdf` (_string_) |
| **headless**             | Run Gazebo Ignition in the headless mode                          | **False**                                                          |
| **robots**               | List of robots that will be spawn in the simulation            | **[]**\*\*                                                    |

> \*You can check all available options using `-s`/`--show-args` flag. (e.g. `ros2 launch rosbot_bringup bringup.launch.py -s`).
> 
> \*\*Example of use: `robots:='robot1={x: 0.0, y: -1.0}; robot2={x: 1.0, y: -1.0};'`

### `rosbot_xl_utils`

This package contains the stable firmware version with the flash script.

## ROS API

### Available Nodes

- `~/controller_manager`
- `~/ekf_filter_node` [_[robot_localization/ekf_node](https://github.com/cra-ros-pkg/robot_localization)_]: used to fuse wheel odometry and IMU data. Parameters are defined in `rosbot_xl_bringup/config/ekf.yaml`.
- `~/imu_broadcaster` [_[imu_sensor_broadcaster/imu_sensor_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster)_]: The broadcaster to publish readings of IMU sensors.
- `~/imu_sensor_node`
- `~/joint_state_broadcaster` [_[joint_state_broadcaster/joint_state_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster)_]: The broadcaster reads all state interfaces and reports them on specific topics.
- `~/laser_scan_box_filter`
- `~/robot_state_publisher` [_[robot_state_publisher/robot_state_publisher](https://github.com/ros/robot_state_publisher)_]: uses the URDF specified by the parameter robot_description and the joint positions from the topic joint_states to calculate the forward kinematics of the robot and publish the results via tf.
- `~/rosbot_system_node`
- `~/rosbot_xl_base_controller` [_[diff_drive_controller/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller)_]: The controller managing a mobile robot with a differential or omni drive (mecanum wheels). Converts speed commands for the robot body to wheel commands for the base. It also calculates odometry based on hardware feedback and shares it.`DiffDriveController` or `MecanumDriveController`
- `~/scan_to_scan_filter_chain` [_[laser_filters/scan_to_scan_filter_chain](https://github.com/ros-perception/laser_filters/blob/ros2/src/scan_to_scan_filter_chain.cpp)_]: node which subscribes to `/scan` topic and removes all points that are within the robot's footprint (defined by config `laser_filter.yaml` in `rosbot_xl_bringup` package). Filtered laser scan is then published on `/scan_filtered` topic.
- `/stm32_node` [_[micro_ros_agent/micro_ros_agent](https://github.com/micro-ROS/micro-ROS-Agent)_]: node enabling communication with Digital Board, it provides the following interface.

### Available Topics

- `/battery_state` (_sensor_msgs/msg/BatteryState_)
- `~/cmd_vel` (_geometry_msgs/Twist_)
- `/diagnostics` (_diagnostic_msgs/msg/DiagnosticArray_)
- `~/dynamic_joint_states` (_control_msgs/msg/DynamicJointState_)
- `~/imu_broadcaster/imu` (_sensor_msgs/Imu_)
- `~/imu_broadcaster/transition_event` (_lifecycle_msgs/msg/TransitionEvent_)
- `~/joint_state_broadcaster/transition_event` (_lifecycle_msgs/msg/TransitionEvent_)
- `~/joint_states` (_sensor_msgs/msg/JointState_)
- `~/laser_scan_box_filter/transition_event` (_lifecycle_msgs/msg/TransitionEvent_)
- `~/odometry/filtered` (_nav_msgs/Odometry_)
- `~/robot_description` (_std_msgs/msg/String_)
- `~/rosbot_xl_base_controller/odom` (_nav_msgs/Odometry_)
- `~/rosbot_xl_base_controller/transition_event` (_lifecycle_msgs/msg/TransitionEvent_)
- `~/scan` (_sensor_msgs/LaserScan_)
- `~/scan_filtered` (_sensor_msgs/LaserScan_)
- `~/set_pose` (_geometry_msgs/msg/PoseWithCovarianceStamped_)
- `~/tf` (_tf2_msgs/TFMessage_)
- `~/tf_static` (_tf2_msgs/TFMessage_)

**Hidden topic:**

- `/_imu/data_raw` (_sensor_msgs/Imu_)
- `/_motors_cmd` (_std_msgs/Float32MultiArray_)
- `/_motors_responses` (_sensor_msgs/JointState_)
