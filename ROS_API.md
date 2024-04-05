# ROS API

Detailed information about content of rosbot_xl package for ROS2.

## rosbot_xl_bringup

Use `bringup.launch.py` from `rosbot_xl_bringup` to start all base functionalities for ROSbot XL. It consists of the following parts:

### External Nodes

#### scan_to_scan_filter_chain

`scan_to_scan_filter_chain` from `laser_filters`, it subscribes to `/scan` topic and removes all points that are within the robot's footprint (defined by config `laser_filter.yaml` in `rosbot_xl_bringup` package). Filtered laser scan is then published on `/scan_filtered` topic

##### Subscribes

- `/scan` (_sensor_msgs/LaserScan_)

##### Publishes

- `/scan_filtered` (_sensor_msgs/LaserScan_)

#### ekf_node

`ekf_node` from `robot_localization`, it is used to fuse wheel odometry and IMU data. Parameters are defined in `ekf.yaml` in `rosbot_xl_bringup/config`. It subscribes to `/rosbot_xl_base_controller/odom` and `/imu_broadcaster/imu` published by ros2 controllers and publishes fused odometry on `/odometry/filtered` topic

##### Subscribes

- `/rosbot_xl_base_controller/odom` (_nav_msgs/Odometry_)
- `/imu_broadcaster/imu` (_sensor_msgs/Imu_)

##### Publishes

- `/tf` (_tf2_msgs/TFMessage_) - `base_link`->`odom` transform
- `/odometry/filtered` (_nav_msgs/Odometry_)

#### micro_ros_agent
Use `micro_ros_agent` to communicate with Digital Board, it provides the following interface:

#### Subscribes

- `/_motors_cmd` (_std_msgs/Float32MultiArray_)

#### Publishes

- `/_motors_responses` (_sensor_msgs/JointState_)
- `/_imu/data_raw` (_sensor_msgs/Imu_)
- `/battery_state` (_sensor_msgs/BatteryState_)

## rosbot_xl_controller

`controller.launch.py` from `rosbot_xl_controller`, it loads robot model defined in `rosbot_xl_description` as well as ros2 control [rosbot_hardware_interfaces](https://github.com/husarion/rosbot_hardware_interfaces). It also starts controllers:

- `joint_state_broadcaster`
- `rosbot_xl_base_controller` - depending on the value of `mecanum` argument it can be `DiffDriveController` or `MecanumDriveController`
- `imu_broadcaster`

#### Subscribes

- `/cmd_vel` (_geometry_msgs/Twist_)
- `/_motors_responses` (_sensor_msgs/JointState_)
- `/_imu/data_raw` (_sensor_msgs/Imu_)

#### Publishes

- `/tf` (_tf2_msgs/TFMessage_)
- `/tf_static` (_tf2_msgs/TFMessage_)
- `/_motors_cmd` (_std_msgs/Float32MultiArray_)
- `/rosbot_xl_base_controller/odom` (_nav_msgs/Odometry_)
- `/imu_broadcaster/imu` (_sensor_msgs/Imu_)
