# ROSbot XL ROS

ROS2 packages for ROSbot XL.

# Docker configurations

## Instructions: How to run software on [real_robot](./compose_robot)

Connect to robot remotely:

``` bash
ssh ubuntu@<robot_ip>
```

Start docker containers:

``` bash
cd rosbot_xl_ros/compose_robot/
docker compose up
```

**Important: for now it is required to reset powerboard manualy**

On PC:

Launch web browser and connect to: `http://<robot_ip>:6080/`

Open terminal isnside vnc and run rviz

``` bash
rviz2 rvzi2
```

## [simulation](./simulation)

Robot description with gazebo configuration.
``` bash
cd simulation
xhost +local:docker
docker compose up
```

# ROS packages

## rosbot_xl_description

URDF model used for both simulation and as a source of transforms on physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

### Models
#### rosbot_xl
Final configuration of rosbot_xl with ability to attach external hardware.

#### rosbot_xl_base
Base of rosbot prepared to be included into preexisting configuration. Meant to be compatible with concept of ROS Industrial ability for manipulators to have interchangeable end effectors.

## rosbot_xl_ekf

Draft Kalman filter configuration for a ROSbot XL. Currently inputs odometry published by microros and creates transform between `/odom` and `/base_link`. IMU not implemented. Covariances were not tweaked.

## rosbot_xl_gazebo

Ported [panther_gazebo](https://github.com/husarion/panther_simulation/tree/ros2/panther_gazebo) simulation modified to work with ROS2 control.

## rosbot_xl_hardware

ROS2 hardware controller for ROSbot XL. Inputs and outputs data from ROS2 control and forwards it via ROS topic to be read by microros. Current state: controller compiles and loads. Crashes in runtime.