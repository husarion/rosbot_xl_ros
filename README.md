# ROSbot XL ROS

ROS2 packages for ROSbot XL

## ROS packages

### `rosbot_xl`

Metapackeage that contains dependencies to other repositories. It is also used to define whether simulation dependencies should be used. 

### `rosbot_xl_bringup`

Package that contains launch, which starts all base functionalities. Also configs for `robot_localization` and `laser_filters` are defined there.

### `rosbot_xl_description`

URDF model used for both simulation and as a source of transforms on physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

Available models:

| Model            | Description                                                                                                                                                                                  |
| ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `rosbot_xl`      | Final configuration of rosbot_xl with ability to attach external hardware.                                                                                                                   |
| `rosbot_xl_base` | Base of rosbot prepared to be included into preexisting configuration. Meant to be compatible with concept of ROS Industrial ability for manipulators to have interchangeable end effectors. |


### `rosbot_xl_gazebo`

Launch files for Ignition Gazebo working with ROS2 control.

### `rosbot_xl_controller`

ROS2 hardware controller for ROSbot XL. Inputs and outputs data from ROS2 control and forwards it via ROS topic to be read by microros.

## ROS API

Available in [ROS_API.md](./ROS_API.md)

## Source build

### Prerequisites

Install `colcon`, `vsc` and `rosdep`:
```
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-vcstool python3-rosdep
```

Create workspace folder and clone `rosbot_xl_ros` repository:
```
mkdir -p ros2_ws/src
cd ros2_ws
git clone https://github.com/husarion/rosbot_xl_ros src/
```

### Build and run on hardware

Building:
```
export HUSARION_ROS_BUILD=hardware

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos

rm -r src/rosbot_xl_gazebo

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build
```

Running:
```
ros2 launch rosbot_xl_bringup bringup.launch.py
```

### Build and run Gazebo simulation

Building:
```
export HUSARION_ROS_BUILD=simulation

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos
vcs import src < src/rosbot_xl/rosbot_xl_simulation.repos

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build
```

Running:
```
ros2 launch rosbot_xl_gazebo simulation.launch.py
```

## Demos

For further usage examples check out our other repositories:
* [`rosbot-xl-docker`](https://github.com/husarion/rosbot-xl-docker) - Dockerfiles for building hardware and simulation images
* [`rosbot-xl-gamepad`](https://github.com/husarion/rosbot-xl-gamepad) - simple teleoperation using a gamepad
* [`rosbot-xl-mapping`](https://github.com/husarion/rosbot-xl-mapping) - creating a map of the environment using slam_toolbox
* [`rosbot-xl-navigation`](https://github.com/husarion/rosbot-xl-navigation) - autonomous navigation demo based on Nav2 and AMCL
* [`rosbot-xl-manipulation`](https://github.com/husarion/rosbot-xl-manipulation) - integration of ROSbot XL with OpenManipulatorX