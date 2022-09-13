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

### `rosbot_xl_hardware`

ROS2 hardware controller for ROSbot XL. Inputs and outputs data from ROS2 control and forwards it via ROS topic to be read by microros.
