# ROSbot XL ROS

ROS2 packages for ROSbot XL

## ROS packages

### `rosbot_xl`

Metapackage that contains dependencies to other repositories. It is also used to define whether simulation dependencies should be used.

### `rosbot_xl_bringup`

Package that contains launch, which starts all base functionalities with the microros agent. Also configs for `robot_localization` and `laser_filters` are defined there.

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

### `rosbot_xl_utils`

This package contains the stable firmware version with the flash script.

## ROS API

Available in [ROS_API.md](./ROS_API.md)

## Usage on hardware

To run the software on real ROSbot XL, also communication with Digital Board will be necessary.
First update your firmware to make sure that you use the latest version, then run the `micro-ROS` agent.
For detailed instructions refer to the [rosbot_xl_firmware repository](https://github.com/husarion/rosbot_xl_firmware).

## Prepare environment

1. **Install `ros-dev-tools` and `stm32flash`** (`stm32flash` is not in the ros index and should be installed manually).

    ```bash
    sudo apt-get update
    sudo apt-get install -y ros-dev-tools stm32flash
    ```

2. **Create workspace folder and clone `rosbot_xl_ros` repository.**

    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws
    git clone https://github.com/husarion/rosbot_xl_ros src/
    ```

3. **Specify build type.**

    The repository is used to run the code both on the real robot and in the simulation. Specify `HUSARION_ROS_BUILD_TYPE` the variable according to your needs.

    **Hardware**:

    ``` bash
    export HUSARION_ROS_BUILD_TYPE=hardware
    ```

    **Simulation**:

    ```bash
    export HUSARION_ROS_BUILD_TYPE=simulation
    ```

### Build from source

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_xl/rosbot_xl_$HUSARION_ROS_BUILD.repos

# Copy only diff_drive_controller and imu_sensor_broadcaster, waits for features from ros2-control
cp -r src/ros2_controllers/diff_drive_controller src/
cp -r src/ros2_controllers/imu_sensor_broadcaster src/
rm -rf src/ros2_controllers

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --packages-up-to rosbot-xl --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

> [!NOTE]
> Before starting the software on the robot please make sure that you're using the latest firmware and run the `micro-ROS` agent as described in the [Usage on hardware](#usage-on-hardware) step.

### Running

**Hardware**:

1. Flash firmware.

    ```bash
    # Get admin permissions to flash firmware
    sudo su
    source install/setup.bash
    ros2 run rosbot_xl_utils flash_firmware
    exit
    ```

2. Launch robot

    ```bash
    source install/setup.bash
    ros2 launch rosbot_xl_bringup combined.launch.py
    ```

**Simulation**:

```bash
source install/setup.bash
ros2 launch rosbot_xl_gazebo simulation.launch.py
```

## Developer info

### pre-commit

[pre-commit configuration](.pre-commit-config.yaml) prepares plenty of tests helping for developing and contributing. Usage:

```bash
# install pre-commit
pip install pre-commit

# initialize pre-commit workspace
pre-commit install

# manually run pre-commit hooks
pre-commit run -a

# update revision
pre-commit autoupdate
```

After initialization [pre-commit configuration](.pre-commit-config.yaml) will applied on every commit.

### Industrial CI

```bash
colcon test
```

> [!NOTE]
> Command `colcon test` does not build the code. Remember to build your code after changes.

If tests finish with errors print logs:

```bash
colcon test-result --verbose
```

## Demos

For further usage examples check out our other repositories:

* [`rosbot-xl-docker`](https://github.com/husarion/rosbot-xl-docker) - Dockerfiles for building hardware and simulation images
* [`rosbot-xl-gamepad`](https://github.com/husarion/rosbot-xl-gamepad) - simple teleoperation using a gamepad
* [`rosbot-xl-mapping`](https://github.com/husarion/rosbot-xl-mapping) - creating a map of the environment using slam_toolbox
* [`rosbot-xl-navigation`](https://github.com/husarion/rosbot-xl-navigation) - autonomous navigation demo based on Nav2 and AMCL
* [`rosbot-xl-manipulation`](https://github.com/husarion/rosbot-xl-manipulation) - integration of ROSbot XL with OpenManipulatorX
