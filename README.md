# ROSbot XL ROS

ROS2 packages for ROSbot XL

## ROS API

You can find ROS API and detailed package description in [ROS_API.md](./ROS_API.md).

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

## Build and run on hardware

1. **Build the package**

    ```bash
    export HUSARION_ROS_BUILD=hardware

    source /opt/ros/$ROS_DISTRO/setup.bash

    vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos

    rm -r src/rosbot_xl_gazebo

    # Copy only diff_drive_controller and imu_sensor_broadcaster, waits for features from ros2-control
    cp -r src/ros2_controllers/diff_drive_controller src/
    cp -r src/ros2_controllers/imu_sensor_broadcaster src/
    rm -rf src/ros2_controllers

    rosdep init
    rosdep update --rosdistro $ROS_DISTRO
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

2. **Running**

    ```bash
    source install/setup.bash
    ros2 launch rosbot_xl_bringup bringup.launch.py
    ```

> [!IMPORTANT]
> Whenever the software version is changed, it is recommended to update the firmware version to ensure that the package version is compatible with the firmware version.
>
> ```bash
> sudo su # Get admin permissions to flash firmware
> source install/setup.bash
> ros2 run rosbot_xl_utils flash_firmware
> exit
> ```

## Build and run Gazebo simulation

1. **Build the package**

    ```bash
    export HUSARION_ROS_BUILD=simulation

    source /opt/ros/$ROS_DISTRO/setup.bash

    vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos
    vcs import src < src/rosbot_xl/rosbot_xl_simulation.repos

    # Copy only diff_drive_controller and imu_sensor_broadcaster, waits for features from ros2-control
    cp -r src/ros2_controllers/diff_drive_controller src/
    cp -r src/ros2_controllers/imu_sensor_broadcaster src/
    rm -rf src/ros2_controllers

    rosdep init
    rosdep update --rosdistro $ROS_DISTRO
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

2. **Running**

    ```bash
    source install/setup.bash
    ros2 launch rosbot_xl_gazebo simulation.launch.py
    ```

## Demos

For further usage examples check out our other repositories:

* [`rosbot-xl-docker`](https://github.com/husarion/rosbot-xl-docker) - Dockerfiles for building hardware and simulation images
* [`rosbot-xl-gamepad`](https://github.com/husarion/rosbot-xl-gamepad) - simple teleoperation using a gamepad
* [`rosbot-xl-autonomy`](https://github.com/husarion/rosbot-xl-autonomy) - autonomous navigation & mapping based on Nav2
* [`rosbot-xl-telepresence`](https://github.com/husarion/rosbot-xl-telepresence) - control and receive signals in real time from any device across the Internet using Husarnet VPN
* [`rosbot-xl-manipulation`](https://github.com/husarion/rosbot-xl-manipulation) - integration of ROSbot XL with OpenManipulatorX
