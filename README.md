# ROSbot XL ROS

ROS2 packages for ROSbot XL

## ROS API

You can find ROS API and detailed package description in [ROS_API.md](./ROS_API.md).

## Usage on hardware

To run the software on real ROSbot XL, also communication with Digital Board will be necessary.
First update your firmware to make sure that you use the latest version, then run the `micro-ROS` agent.
For detailed instructions refer to the [rosbot_xl_firmware repository](https://github.com/husarion/rosbot_xl_firmware).

## Prepare environment

1. **Install `colcon`, `vsc` and `rosdep`**
```
sudo apt-get update
sudo apt-get install -y ros-dev-tools stm32flash
```

2. **Create workspace folder and clone `rosbot_xl_ros` repository:**
```
mkdir -p ros2_ws/src
cd ros2_ws
git clone https://github.com/husarion/rosbot_xl_ros src/
```

### Build and run on hardware

1. **Building**
```
export HUSARION_ROS_BUILD=hardware

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos

# Remove tests from micro_ros_msgs
sed '/if(BUILD_TESTING)/,/endif()/d' src/micro_ros_msgs/CMakeLists.txt -i

rm -r src/rosbot_xl_gazebo

# Copy only diff_drive_controller and imu_sensor_broadcaster, waits for features from ros2-control
cp -r src/ros2_controllers/diff_drive_controller src/
cp -r src/ros2_controllers/imu_sensor_broadcaster src/
rm -rf src/ros2_controllers

# stm32flash is not in the ros index and should be installed manually
sudo apt install stm32flash

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build
```

> [!NOTE]
> Before starting the software on the robot please make sure that you're using the latest firmware and run the `micro-ROS` agent as described in the [Usage on hardware](#usage-on-hardware) step.

2. **Running**

Flash firmware.
```bash
# Get admin permissions to flash firmware
sudo su
source install/setup.bash
ros2 run rosbot_xl_utils flash_firmware
exit
```

```
source install/setup.bash
ros2 launch rosbot_xl_bringup combined.launch.py
```

### Build and run Gazebo simulation

1. **Building**
```
export HUSARION_ROS_BUILD=simulation

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos
vcs import src < src/rosbot_xl/rosbot_xl_simulation.repos

# Remove tests from micro_ros_msgs
sed '/if(BUILD_TESTING)/,/endif()/d' src/micro_ros_msgs/CMakeLists.txt -i

# Copy only diff_drive_controller and imu_sensor_broadcaster, waits for features from ros2-control
cp -r src/ros2_controllers/diff_drive_controller src/
cp -r src/ros2_controllers/imu_sensor_broadcaster src/
rm -rf src/ros2_controllers

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build
```

2. **Running**
```
source install/setup.bash
ros2 launch rosbot_xl_gazebo simulation.launch.py
```
## Testing package

### pre-commit
[pre-commit configuration](.pre-commit-config.yaml) prepares plenty of tests helping for developing and contributing. Usage:

```bash
# install pre-commit
pip install pre-commit

# initialize pre-commit workspace
pre-commit install

# manually run tests
pre-commit run -a

# update revision
pre-commit autoupdate
```

After initialization [pre-commit configuration](.pre-commit-config.yaml) will applied on every commit.

### Industrial CI
```
colcon test
```

> [!NOTE]
> Command `colcon test` does not build the code. Remember to build your code after changes.

If tests finish with errors print logs:
```
colcon test-result --verbose
```

### Format python code with [Black](https://github.com/psf/black)
```
cd src/
black rosbot*
```


## Testing package

### Industrial CI
```
colcon test
```

> [!NOTE]
> Command `colcon test` does not build the code. Remember to build your code after changes.

If tests finish with errors print logs:
```
colcon test-result --verbose
```

### Format python code with [Black](https://github.com/psf/black)
```
cd src/
black rosbot*
```

## Demos

For further usage examples check out our other repositories:
* [`rosbot-xl-docker`](https://github.com/husarion/rosbot-xl-docker) - Dockerfiles for building hardware and simulation images
* [`rosbot-xl-gamepad`](https://github.com/husarion/rosbot-xl-gamepad) - simple teleoperation using a gamepad
* [`rosbot-xl-mapping`](https://github.com/husarion/rosbot-xl-mapping) - creating a map of the environment using slam_toolbox
* [`rosbot-xl-navigation`](https://github.com/husarion/rosbot-xl-navigation) - autonomous navigation demo based on Nav2 and AMCL
* [`rosbot-xl-manipulation`](https://github.com/husarion/rosbot-xl-manipulation) - integration of ROSbot XL with OpenManipulatorX
