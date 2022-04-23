#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_ws/install/setup.bash"

# setup Fast DDS RMW
# if [ "$RMW_IMPLEMENTATION" == "rmw_fastrtps_cpp" ]; then
    source "/fastdds_overlay/install/setup.bash"
# fi

# GAZEBO_SETUP_PATH=/usr/share/gazebo/setup.sh
# if [[ -f "$GAZEBO_SETUP_PATH" ]]; then
#     source "$GAZEBO_SETUP_PATH"
# fi

echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo ". /ros2_ws/install/setup.bash" >> ~/.bashrc

exec "$@"