#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/app/ros2_ws/install/setup.bash"

echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo ". /app/ros2_ws/install/setup.bash" >> ~/.bashrc

exec "$@"