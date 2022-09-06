#!/bin/bash
set -e

# setup ros environment
. "/opt/ros/$ROS_DISTRO/setup.bash"
. "/ros2_ws/install/setup.bash"
echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo ". /ros2_ws/install/setup.bash" >> ~/.bashrc

exec "$@"