#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  rosbot_xl_description = get_package_share_directory('rosbot_xl_description')
  rosbot_xl_gazebo = get_package_share_directory('rosbot_xl_gazebo')
  world = LaunchConfiguration('world', default=os.path.join(rosbot_xl_gazebo, 'worlds', 'turtlebot_playground.world'))

  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([rosbot_xl_gazebo, '/launch/world.launch.py']),
      launch_arguments = {
        'world' : world,
      }.items(),
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([rosbot_xl_gazebo, '/launch/spawn_robot.launch.py']),
      launch_arguments = {
        'pos_x' : '0.0',
        'pos_y' : '0.0',
      }.items(),
    ),
  ])


if __name__ == '__main__':
  generate_launch_description()