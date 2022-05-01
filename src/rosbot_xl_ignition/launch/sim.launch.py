#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
  
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  
  use_sim = use_sim_time
  
  rosbot_xl_description = get_package_share_directory('rosbot_xl_description')
  xacro_file = os.path.join(rosbot_xl_description, 'models', 'rosbot_xl', 'rosbot_xl.urdf.xacro')
  
  robot_description = {
    'robot_description' : Command([
      'xacro --verbosity 0 ', xacro_file,
      ' use_sim:=', use_sim
    ])
  }
  
  robot_state_pub_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[
      {'use_sim_time': use_sim_time},
      robot_description
    ]
  )
  
  control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[robot_description,],
    output={
        'stdout': 'screen',
        'stderr': 'screen',
    },
    remappings=[
      ('/rosbot_xl_base_controller/cmd_vel_unstamped', '/cmd_vel')
    ]
  )


  joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
      'joint_state_broadcaster',
      '--controller-manager',
      '/controller_manager'
    ]
  )

  robot_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        'rosbot_xl_base_controller',
        '--controller-manager',
        '/controller_manager',
    ]
  )

  ign_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
      launch_arguments={'ign_args': '-r -v 4 empty.sdf'}.items(),
  )
  
  # Ign - ROS Bridge
  bridge = Node(
    package='ros_ign_bridge',
    executable='parameter_bridge',
    arguments=['/imu/data_raw@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
    output='screen'
  )
  
  spawn = Node(
    package='ros_ign_gazebo',
    executable='create',
    arguments=[
      '-name', 'rosbot_xl',
      '-allow_renaming', 'true',
      '-topic', 'robot_description'
    ],
    output='screen',
  )
  
  delay_controllers_after_gazebo_spawner = (
    RegisterEventHandler(
      event_handler=OnProcessExit(
        target_action=spawn,
        on_exit=[
          control_node,
          joint_state_broadcaster_spawner
        ]
      )
    )
  )
  
  delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
    RegisterEventHandler(
      event_handler=OnProcessExit(
        target_action=joint_state_broadcaster_spawner,
        on_exit=[robot_controller_spawner]
      )
    )
  )

  return LaunchDescription([
    robot_state_pub_node,
    bridge,
    ign_gazebo,
    spawn,
    delay_controllers_after_gazebo_spawner,
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
  ])

if __name__ == '__main__':
    generate_launch_description()