#!/usr/bin/env python3

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

  rosbot_xl_description = get_package_share_directory('rosbot_xl_description')
  xacro_file = os.path.join(rosbot_xl_description, 'models', 'rosbot_xl', 'rosbot_xl.urdf.xacro')
  
  use_sim_time = LaunchConfiguration('use_sim_time', default=True)
  launch_rviz = LaunchConfiguration('launch_rviz', default=True)
  rviz_config = LaunchConfiguration('rviz_config',
    default=os.path.join(rosbot_xl_description, 'rviz/rosbot_xl.rviz'))


  pos_x = LaunchConfiguration('pos_x', default='0.0')
  pos_y = LaunchConfiguration('pos_y', default='0.0')
  pos_z = LaunchConfiguration('pos_z', default='0.0')

  use_sim = use_sim_time

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
    parameters=[robot_description],
    output={
        'stdout': 'screen',
        'stderr': 'screen',
    },
    remappings=[
        ('~/rosbot_xl_base_controller/cmd_vel_unstamped', '/cmd_vel'),
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
  

  gazebo_spawner_node = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    # output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=['-spawn_service_timeout', '600',
                '-entity', 'rosbot_xl',
                '-x', pos_x, '-y', pos_y, '-z', pos_z,
                '-topic', 'robot_description']
  )
  
  rviz_configuration = DeclareLaunchArgument(
    name='rvizconfig',
    default_value=rviz_config,
    description='Absolute path to rviz config file'
  )
  
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', LaunchConfiguration('rvizconfig')],
  )
  
  delay_controllers_after_gazebo_spawner = (
    RegisterEventHandler(
      event_handler=OnProcessExit(
        target_action=gazebo_spawner_node,
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
  
  # dealy_rviz_after_joint_state_broadcaster = (
  #   RegisterEventHandler(
  #     event_handler=OnProcessExit(
  #       target_action=joint_state_broadcaster_spawner,
  #       on_exit=[rviz_configuration, rviz_node]
  #     ),
  #     # condition=IfCondition(LaunchConfiguration('launch_rviz'))
  #   )
  # )


  nodes = [
    robot_state_pub_node,
    gazebo_spawner_node,
    delay_controllers_after_gazebo_spawner,
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    # dealy_rviz_after_joint_state_broadcaster
  ]
  
  return LaunchDescription(nodes)

if __name__ == '__main__':
  generate_launch_description()