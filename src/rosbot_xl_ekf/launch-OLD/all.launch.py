import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rosbot_xl_description = get_package_share_directory('rosbot_xl_description')
    xacro_file = os.path.join(rosbot_xl_description, 'models', 'rosbot_xl', 'rosbot_xl.urdf.xacro')


    print(xacro_file)


    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_description' : Command([
                    'xacro --verbosity 0 ', xacro_file,
                    ' use_sim:=false'])
                }]
        ),

        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui'
        # ),

        Node(
            package="rosbot_xl_kinematics",
            executable="classic_kinematics",
            name="rosbot_xl_classic_kinematics"
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("rosbot_xl_ekf"), 'config', 'ekf.yaml')],
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
        ),
    ])

if __name__ == '__main__':
  generate_launch_description()