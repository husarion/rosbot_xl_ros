ros2 topic pub /box_goal std_msgs/msg/String "data: box 1"
ros2 topic pub /box_goal std_msgs/msg/String "data: box 2"
ros2 topic pub -1 /finished_exploration std_msgs/msg/Bool "{data: true}"
rviz2 rviz2
ros2 topic list
