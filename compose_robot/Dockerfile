FROM ros:foxy-ros-base

# select bash as default shell
SHELL ["/bin/bash", "-c"]

# install dependencies
RUN apt update && apt upgrade -y \
    && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-xacro

# install everything inside /app folder
WORKDIR /app

# create ros2_ws
RUN mkdir -p ros2_ws/src

# copy packages
COPY ./src/rosbot_xl_bringup /app/ros2_ws/src/rosbot_xl_bringup
COPY ./src/rosbot_xl_ekf /app/ros2_ws/src/rosbot_xl_ekf
# COPY ./src/rosbot_xl_hardware /app/ros2_ws/src/rosbot_xl_hardware
COPY ./src/rosbot_xl_description /app/ros2_ws/src/rosbot_xl_description
COPY ./src/rosbot_xl_kinematics /app/ros2_ws/src/rosbot_xl_kinematics

# build workspace
RUN cd ros2_ws \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep update --rosdistro $ROS_DISTRO \
    && colcon build --symlink-install
    
# clear ubuntu packages
RUN apt clean && \
    rm -rf /var/lib/apt/lists/*

# replace entrypoint
COPY ./compose_robot/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

