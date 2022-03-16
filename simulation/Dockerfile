FROM osrf/ros:galactic-desktop

# select bash as default shell
SHELL ["/bin/bash", "-c"]

# install dependencies
RUN apt update && apt upgrade -y && apt install -y \
    python3-pip \
    python3-colcon-common-extensions
    
# should be installed by rosdep
# ros-$ROS_DISTRO-ros2-control \
# ros-$ROS_DISTRO-ros2-controllers \
# ros-$ROS_DISTRO-xacro \
# ros-$ROS_DISTRO-gazebo-ros-pkgs \
# ros-$ROS_DISTRO-geographic-msgs \
# ros-$ROS_DISTRO-robot-localization \
# ros-$ROS_DISTRO-joint-state-publisher \
# ros-$ROS_DISTRO-joint-state-publisher-gui

# install everything inside /app folder
WORKDIR /app

# create ros2_ws
RUN mkdir -p ros2_ws/src

# copy packages
COPY ./src /app/ros2_ws/src

# build workspace
RUN cd ros2_ws \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y \
    && colcon build --symlink-install
    
# clear ubuntu packages
RUN apt clean && \
    rm -rf /var/lib/apt/lists/*

# replace entrypoint
COPY ./simulation/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

