FROM ros:galactic-ros-core

# select bash as default shell
SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY . .

# install everything needed
RUN apt-get update && apt-get install -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-$ROS_DISTRO-teleop-twist-keyboard \
        ros-$ROS_DISTRO-rmw-fastrtps-cpp && \
    apt-get upgrade -y && \
    # build & install ROSbot XL packages
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install && \
    # make the image smaller
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep && \       
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ENTRYPOINT ["/ros2_ws/ros_entrypoint.sh"]

