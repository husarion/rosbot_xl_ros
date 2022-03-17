FROM osrf/ros:galactic-desktop

# select bash as default shell
SHELL ["/bin/bash", "-c"]

# install dependencies
RUN apt update && apt upgrade -y && apt install -y \
    python3-pip \
    python3-colcon-common-extensions

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

