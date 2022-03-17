FROM husarion/nav2:galactic-latest

SHELL ["/bin/bash", "-c"]

# RUN apt install -y \    
#     ros-galactic-cyclonedds

# RUN apt remove 

RUN apt update

RUN apt install -y \
    ros-galactic-rmw-fastrtps-cpp

RUN apt-get remove -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp