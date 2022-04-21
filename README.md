# ROSbot XL ROS

ROS2 packages for ROSbot XL.

## ROS packages

### `rosbot_xl_description`

URDF model used for both simulation and as a source of transforms on physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

### `rosbot_xl_ekf`

Draft Kalman filter configuration for a ROSbot XL. Currently inputs odometry published by microros and creates transform between `/odom` and `/base_link`. IMU not implemented. Covariances were not tweaked.

### `rosbot_xl_gazebo`

Ported [panther_gazebo](https://github.com/husarion/panther_simulation/tree/ros2/panther_gazebo) simulation modified to work with ROS2 control.

### `rosbot_xl_hardware`

ROS2 hardware controller for ROSbot XL. Inputs and outputs data from ROS2 control and forwards it via ROS topic to be read by microros. Current state: controller compiles and loads. Crashes in runtime.

## Models

### rosbot_xl

Final configuration of rosbot_xl with ability to attach external hardware.

### rosbot_xl_base

Base of rosbot prepared to be included into preexisting configuration. Meant to be compatible with concept of ROS Industrial ability for manipulators to have interchangeable end effectors.

## Docker Image

Official ROSbot XL docker images built from this repo are available here: https://hub.docker.com/r/husarion/rosbot-xl/tags

- `husarion/rosbot-xl:galactic` - the image for a real (physical) robot
- `husarion/rosbot-xl:galactic-simulation` - the image with built-in Gazebo simulation model

## Demo

### Control ROSbot XL in LAN from RViz running on your laptop (Nav2 based)

Connect your ROSbot XL and laptop to the same Wi-Fi network and execute:

- On laptop:

    ```bash
    cd demo/
    xhost local:root
    docker compose -f compose.pc.yaml -f compose.pc.lan.yaml up
    ```

- On ROSbot XL:

    ```bash
    cd demo/
    docker compose -f compose.rosbot.yaml -f compose.rosbot.lan.yaml up
    ```

### Control ROSbot XL over the Internet from RViz running on your laptop (Nav2 based)

Login at https://app.husarnet.com, create a new network, copy a **Join Code** a place it in `demo/.env` file:

```bash
# Place your own Join Code below
HUSARNET_JOINCODE=fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/xxxxxxxxxxxxxxxxxxxxxx
```

ROSbot XL and your laptop can be in the same or in different Wi-Fi networks. Execute in their terminals:

- On laptop:

    ```bash
    cd demo/
    xhost local:root
    docker compose -f compose.pc.yaml -f compose.pc.husarnet.yaml up
    ```

- On ROSbot XL:

    ```bash
    cd demo/
    docker compose -f compose.rosbot.yaml -f compose.rosbot.husarnet.yaml up
    ```