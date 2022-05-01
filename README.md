# ROSbot XL ROS

ROS2 packages for ROSbot XL.

## ROS packages

### `rosbot_xl_description`

URDF model used for both simulation and as a source of transforms on physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

Available models:

| Model | Description |
| - | - |
| `rosbot_xl` | Final configuration of rosbot_xl with ability to attach external hardware. |
| `rosbot_xl_base` | Base of rosbot prepared to be included into preexisting configuration. Meant to be compatible with concept of ROS Industrial ability for manipulators to have interchangeable end effectors. |


### `rosbot_xl_ekf`

Draft Kalman filter configuration for a ROSbot XL. Currently inputs odometry published by microros and creates transform between `/odom` and `/base_link`. IMU not implemented. Covariances were not tweaked.

### `rosbot_xl_ignition`

Launch files for Ignition Gazebo working with ROS2 control.

### `rosbot_xl_hardware`

ROS2 hardware controller for ROSbot XL. Inputs and outputs data from ROS2 control and forwards it via ROS topic to be read by microros. Current state: controller compiles and loads. Crashes in runtime.


## Docker Image

Official ROSbot XL docker images built from this repo are available here: https://hub.docker.com/r/husarion/rosbot-xl/tags

- `husarion/rosbot-xl:galactic` - the image for a real (physical) robot
- `husarion/rosbot-xl:galactic-simulation` - the image with built-in Gazebo simulation model

## Flashing the firmware on STM32

Connect your laptop to Micro USB port on the ROSbot XL digital board (with STM32F4), check USB port in your OS with a serial connection to the board (in most cases `/dev/ttyUSB0`).

Set dip switch no. 3 on ROSbot XL digital board to **"on" state** (`BOOT0` pin to HIGH) and click the `RESET` button, to enter the programming mode.

Execute in a termianl on your laptop:

```bash
docker run --rm -it \
--device /dev/ttyUSB0:/dev/ttyUSB0 \
husarion/rosbot-xl:galactic \
/stm32flash -w /firmware.bin -b 115200 -v /dev/ttyUSB0
```

Set dip switch no. 3 to **"off" state**  (`BOOT0` pin to LOW) and click the `RESET` button to start a newly flashed firmware.

## Demo

> **Prerequisites**
>
> Make sure you have [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) and [Docker Compose v2](https://docs.docker.com/compose/cli-command/#install-on-linux) installed on your laptop. Tested on Ubuntu 20.04.
>
> If you don't have, here's a quick summary for Ubuntu 20.04:
> 
> 1. Installing Docker (just click the `copy` button, and paste it to the Linux terminal):
>     ```bash
>     sudo apt-get update && sudo apt-get install -y ca-certificates curl gnupg lsb-release
>     ```
>     ```bash
>     curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
>     ```
>     ```bash
>     echo \
>     "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
>     $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
>     ```
>     ```bash
>     sudo apt-get update && sudo apt-get install docker-ce docker-ce-cli containerd.io
>     ```
>
> 2. Installing Docker Compose v2
>     ```bash
>     mkdir -p /usr/local/lib/docker/cli-plugins
>     ```
>     ```bash
>     curl -SL https://github.com/docker/compose/releases/download/v2.2.3/docker-compose-linux-x86_64 -o /usr/local/lib/docker/cli-plugins/docker-compose
>     ```
>     ```bash
>     chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
>     ```
>
> The proper version of Docker and Docker Compose are already installed in the official ROSbot XL system image.

### Control ROSbot XL in LAN from RViz running on your laptop (Nav2 based)

Connect your ROSbot XL and laptop to the same Wi-Fi network, navigate to `demo/` folder and execute:

- On laptop:

    ```bash
    xhost local:root
    ```

    ```bash
    docker compose -f compose.pc.yaml -f compose.pc.lan.yaml up
    ```

- On ROSbot XL:

    ```bash
    docker compose -f compose.rosbot.yaml -f compose.rosbot.lan.yaml up
    ```

### Control ROSbot XL over the Internet from RViz running on your laptop (Nav2 based)

**This setup is not finished yet!!!**

Login at https://app.husarnet.com, create a new network, copy a **Join Code** a place it in `demo/.env` file:

```bash
HUSARNET_JOINCODE=fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/xxxxxxxxxxxxxxxxxxxxxx
```

Do that both on your laptop and ROSbot XL (they don't need to be in the same LAN)

#### Steps to do on ROSbot XL

In the ROSbot XL terminal navigate to `demo/` folder and execute:

```bash
docker run --rm -it husarnet/husarnet:latest husarnet genid > id
```

to generate unique Husarnet ID for ROSbot XL. Print ROSbot XL Husarnet IPv6 address with this command:

```
sed -r 's/([a-f0-9:]*)\s.*/\1/g' id
```

and paste it to `dds-config.client.xml` and `dds-config.server.xml` files here:

```xml
<locator>
    <udpv6>
        <address>fc94:2b69:4b3d:68d0:2da7:99ea:1340:989f</address>
        <port>11811</port>
    </udpv6>
</locator>
```

Now you can launch the docker compose stack:

```bash
docker compose -f compose.rosbot.yaml -f compose.rosbot.husarnet.yaml up
```

#### Steps to do on your laptop

paste ROSbot XL IPv6 address from the prvious steps to `dds-config.client.xml` and `dds-config.server.xml` files here:

```xml
<locator>
    <udpv6>
        <address>fc94:2b69:4b3d:68d0:2da7:99ea:1340:989f</address>
        <port>11811</port>
    </udpv6>
</locator>
```

And execute in the Linux terminal

```bash
xhost local:root
```

```bash
docker compose -f compose.pc.yaml -f compose.pc.husarnet.yaml up
```
