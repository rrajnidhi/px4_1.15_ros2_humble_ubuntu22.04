# UAV Simulation with PX4, ROS2 & Micro XRCE-DDS

Environment for running UAV simulations with **PX4**, **ROS2**, **Gazebo**, and **Micro XRCE-DDS**, including the **PX4-ROS2 bridge**.  
---

## Features

- Fully containerized PX4 SITL environment for Ubuntu 22.04
- Homogeneous multi UAV support


## Components Included

| Component           | Version                      | Description                           |
| ------------------- | ---------------------------- | ------------------------------------- |
| **Ubuntu**          | 22.04                        | Base OS                               |
| **ROS 2**           | Humble Hawksbill             | Core ROS2 environment                 |
| **PX4**             | v1.15.0                      | Autopilot firmware                    |
| **Gazebo Sim**      | Garden                       | Simulation environment                |
| **DDS**             | FastDDS                      | Default RMW                           |
| **Micro XRCE-DDS**  | Latest                       | Lightweight DDS Agent                 |
| **PX4 ROS2 Bridge** | 1.15                         | Includes `px4_msgs` and `px4_ros_com` |
| **Tools**           | tmux, nano, colcon           | Development & debugging               |

---

<details>
<summary>Frameworks & Tools Used</summary>

![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?logo=ubuntu)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros)
![PX4](https://img.shields.io/badge/PX4-v1.15.0-lightgrey?logo=px4)
![Gazebo](https://img.shields.io/badge/Gazebo-Garden-green?logo=gazebo)
![Docker](https://img.shields.io/badge/Docker-ready-blue?logo=docker)

</details>


## Directory Structure

```
px4_1.15_ros2_humble_ubuntu22.04/
├── build.sh
├── Dockerfile
├── entrypoint.sh
├── Readme.txt
├── run.sh
└── scripts/
    └── sitl.sh
```

---

## Installation 

### Clone and build

```bash
git clone https://github.com/rrajnidhi/px4_1.15_ros2_humble_ubuntu22.04.git
cd px4_1.15_ros2_humble_ubuntu22.04
./build.sh
```

---

## Launch container

To start and enter the docker container 

```bash
./run.sh
```

---

## A. Single drone SITL

Once inside to launch PX4 SITL + ROS2 Bridge + Micro XRCE-DDS:

```bash
cd /scripts
./sitl.sh
```
This script launches a **tmux** session with:

| Pane | Description                                  |
| ---- | -------------------------------------------- |
| 0    | PX4 SITL + Gazebo (headless)                 |
| 1    | Micro XRCE-DDS Agent (auto-starts after PX4) |
| 2    | Reserved for user commands / ROS2 nodes      |

### Environment Variables

The following environment variables can be set before running `sitl.sh` to customize the simulation.  
If not provided, the script uses the default values shown below.

| Variable | Default | Value Range / Example | Description |
|-----------|----------|------------------------|--------------|
| **ROS_DOMAIN_ID** | `3` | Integer (0–255) | Unique ROS 2 domain ID to isolate DDS communication between multiple simulations. |
| **PX4_UXRCE_DDS_PORT** | `8888` | Integer (e.g., 8888–9999) | UDP port used by the Micro XRCE-DDS Agent to communicate with PX4 SITL. |
| **PX4_UXRCE_DDS_NS** | `drone` | String | Namespace prefix used for PX4 ROS 2 topics and nodes. |
| **PX4_UAV_MODEL** | `gz_x500` | PX4 supported Gazebo model (e.g., `gz_x500`, `gz_x500_depth`, `gz_x500_vision`, `gz_standard_vtol`, `gz_rc_cessna`, `gz_quadtailsitter`, `gz_tiltrotor`, `gz_rover_differential`, `gz_rover_ackermann`, `gz_rover_mecanum`) | Specifies which UAV model to launch in Gazebo Sim. |
| **PX4_UAV_COUNT** | `1` | Integer (1–N) | Number of UAV instances to simulate (for multi-vehicle setups). |

#### Example usage

```bash
ROS_DOMAIN_ID=5 PX4_UXRCE_DDS_PORT=9000 PX4_UXRCE_DDS_NS=uav PX4_UAV_MODEL=gz_standard_vtol ./sitl.sh
```

---

## B. Multi drone SITL : Method-1

    Multi-drone is implemented as ros2 package. Launch it with following command

    ```bash
    ros2 launch multi_drone test_multi_drone_run.launch.py 
    ```

---

## C. Multi drone SITL : Method-2

    This is still in development.

---


## Help with ros2 and tmux  

### Tmux Controls

* Detach: `Ctrl+b d`
* Move-between-panes: `Ctrl+b <arrow-key>`
* Reattach:

```bash
tmux attach -t sitl_px4_ros2_humble
```

* Kill session:

```bash
Ctrl+b e
```

---

### List PX4 topics

```bash
ros2 topic list
```

### Echo vehicle odometry

```bash
ros2 topic echo /fmu/out/vehicle_odometry
```

## Cleanup

To stop and remove containers:

```bash
docker ps
docker stop <container_id>
```

The container is automatically removed on exit because `--rm` is used in `run.sh`.

---

## Maintainer

**Nidhi Raj**
[nidhirajr@gmail.com](mailto:nidhirajr@gmail.com)

---

## Acknowledgements

* [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
* [ROS 2 Humble](https://docs.ros.org/en/humble/)
* [Gazebo Sim](https://gazebosim.org/)
* [Micro XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS-Agent)

---


> Enjoy! 