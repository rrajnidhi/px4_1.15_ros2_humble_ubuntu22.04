# UAV Simulation with PX4, ROS2 & Micro XRCE-DDS

Environment for running UAV simulations with **PX4**, **ROS2**, **Gazebo**, and **Micro XRCE-DDS**, including the **PX4-ROS2 bridge**.  
---

## Features

- Fully containerized PX4 SITL environment for Ubuntu 22.04


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


```bash
./run.sh
```

---

## Inside the Container

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

## Configuration Options (in `sitl.sh`)

| Variable             | Default | Description              |
| -------------------- | ------- | ------------------------ |
| `ROS_DOMAIN_ID`      | 3       | ROS2 domain ID           |
| `PX4_UXRCE_DDS_PORT` | 8888    | Micro XRCE-DDS UDP port  |
| `PX4_UXRCE_DDS_NS`   | drone   | Namespace for PX4 topics |
| `PX4_UAV_MODEL`      | gz_x500 | PX4 vehicle model        |

You can change these values to customize your simulation setup.

---

## Example ROS2 Commands

Once SITL and Micro XRCE-DDS Agent are running, open a new terminal or tmux pane:

### List PX4 topics

```bash
ros2 topic list
```

### Echo vehicle odometry

```bash
ros2 topic echo /fmu/out/vehicle_odometry
```

### Publish offboard velocity command

```bash
ros2 topic pub /fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode "{velocity: {x: 1.0, y: 0.0, z: 0.0}}"
```

---

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

> Contributions welcome! Feel free to submit issues or PRs to improve this environment.