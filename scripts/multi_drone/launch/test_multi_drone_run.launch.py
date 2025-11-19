#!/usr/bin/env python3
import os
import logging
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from multi_drone.utils.methods import load_yaml_params
from multi_drone.scripts.runner import launch_robot, get_microxrce_agent_exec
from multi_drone.scripts.gazebo_server import run_simulation_gazebo  


# Gazebo world name
WORLD = 'walls'


def generate_launch_description():
    pkg_multi_drone = get_package_share_directory('multi_drone')
    params_file = os.path.join(pkg_multi_drone, 'config', 'test_params.yaml')
    robots_config = load_yaml_params(params_file)
    launch_descriptions = []

    # ✅ 1. Start Gazebo simulation first
    gazebo_server = run_simulation_gazebo(
        world=WORLD,
        custom_model_store_other=[],
        headless=False,  # set True if no GUI
        gz_ip=None,
        gz_partition=None
    )
    launch_descriptions.append(gazebo_server)

    # ✅ 2. Start Micro XRCE-DDS agent
    microxrce_agent = get_microxrce_agent_exec(udp='udp4', port='8888')
    launch_descriptions.append(microxrce_agent)

    # ✅ 3. Launch each drone defined in YAML
    for robot in robots_config['robots']:
        logging.info(f"Launching drone {robot['drone_id']} of type {robot['drone_type']}")
        launch_descriptions.extend(
            launch_robot(
                drone_id=robot['drone_id'],
                drone_type=robot['drone_type'],
                gz_world=WORLD,
                spawn_position=robot['position'],
                px4_autostart=robot['px4_autostart'],
                px4_dir="/app/PX4-Autopilot/",
                terminal='bash',
                package_name="multi_drone",
                controller_script=robot['controller_script'],
                additional_params={}
            )
        )

    return LaunchDescription(launch_descriptions)


if __name__ == "__main__":
    generate_launch_description()

