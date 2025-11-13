#!/usr/bin/env python3
from typing import List
import subprocess
import os
import shutil
from launch.actions import ExecuteProcess

DEFAULT_DOWNLOAD_DIR = "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip"


def run(cmd: str):
    """Helper to run shell commands."""
    process_handle = subprocess.Popen(['bash', '-c', cmd], cwd='.')
    return process_handle


def run_simulation_gazebo(
    world: str = "default",
    gz_partition: str = None,
    gz_ip: str = None,
    model_download_source: str = DEFAULT_DOWNLOAD_DIR,
    px4_model_store: str = "~/simulation-gazebo",
    custom_model_store: str = None,
    custom_model_store_other: List[str] = [],
    overwrite: bool = False,
    headless: bool = False,
    return_cmd: bool = False,
) -> ExecuteProcess:
    """
    Function to set up and launch a Gazebo simulation with support for custom models and worlds.

    Args:
        world (str): Name of the world file to simulate (without extension).
        gz_partition (str): Optional Gazebo resource partition.
        gz_ip (str): Optional IP address for network interface.
        model_download_source (str): URL or path to download PX4 models from.
        px4_model_store (str): Local directory to store PX4 models.
        custom_model_store (str): Directory for custom models/worlds.
        custom_model_store_other (List[str]): Additional model/world directories.
        overwrite (bool): Overwrite existing models if True.
        headless (bool): Run Gazebo without GUI if True.
        return_cmd (bool): Return the command string instead of ExecuteProcess (for debugging).

    Returns:
        ExecuteProcess: Launch action to start the Gazebo server.

    Example:
        >>> gazebo_server = run_simulation_gazebo(
        ...     world="test_world",
        ...     gz_partition="default",
        ...     gz_ip="127.0.0.1",
        ...     model_download_source="https://example.com/models.zip",
        ...     px4_model_store="/home/user/gazebo_models",
        ...     custom_model_store="/home/user/custom_models",
        ...     custom_model_store_other=["/opt/models", "/usr/share/models"],
        ...     overwrite=True,
        ...     headless=True
        ... )
    """
    # Expand user path (~) for model storage
    px4_model_store = os.path.expanduser(px4_model_store)

    # Ensure model storage directory exists
    if not os.path.exists(px4_model_store):
        print("Creating model storage directory...")
        os.makedirs(px4_model_store)

    # Check if models already exist
    model_count = int(subprocess.check_output(
        f'find {px4_model_store} -type f | wc -l', shell=True, text=True
    ))
    models_exist = model_count > 0
    print(f"Found {model_count} files in {px4_model_store}")

    if models_exist and not overwrite:
        print("Model directory is not empty and overwrite is disabled. Skipping download.")
    elif overwrite and models_exist:
        # Remove existing subdirectories in overwrite mode
        try:
            subdirs = [
                os.path.join(px4_model_store, d)
                for d in os.listdir(px4_model_store)
                if os.path.isdir(os.path.join(px4_model_store, d))
            ]
            for directory in subdirs:
                shutil.rmtree(directory)
            print("Overwrite enabled. Existing subdirectories removed.")
        except Exception as e:
            print(f"Error during cleanup: {e}")

    # Download and extract models if needed
    if not models_exist or overwrite:
        print("Downloading models from default source...")
        os.system(f'curl -L -o {px4_model_store}/resources.zip {model_download_source}')

        # Extract archive
        try:
            shutil.unpack_archive(f'{px4_model_store}/resources.zip', px4_model_store, 'zip')
        except Exception as e:
            print(f"Warning: Failed to unpack models from {px4_model_store}/resources.zip. Error: {e}")

        # Organize extracted files
        os.system(f'mv {px4_model_store}/PX4-gazebo-models-main/models {px4_model_store}/models')
        os.system(f'mv {px4_model_store}/PX4-gazebo-models-main/worlds {px4_model_store}/worlds')
        os.system(f'rm {px4_model_store}/resources.zip')
        os.system(f'rm -rf {px4_model_store}/PX4-gazebo-models-main/')

    print(f"> Launching Gazebo simulation with world: {world}")

    # Build GZ_SIM_RESOURCE_PATH
    custom_model_paths = []
    world_path = f"{world}.sdf"

    if custom_model_store:
        custom_model_paths = [
            f"{custom_model_store}/models",
            f"{custom_model_store}/worlds"
        ]

    custom_model_paths.extend(
        [f"{path}/models:{path}/worlds" for path in custom_model_store_other]
    )

    gz_sim_resource_path = ':'.join(
        [f"{px4_model_store}/models", f"{px4_model_store}/worlds"] + custom_model_paths
    )
    print(f"GZ_SIM_RESOURCE_PATH set to: {gz_sim_resource_path}")

    # Base Gazebo command
    gz_cmd = ['gz', 'sim', '-r', world_path]
    if headless:
        gz_cmd.append('-s')  # Server-only mode

    # Environment variables
    gz_env = {
        'GZ_SIM_RESOURCE_PATH': gz_sim_resource_path
    }
    if gz_partition:
        gz_env['GZ_PARTITION'] = gz_partition
    if gz_ip:
        gz_env['GZ_IP'] = gz_ip

    if return_cmd:
        # Return full command string for debugging
        cmd_str = f"GZ_SIM_RESOURCE_PATH={gz_sim_resource_path} "
        cmd_str += f"GZ_PARTITION={gz_partition} " if gz_partition else ''
        cmd_str += f"GZ_IP={gz_ip} " if gz_ip else ''
        cmd_str += ' '.join(gz_cmd)
        return cmd_str

    # Launch Gazebo via ExecuteProcess
    gazebo_server = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        additional_env=gz_env
    )
    return gazebo_server


if __name__ == "__main__":
    cmd = run_simulation_gazebo(return_cmd=True)
    run(cmd)
