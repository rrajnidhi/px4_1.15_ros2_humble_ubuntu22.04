#!/usr/bin/env python3
from typing import List, Literal, Optional
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


class PX4Process:
    """
    Class for creating and managing PX4 processes with flexible configuration.

    Parameters
    ----------
    - drone_id : int
        Unique identifier for the drone instance. Default: 1.
    - drone_type : str
        Drone type (e.g., 'x500'). Default: 'x500'.
    - gz_world : str
        Name of the Gazebo world file. Default: 'default'.
    - spawn_position : List[float]
        Initial pose in format [x, y, z, roll, pitch, yaw]. Default: [0, 0, 0, 0, 0, 0].
    - px4_autostart : int
        PX4 SYS_AUTOSTART ID for configuration. Default: 4001.
    - px4_dir : str
        Path to PX4 installation directory. Default: "/app/PX4-Autopilot/".
    - px4_simulator : str
        Simulator type (e.g., 'GZ' for Gazebo). Default: 'GZ'.
    - px4_model_name : Optional[str]
        Name of an existing Gazebo model to attach PX4 to. Mutually exclusive with px4_sim_model.
    - px4_sim_model : Optional[str]
        Name of a new Gazebo model to spawn. Mutually exclusive with px4_model_name.
    - px4_render_engine : Optional[str]
        Gazebo rendering engine. Default: 'ogre'.
    - terminal : Literal['gnome-terminal', 'xterm', 'konsole', 'bash']
        Terminal emulator to use. Default: 'gnome-terminal'.

    Raises
    ------
    ValueError
        If both px4_model_name and px4_sim_model are specified (mutually exclusive).
    ValueError
        If an unsupported terminal emulator is selected.
    """
    def __init__(
        self,
        drone_id: int = 1,
        gz_world: str = "default",
        px4_autostart: int = 4001,
        px4_dir: str = "/app/PX4-Autopilot/",
        px4_simulator: str = "GZ",
        px4_model_name: Optional[str] = None,
        px4_sim_model: Optional[str] = "x500",
        px4_gz_model_pose: List[float] = [0, 0, 0, 0, 0, 0],
        px4_render_engine: Optional[str] = "ogre",
        standalone: bool = True,
        terminal: Literal["gnome-terminal", "xterm", "konsole", "bash"] = "gnome-terminal",
    ):
        if px4_model_name and px4_sim_model:
            raise ValueError("px4_model_name and px4_sim_model are mutually exclusive.")
       
        self.drone_id = drone_id
        self.gz_world = gz_world
        self.px4_gz_model_pose = ','.join(map(str, px4_gz_model_pose))
        self.px4_autostart = px4_autostart
        self.px4_dir = px4_dir
        self.px4_simulator = px4_simulator
        self.px4_model_name = px4_model_name
        self.px4_sim_model = px4_sim_model
        self.px4_render_engine = px4_render_engine
        self.standalone = standalone
        self.terminal = terminal

    def create_process(self) -> ExecuteProcess:
        """
        Creates a PX4 process with the configured environment variables.

        Returns
        -------
        ExecuteProcess
            Launch action to start the PX4 instance.
        """
        env_vars = [
            f"PX4_SYS_AUTOSTART={self.px4_autostart}",
            f"PX4_SIMULATOR={self.px4_simulator}",
            f"PX4_GZ_WORLD={self.gz_world}",
        ]
       
        if self.standalone:
            env_vars.append("PX4_GZ_STANDALONE=1")
        if self.px4_model_name:
            env_vars.append(f"PX4_GZ_MODEL_NAME={self.px4_model_name}")
        elif self.px4_sim_model:
            env_vars.extend([
                f"PX4_SIM_MODEL={self.px4_sim_model}",
                f"PX4_GZ_MODEL_POSE={self.px4_gz_model_pose}",
            ])
        if self.px4_render_engine:
            env_vars.append(f"PX4_GZ_SIM_RENDER_ENGINE={self.px4_render_engine}")

        cmd = [
            *env_vars,
            f"{self.px4_dir}/build/px4_sitl_default/bin/px4",
            f"-i {self.drone_id}",
        ]

        if self.terminal == "gnome-terminal":
            launch_cmd = ["gnome-terminal", "--", "bash", "-c", ' '.join(cmd)]
        elif self.terminal == "xterm":
            launch_cmd = ["xterm", "-hold", "-e", ' '.join(cmd)]
        elif self.terminal == "konsole":
            launch_cmd = ["konsole", "--hold", "-e", ' '.join(cmd)]
        elif self.terminal == "bash":
            launch_cmd = ["bash", "-c", ' '.join(cmd)]
        else:
            raise ValueError(f"Unsupported terminal: {self.terminal}")

        return ExecuteProcess(cmd=launch_cmd, output="screen")


class ControllerNode:
    """
    Universal class for creating and managing a drone controller node.
    """
    def __init__(
        self,
        package_name: str = 'multi_drone',
        controller_script: str = 'x500.py',
        drone_id: int = 1,
        drone_type: str = 'x500',
        spawn_position: list = [0, 0, 0, 0, 0, 0],
        output: str = 'log',
        additional_params: dict = {},
    ):
        """
        Initialize the controller node.

        :param drone_id: Drone identifier.
        :param drone_type: Type of drone.
        :param spawn_position: Initial pose [x, y, z, roll, pitch, yaw].
        :param output: Output type ('log' or 'screen').
        :param package_name: ROS package name.
        :param additional_params: Extra parameters to pass to the node.
        """
        self.package_name = package_name
        self.executables_script = controller_script
        self.drone_id = drone_id
        self.drone_type = drone_type
        self.default_position = [float(x) for x in spawn_position[:3]]
        self.default_orientation = [float(x) for x in spawn_position[3:]]
        self.output = output
        self.additional_params = additional_params

    def create_node(self) -> Node:
        """
        Creates and returns the controller node.
        """
        parameters = [
            {'drone_id': self.drone_id},
            {'drone_type': self.drone_type},
            {'default_position': self.default_position},
            {'default_orientation': self.default_orientation},
            *[{key: value} for key, value in self.additional_params.items()]
        ]
        return Node(
            package=self.package_name,
            executable=self.executables_script,
            name=f'id_{self.drone_id}_{self.drone_type}_controller_node',
            output=self.output,
            parameters=parameters,
        )


def get_microxrce_agent_exec(
        udp: str = 'udp4',
        port: str = '8888'
    ) -> ExecuteProcess:
    """Launch MicroXRCE-DDS Agent."""
    return ExecuteProcess(
        cmd=['MicroXRCEAgent', f'{udp}', '-p', f'{port}'],
        output='log',
        name="microxrce_agent"
    )


def launch_robot(
        drone_id: int = 1,
        drone_type: str = 'x500',
        gz_world: str = 'default',
        spawn_position: List[float] = [0, 0, 0, 0, 0, 0],
        px4_autostart: int = 4001,
        px4_dir: str = "/app/PX4-Autopilot/",
        px4_simulator: str = "GZ",
        px4_model_name: Optional[str] = None,
        px4_render_engine: Optional[str] = "ogre",
        standalone: bool = True,
        terminal: Literal['gnome-terminal', 'xterm', 'konsole', 'bash'] = 'gnome-terminal',
        package_name: str = 'multi_drone',
        controller_script: str = 'x500.py',
        additional_params: dict = {}
    ):
    """
    Generate a list of launch actions (PX4 + controller node) for a single drone.

    Returns
    -------
    List of launch actions (ExecuteProcess + Node).
    """
    px4_process = PX4Process(
        drone_id=drone_id,
        gz_world=gz_world,
        px4_autostart=px4_autostart,
        px4_dir=px4_dir,
        px4_simulator=px4_simulator,
        px4_model_name=px4_model_name,
        px4_sim_model=drone_type,
        px4_gz_model_pose=spawn_position,
        px4_render_engine=px4_render_engine,
        standalone=standalone,
        terminal=terminal
    )
    controller_node = ControllerNode(
        package_name=package_name,
        controller_script=controller_script,
        additional_params=additional_params,
        drone_id=drone_id,
        drone_type=drone_type,
        spawn_position=spawn_position,
        output='log'
    )
    return [
        px4_process.create_process(),
        controller_node.create_node()
    ]


if __name__ == '__main__':
    launch_robot()
