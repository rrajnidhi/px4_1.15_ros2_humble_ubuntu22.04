#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy
)
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import (
    VehicleAttitude,
    VehicleCommand,
    VehicleLocalPosition
)
from multi_drone.controllers.base.base_data import PositionData, OrientationData
from multi_drone.controllers.base.position_transformer import DroneLocalityState


class BaseDroneController(Node):
    def __init__(
        self,
        drone_type: str = 'x500',
        drone_id: int = 1,
        default_position: list = [0.0, 0.0, 0.0],  # Position given as list [x, y, z] in meters
        default_orientation: list = [0.0, 0.0, 0.0],  # Orientation given as list [roll, pitch, yaw] in radians
    ):
        super().__init__(f'drone_controller')

        self.drone_type = drone_type
        self.drone_id = drone_id

        self.declare_parameter('drone_id', value=drone_id, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('drone_type', value=drone_type, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('default_position', value=default_position, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('default_orientation', value=default_orientation, descriptor=ParameterDescriptor(dynamic_typing=True))

        self.drone_id: int = self.get_parameter('drone_id').value
        self.drone_type: str = self.get_parameter('drone_type').value
        default_position: list = self.get_parameter('default_position').value
        default_orientation: list = self.get_parameter('default_orientation').value

        self.default_world_position_ENU = PositionData(
            x=default_position[0],
            y=default_position[1],
            z=default_position[2]
        )
        self.default_world_orientation_ENU = OrientationData(
            roll=default_orientation[0],
            pitch=default_orientation[1],
            yaw=default_orientation[2]
        )

        self.current_position = DroneLocalityState(
            self.default_world_position_ENU,
            self.default_world_orientation_ENU
        )
        self.target_position = DroneLocalityState(
            self.default_world_position_ENU,
            self.default_world_orientation_ENU
        )

        # Topic prefixes
        self.prefix_px = f"px4_{self.drone_id}"
        self.prefix_name = f"id_{self.drone_id}_{self.drone_type}"

        self.qos_profile_unreliable = self.get_qos_profile(
            reliable=False, depth=5
        )
        self.qos_profile_reliable = self.get_qos_profile(
            reliable=True, depth=10
        )

        # Subscriptions
        self.subscriber_vehicle_attitude = self.create_subscription(
            VehicleAttitude,
            f'{self.prefix_px}/fmu/out/vehicle_attitude',
            self.callback_vehicle_attitude,
            self.qos_profile_unreliable
        )

        self.subscriber_local_position = self.create_subscription(
            VehicleLocalPosition,
            f'{self.prefix_px}/fmu/out/vehicle_local_position',
            self.callback_local_position,
            self.qos_profile_unreliable
        )

        # Publisher
        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand,
            f'{self.prefix_px}/fmu/in/vehicle_command',
            self.qos_profile_reliable
        )

    def get_qos_profile(self, reliable=True, depth=10):
        """
        Returns a QoS profile. Reliable by default.
        """
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE
                if reliable
                else QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
        )

    def callback_vehicle_attitude(self, msg: VehicleAttitude):
        """
        Callback for receiving the drone's orientation in ENU and converting it to NED.
        """
        self.current_position.update_orientation(msg.q)

    def callback_local_position(self, msg: VehicleLocalPosition):
        """
        Receives local drone coordinates and processes velocities
        in both NED and ENU coordinate systems.
        """
        self.current_position.update_position(
            np.array([msg.x, msg.y, msg.z]),
            system='local_NED')
        self.current_position.update_velocity(
            np.array([msg.vx, msg.vy, msg.vz]),
            system='local_NED')

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=5.0,
        target_system=None,
        target_component=1,
        source_system=None,
        source_component=1,
        from_external=True
    ):
        """
        Publishes a VehicleCommand to control the drone.

        Parameters:
        ----------
        command : int
            Command identifier (e.g., arm, takeoff, land, etc.).
        param1 : float, optional
            Primary command parameter. Depends on command type (default 0.0).
            Examples:
            - For arming: 1.0 (ARM), 0.0 (DISARM).
            - For takeoff: time in seconds.
        param2 : float, optional
            Secondary command parameter (default 0.0).
            Example:
            - For SET_MODE: flight mode (1: AUTO, 6: OFFBOARD, etc.).
        param3 : float, optional
            Tertiary command parameter (default 0.0).
        param4 : float, optional
            Fourth command parameter (default 0.0).
        param5 : float, optional
            Fifth command parameter (default 0.0). Often used for X coordinate.
        param6 : float, optional
            Sixth command parameter (default 0.0). Often used for Y coordinate.
        param7 : float, optional
            Seventh command parameter (default 5.0). Often used for altitude (Z coordinate).
        target_system : int, optional
            System that should execute the command. If None, uses `self.drone_id + 1`.
        target_component : int, optional
            Component that should execute the command (default 1).
        source_system : int, optional
            System sending the command. If None, uses `self.drone_id`.
        source_component : int, optional
            Component sending the command (default 1).
        from_external : bool, optional
            Flag indicating the command was sent externally (default True).
        """
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = target_system if target_system is not None else self.drone_id + 1
        msg.target_component = target_component
        msg.source_system = source_system if source_system is not None else self.drone_id
        msg.source_component = source_component
        msg.from_external = from_external
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(msg)

    def log_error(self, message):
        self.get_logger().error(message)

    def log_info(self, message):
        self.get_logger().info(message)


def main():
    rclpy.init()
    control_node = BaseDroneController()

    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info("Shutting down controller.")
    finally:
        control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
