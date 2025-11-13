import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multi_drone.move_commands.x500.g_code import G20_MoveToPoint


class TestMoveToPointNode(Node):
    """
    Node for sending a test G20_MoveToPoint command in the global ENU coordinate system.
    """
    def __init__(self):
        super().__init__('test_move_to_point_node')

        # Create a test movement command: go to (0, 0, 10) in global ENU
        self.target_point = G20_MoveToPoint(
            x=0.0,
            y=0.0,
            z=10.0,
            coordinate_system='global_ENU'
        )
       
        # Publisher for JSON-encoded G-code commands
        self.command_publisher = self.create_publisher(
            String,
            '/px4_1/command_json',
            10
        )

        try:
            command_msg = String()
            command_msg.data = json.dumps(self.target_point.to_dict())
            self.command_publisher.publish(command_msg)
            self.get_logger().info(f"Command sent: {command_msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TestMoveToPointNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
