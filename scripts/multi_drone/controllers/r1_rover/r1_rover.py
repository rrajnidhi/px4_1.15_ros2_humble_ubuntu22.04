import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import numpy as np


class RoverControl(Node):
    def __init__(self):
        super().__init__('rover_control')

        # Subscribe to velocity commands (Twist) that will control the robot
        self.velocity_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        # Subscribe to laser scanner data for obstacle avoidance
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Publisher to send motor commands to the robot
        self.motor_pub = self.create_publisher(Twist, '/r1_rover/cmd_vel', 10)

        # Variables to store current linear and angular velocity
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Flag to track obstacle detection state
        self.obstacle_detected = False

    def velocity_callback(self, msg):
        """Receive linear and angular velocity commands"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.get_logger().info(f"Received command: linear velocity {self.linear_velocity}, angular velocity {self.angular_velocity}")

    def laser_callback(self, msg):
        """Process laser scanner data for obstacle detection"""
        # Simple obstacle detection example (if distance is less than 0.5 meters)
        min_distance = np.min(msg.ranges)
        if min_distance < 0.5:
            self.obstacle_detected = True
            self.get_logger().info(f"Obstacle detected at distance {min_distance} meters")
        else:
            self.obstacle_detected = False

    def control_loop(self):
        """Control loop to send movement commands to the robot"""
        # If an obstacle is detected, stop the robot
        if self.obstacle_detected:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.get_logger().info("Stopping due to obstacle")

        # Publish the movement command
        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = self.angular_velocity
        self.motor_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RoverControl()

    # Run the control loop at 10 Hz
    timer_period = 0.1  # 100 ms
    node.create_timer(timer_period, node.control_loop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
