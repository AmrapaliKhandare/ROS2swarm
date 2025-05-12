
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class DrivePatternNode(Node):
    def __init__(self):
        super().__init__('drive_pattern')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('loop_rate', 0.2)
        self.robot_id = self.get_parameter('robot_id').value
        self.loop_rate = self.get_parameter('loop_rate').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(self.loop_rate, self.timer_callback)

        self.state = 1  # 1: forming diamond, 2: moving straight
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def get_command(self):
        """
        Define robot behavior per state.
        """
        formation = {
            1: {  # Form diamond
                1: [0.2, 0.3],   # top left
                2: [0.2, 0.6],   # top right
                3: [0.2, -0.6],  # bottom left
                4: [0.2, -0.3],  # bottom right
            },
            2: {  # Move straight
                1: [0.2, 0.0],
                2: [0.2, 0.0],
                3: [0.2, 0.0],
                4: [0.2, 0.0],
            }
        }
        return formation[self.state].get(self.robot_id, [0.0, 0.0])

    def timer_callback(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = now - self.start_time

        if self.state == 1 and elapsed > 10:
            self.state = 2
            self.get_logger().info("Diamond formed. Now moving forward.")
            self.start_time = now

        linear, angular = self.get_command()
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DrivePatternNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
