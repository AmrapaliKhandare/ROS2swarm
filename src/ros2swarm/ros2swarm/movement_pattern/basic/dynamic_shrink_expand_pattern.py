
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class DiamondPatternNode(Node):
    def __init__(self):
        super().__init__('diamond_shrink_expand_pattern')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('loop_rate', 0.2)
        self.robot_id = self.get_parameter('robot_id').value
        self.loop_rate = self.get_parameter('loop_rate').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(self.loop_rate, self.timer_callback)

        self.state = 1  # 1: form diamond, 2: wide, 3: narrow, 4: expand
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.obstacle_detected = False

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = ranges[np.isfinite(ranges)]
        if len(ranges) == 0:
            return
        min_dist = np.min(ranges)
        self.obstacle_detected = (min_dist < 0.6)

    def get_movement_for_state(self):
        # Format: [linear_x, angular_z]
        formation = {
            1: {  # Move to diamond
                1: [0.2, 0.0],
                2: [0.2, 0.5],
                3: [0.2, -0.5],
                4: [0.2, 0.0]
            },
            2: {  # Maintain wide diamond
                1: [0.15, 0.0],
                2: [0.15, 0.4],
                3: [0.15, -0.4],
                4: [0.14, 0.0]
            },
            3: {  # Narrow formation to pass
                1: [0.15, 0.0],
                2: [0.13, 0.2],
                3: [0.13, -0.2],
                4: [0.12, 0.0]
            },
            4: {  # Expand back to wide
                1: [0.15, 0.0],
                2: [0.15, 0.5],
                3: [0.15, -0.5],
                4: [0.14, 0.0]
            }
        }
        return formation[self.state].get(self.robot_id, [0.0, 0.0])

    def timer_callback(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = now - self.start_time

        # Simple transitions based on time and obstacle
        if self.state == 1 and elapsed > 10:
            self.state = 2
            self.start_time = now
            self.get_logger().info("Holding wide diamond formation.")
        elif self.state == 2 and self.obstacle_detected:
            self.state = 3
            self.start_time = now
            self.get_logger().info("Narrowing to pass obstacle.")
        elif self.state == 3 and elapsed > 10:
            self.state = 4
            self.start_time = now
            self.get_logger().info("Expanding back to wide formation.")
        elif self.state == 4 and elapsed > 10:
            self.state = 2  # Loop between wide–narrow–wide
            self.start_time = now
            self.get_logger().info("Back to wide formation.")

        # Publish motion command
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = self.get_movement_for_state()
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DiamondPatternNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
