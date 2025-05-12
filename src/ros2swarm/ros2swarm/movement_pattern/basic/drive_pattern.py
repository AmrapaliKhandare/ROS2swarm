
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
        self.robot_id = self.get_parameter('robot_id').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.reached = False
        self.state = 1  # 1 = move to position, 2 = move forward
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Define diamond target positions
        self.targets = {
            1: (1.5, 1.5),   # top
            2: (2.5, 0.0),   # right
            3: (0.5, 0.0),   # left
            4: (1.5, -1.5)   # bottom
        }

        self.goal_x, self.goal_y = self.targets[self.robot_id]

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        cmd = Twist()

        if self.state == 1:
            dx = self.goal_x - self.x
            dy = self.goal_y - self.y
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.yaw)

            if distance > 0.1:
                cmd.linear.x = min(0.2, distance)
                cmd.angular.z = 1.5 * angle_diff
            else:
                self.get_logger().info(f"Robot {self.robot_id} reached diamond point.")
                self.state = 2  # Switch to move-forward phase

        elif self.state == 2:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = DrivePatternNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
