#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import csv
import os
import math
import time


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('steering_controller')
        self.i = 0
        self.lookahead_distance = 3.0
        self.v_max = 10.0
        self.current_index = 0
        self.position = None
        self.yaw = 0.0

        self.current_velocity_cmd = 0.0  # <-- Value from PID node

        # Load (x, y) waypoints
        csv_file_path = os.path.expanduser('~/ros2_ws/src/pure_pursuit/trajectory2.csv')
        self.waypoints = self.load_waypoints(csv_file_path)

        # Subscriptions
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.velocity_sub = self.create_subscription(Twist, 'cmd_vel_pid', self.velocity_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control Loop
        self.timer = self.create_timer(0.1, self.publish_cmd)

        self.get_logger().info('Pure Pursuit Node started and waiting for PID velocity input.')

    def load_waypoints(self, path):
        waypoints = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    waypoints.append((float(row[0]), float(row[1])))
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints')
        return waypoints

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.position = (x, y)

        q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def velocity_callback(self, msg):
        self.current_velocity_cmd = msg.linear.x

    def quaternion_to_yaw(self, x, y, z, w):
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def find_goal_point(self):
        if self.position is None:
            return None

        x, y = self.position
        for i in range(self.current_index, len(self.waypoints)):
            wp_x, wp_y = self.waypoints[i]
            dist = math.hypot(wp_x - x, wp_y - y)
            if dist >= self.lookahead_distance:
                self.current_index = i
                return wp_x, wp_y
        return None

    def publish_cmd(self):
        if self.position is None:
            return

        goal = self.find_goal_point()
        if goal is None:
            self.get_logger().info('No goal point found — stopping.')
            self.publisher.publish(Twist())  # Stop
            return

        x, y = self.position
        goal_x, goal_y = goal

        dx = goal_x - x
        dy = goal_y - y
        alpha = math.atan2(dy, dx) - self.yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # normalize angle

        v = self.current_velocity_cmd
        L = self.lookahead_distance
        omega = 2 * v * math.sin(alpha) / L
        omega = max(min(omega, 1.5), -1.5)

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self.publisher.publish(msg)

        if self.i % 10 == 0:
            self.get_logger().info(f'Pos: ({x:.2f}, {y:.2f}), Target: ({goal_x:.2f}, {goal_y:.2f}), α: {math.degrees(alpha):.2f}°, ω: {omega:.2f}, v: {v:.2f}')
            self.i = 0
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
