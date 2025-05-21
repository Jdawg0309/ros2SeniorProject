#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class PID:
    def __init__(self, Kp, Ki, Kd, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limits = output_limits

        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def compute(self, error, current_time):
        dt = current_time - self.previous_time if self.previous_time else 0.0
        self.previous_time = current_time

        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        self.integral += error * dt
        self.previous_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)

        return output

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.target_speed = 3.0  # m/s
        self.v_max = 10.0

        self.pid = PID(Kp=1.0, Ki=0.05, Kd=0.2, output_limits=(0.0, self.v_max))

        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel_pid',
            10
        )

        self.get_logger().info('PID Controller Node has been started.')

    def odom_callback(self, msg):
        current_speed = msg.twist.twist.linear.x
        error = self.target_speed - current_speed
        v_cmd = self.pid.compute(error, time.time())

        msg_out = Twist()
        msg_out.linear.x = v_cmd
        self.publisher.publish(msg_out)

        self.get_logger().info(
            f'Target: {self.target_speed:.2f}, Measured: {current_speed:.2f}, '
            f'Error: {error:.2f}, Cmd: {v_cmd:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
