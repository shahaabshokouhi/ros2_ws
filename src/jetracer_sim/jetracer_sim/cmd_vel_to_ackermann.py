#!/usr/bin/env python3
"""cmd_vel -> Ackermann drive for the simulated JetRacer.

Stands in for the real base driver: /<agent>/cmd_vel (geometry_msgs/Twist,
from Nav2 or the keyboard teleop) becomes /<agent>/drive
(ackermann_msgs/AckermannDriveStamped) for Isaac Sim, at 20 Hz.

The turn rate is turned into a front-wheel angle with the bicycle model,
steering = atan(wheelbase * w / v), clamped to the steering limit, so a
command the car cannot follow saturates the steering as on the real car. The
car stops 1 s after the last command, like the real driver.

Like the real driver it also publishes the "wheel odometry" /<agent>/odom
(Nav2 reads the robot's speed there): here, the simulator's true odometry
from /<agent>/ground_truth/odom, in frames odom -> base_footprint.
"""
import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        agent = self.declare_parameter('agent_name', 'sim').value
        self.L = float(self.declare_parameter('wheelbase', 0.15).value)
        self.max_steer = float(self.declare_parameter('max_steering', 0.59).value)
        self.timeout = float(self.declare_parameter('timeout', 1.0).value)
        self.pub = self.create_publisher(AckermannDriveStamped, f'/{agent}/drive', 10)
        self.create_subscription(Twist, f'/{agent}/cmd_vel', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, f'/{agent}/odom', 20)
        self.create_subscription(Odometry, f'/{agent}/ground_truth/odom', self.on_odom, 20)
        self.v = self.w = 0.0
        self.steer = 0.0
        self.last = None
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'/{agent}/cmd_vel -> /{agent}/drive (wheelbase {self.L} m, '
                               f'steering limit {self.max_steer} rad)')

    def on_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z
        self.last = self.get_clock().now()

    def on_odom(self, m):
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_footprint'
        self.odom_pub.publish(m)

    def tick(self):
        v, w = self.v, self.w
        if self.last is None or (self.get_clock().now() - self.last).nanoseconds * 1e-9 > self.timeout:
            v = w = 0.0
        if abs(v) > 1e-3:
            self.steer = max(-self.max_steer, min(self.max_steer, math.atan(self.L * w / v)))
        # at standstill the wheels keep their last angle
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.drive.speed = float(v)
        m.drive.steering_angle = float(self.steer)
        self.pub.publish(m)


def main():
    rclpy.init()
    try:
        rclpy.spin(CmdVelToAckermann())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
