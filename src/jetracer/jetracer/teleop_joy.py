#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_joy')

        self.declare_parameter('x_speed', 0.3)
        self.declare_parameter('y_speed', 0.0)  # not used
        self.declare_parameter('w_speed', 1.0)
        self.declare_parameter('hz', 20)

        self.x_speed = self.get_parameter('x_speed').value
        self.w_speed = self.get_parameter('w_speed').value
        hz = self.get_parameter('hz').value

        self.active = 0
        self.cmd = Twist()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.timer = self.create_timer(1.0 / hz, self.publish_cmd)

    def joy_callback(self, data):
        if data.buttons[6] == 1:
            self.cmd.linear.x = self.x_speed * data.axes[3]
            self.cmd.angular.z = self.w_speed * data.axes[0]
            self.active = 1
        else:
            self.cmd = Twist()  # reset
            self.active = 0

    def publish_cmd(self):
        if self.active:
            self.cmd_pub.publish(self.cmd)
        else:
            self.cmd_pub.publish(self.cmd)  # send zero if inactive

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
