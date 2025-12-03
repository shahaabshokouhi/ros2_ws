#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class OdomEKF(Node):
    def __init__(self):
        super().__init__('odom_ekf')
        self.declare_parameter('agent_name','agent_0')
        self.ekf_pub = self.create_publisher(Odometry, agent_name + '/odom', 10)
        self.create_subscription(PoseWithCovarianceStamped,
        agent_name + '/odom_combined',
        self.pub_ekf_odom,
        10)

        self.get_logger().info('odom_ekf node started')

    def pub_ekf_odom(self, msg: PoseWithCovarianceStamped):
        o = Odometry()
        o.header = msg.header
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.pose = msg.pose
        self.ekf_pub.publish(o)


def main(args=None):
    rclpy.init(args=args)
    node = OdomEKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
