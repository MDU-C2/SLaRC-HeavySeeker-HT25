#! /usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry

class GpsOdomNode(Node):
    def __init__(self):
        super().__init__(node_name='gps_odom_node')
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'pose', self.pose_cb, 10)
        self.twist_sub = self.create_subscription(TwistWithCovarianceStamped, 'twist', self.twist_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.01, self.timer_cb)
        self.pose = None
        self.twist = None


    def twist_cb(self, msg):
        self.twist = msg
    
    def pose_cb(self, msg):
        self.pose = msg
    
    def timer_cb(self):
        if self.twist is not None and self.pose is not None:
            msg = Odometry()
            msg.header = self.pose.header

            msg.child_frame_id = self.twist.header.frame_id
            msg.pose = self.pose.pose
            msg.twist = self.twist.twist

            self.odom_pub.publish(msg=msg)


def main(args=None):
    rclpy.init(args=args)

    node = GpsOdomNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Interrupt caught, allowing rclpy to shutdown.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()