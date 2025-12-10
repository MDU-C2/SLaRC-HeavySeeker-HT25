#!/usr/bin/env python3
# cloud_frame_relay.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

IN  = '/lidar_points'       # or the name you bridged to
OUT = '/lidar_points_fixed'
FRAME = 'livox_frame'

IN_2  = '/oakd_pro_1/oakd/rgbd/points'
OUT_2 = '/oakd_points_1_fixed'
FRAME_2 = 'oakd_pro_1_rgb_camera_frame'

IN_3  = '/oakd_pro_2/oakd/rgbd/points'
OUT_3 = '/oakd_points_2_fixed'
FRAME_3 = 'oakd_pro_2_rgb_camera_frame'

class Relay(Node):
    def __init__(self):
        super().__init__('cloud_relay')
        self.pub_lidar = self.create_publisher(PointCloud2, OUT, 10)
        self.sub_lidar = self.create_subscription(PointCloud2, IN, self.cb_lidar, 10)

        self.pub_oakd_1 = self.create_publisher(PointCloud2, OUT_2, 10)
        self.sub_oakd_1= self.create_subscription(PointCloud2, IN_2, self.cb_oakd1, 10)

        self.pub_oakd_2 = self.create_publisher(PointCloud2, OUT_3, 10)
        self.sub_oakd_2= self.create_subscription(PointCloud2, IN_3, self.cb_oakd2, 10)

    def cb_lidar(self, msg):
        msg.header.frame_id = FRAME
        self.pub_lidar.publish(msg)

    def cb_oakd1(self, msg):
        msg.header.frame_id = FRAME_2
        self.pub_oakd_1.publish(msg)

    def cb_oakd2(self, msg):
        msg.header.frame_id = FRAME_3
        self.pub_oakd_2.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = Relay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
