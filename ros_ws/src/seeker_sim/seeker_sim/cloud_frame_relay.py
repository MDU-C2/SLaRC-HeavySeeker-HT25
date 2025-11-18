# cloud_frame_relay.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

IN  = '/lidar_points'       # or the name you bridged to
OUT = '/lidar_points_fixed'
FRAME = 'mid360_lidar_link'

IN_2  = '/oakd_points'
OUT_2 = '/oakd_points_fixed'
FRAME_2 = 'oakd_pro_rgb_camera_optical_frame'

class Relay(Node):
    def __init__(self):
        super().__init__('cloud_relay')
        self.pub_lidar = self.create_publisher(PointCloud2, OUT, 10)
        self.sub_lidar = self.create_subscription(PointCloud2, IN, self.cb_lidar, 10)

        self.pub_oakd = self.create_publisher(PointCloud2, OUT_2, 10)
        self.sub_oakd = self.create_subscription(PointCloud2, IN_2, self.cb_oakd, 10)

    def cb_lidar(self, msg):
        msg.header.frame_id = FRAME
        self.pub_lidar.publish(msg)

    def cb_oakd(self, msg):
        msg.header.frame_id = FRAME_2
        self.pub_oakd.publish(msg)

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
