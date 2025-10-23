# cloud_frame_relay.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

IN  = '/lidar/mid360/points'       # or the name you bridged to
OUT = '/lidar/mid360/points_fixed'
FRAME = 'lidar'

class Relay(Node):
    def __init__(self):
        super().__init__('cloud_relay')
        self.pub = self.create_publisher(PointCloud2, OUT, 10)
        self.sub = self.create_subscription(PointCloud2, IN, self.cb, 10)

    def cb(self, msg):
        msg.header.frame_id = FRAME
        self.pub.publish(msg)

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
