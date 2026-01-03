#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class CropSelfFilter(Node):
    def __init__(self):
        super().__init__("oakd_self_filter")

        self.declare_parameter("input_topic", "/oakd_points_1_fixed")
        self.declare_parameter("output_topic", "/oakd_points_1_filtered")
        self.declare_parameter("target_frame", "base_link")

        # Crop box (in target_frame coordinates)
        self.declare_parameter("min_x", -0.55)
        self.declare_parameter("max_x",  0.55)
        self.declare_parameter("min_y", -0.50)
        self.declare_parameter("max_y",  0.50)
        self.declare_parameter("min_z", -0.30)
        self.declare_parameter("max_z",  0.60)

        self.in_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.target_frame = self.get_parameter("target_frame").value

        self.min_x = float(self.get_parameter("min_x").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.min_y = float(self.get_parameter("min_y").value)
        self.max_y = float(self.get_parameter("max_y").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointCloud2, self.out_topic, 10)
        self.sub = self.create_subscription(PointCloud2, self.in_topic, self.cb, 10)

        self.get_logger().info(f"Filtering {self.in_topic} -> {self.out_topic} in frame {self.target_frame}")
        self.get_logger().info(f"Crop self box: x[{self.min_x},{self.max_x}] y[{self.min_y},{self.max_y}] z[{self.min_z},{self.max_z}]")

    def cb(self, msg: PointCloud2):
        try:
            # Transform cloud into target frame
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
            cloud_bl = do_transform_cloud(msg, tf)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        # Read points
        pts = []
        for p in pc2.read_points(cloud_bl, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])

            # Remove points inside the robot box
            inside = (self.min_x <= x <= self.max_x and
                      self.min_y <= y <= self.max_y and
                      self.min_z <= z <= self.max_z)
            if not inside:
                pts.append((x, y, z))

        # Create filtered cloud (still in target_frame)
        header = cloud_bl.header
        out = pc2.create_cloud_xyz32(header, pts)
        self.pub.publish(out)

def main():
    rclpy.init()
    node = CropSelfFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

