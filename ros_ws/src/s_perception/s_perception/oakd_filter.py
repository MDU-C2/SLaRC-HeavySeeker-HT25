#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

def quat_to_rot_matrix(qx, qy, qz, qw):
    # Quaternion -> rotation matrix (3x3)
    # Assumes normalized quaternion (TF usually is)
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return np.array([
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)]
    ], dtype=np.float64)

class CropSelfFilter(Node):
    def __init__(self):
        super().__init__("oakd_self_filter")

        self.declare_parameter("input_topic", "/oakd_points_1_fixed")
        self.declare_parameter("output_topic", "/oakd_points_1_filtered")
        self.declare_parameter("target_frame", "base_link")

        # Crop box in target_frame (base_link)
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

        self.get_logger().info(f"Filtering {self.in_topic} -> {self.out_topic} in {self.target_frame}")
        self.get_logger().info(
            f"Crop self box: x[{self.min_x},{self.max_x}] y[{self.min_y},{self.max_y}] z[{self.min_z},{self.max_z}]"
        )

    def cb(self, msg: PointCloud2):
        # Use the cloud timestamp for TF lookup (important when turning!)
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,         # target
                msg.header.frame_id,       # source
                stamp
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Build transform (R, t)
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=np.float64)

        # Read xyz points (ignore all other fields safely)
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Force a plain Nx3 float array
        pts_list = [(float(x), float(y), float(z)) for (x, y, z) in gen]
        if not pts_list:
            return

        pts = np.asarray(pts_list, dtype=np.float64)
        if pts.size == 0:
            return

        # Transform into base_link: p' = R p + t
        pts_bl = (pts @ R.T) + trans

        # Remove points inside robot box
        x = pts_bl[:, 0]
        y = pts_bl[:, 1]
        z = pts_bl[:, 2]
        inside = (
            (x >= self.min_x) & (x <= self.max_x) &
            (y >= self.min_y) & (y <= self.max_y) &
            (z >= self.min_z) & (z <= self.max_z)
        )
        pts_out = pts_bl[~inside]

        # Publish filtered cloud in base_link frame
        header = msg.header
        header.frame_id = self.target_frame
        out_msg = pc2.create_cloud_xyz32(header, pts_out.astype(np.float32))
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    node = CropSelfFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
