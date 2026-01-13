#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import copy
import tf_transformations as tf
import math
import numpy as np


class OakdImuRotataion(Node):
    def __init__(self):
        super().__init__('oakd_imu_rotation')
        self.imu_sub = self.create_subscription(
            Imu, 'oakd/imu/data', self.imu_callback, 10)
        self.imu_pub = self.create_publisher(Imu, 'oakd/imu_rot/data', 10)

    def imu_callback(self, msg):
        imu = copy.deepcopy(msg)
        old_orientation = (msg.orientation.x, msg.orientation.y,
                           msg.orientation.z, msg.orientation.w)
        new_orientation = Quaternion()

        # a bunch of frame transformations to fix the driver's outputed yaw
        R = tf.quaternion_matrix(tf.quaternion_inverse(old_orientation))[:3, :3]

        # +90 degrees rotation about z axis - fixes the yaw, but the axis' direction is opposite
        R_des2enu = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ])
        R_enu = R_des2enu @ R
        q_old = tf.quaternion_from_matrix(np.pad(R_enu, ((0, 1), (0, 1)), 'constant', constant_values=0))
        rotated_q = tf.quaternion_matrix(tf.quaternion_multiply(q_old, tf.quaternion_about_axis(math.pi, [0, 0, 1])))[:3, :3]

        # invert z axis, so the yaw sign is correct
        S = np.diag((-1, 1, 1))
        new_q = S @ rotated_q @ S
        inverted_q = tf.quaternion_from_matrix(np.pad(new_q, ((0, 1), (0, 1)), 'constant', constant_values=0))

        # rotate it 90 degrees to right, so east is 0
        (new_orientation.x, new_orientation.y, new_orientation.z, new_orientation.w) = tf.quaternion_multiply(inverted_q, tf.quaternion_about_axis(math.pi / 2, [0, 0, 1]))
        imu.orientation = new_orientation
        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)

    node = OakdImuRotataion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Interrupt caught, allowing rclpy to shutdown.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
