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
        rotation = tf.quaternion_from_euler(math.pi / 2.0, 0.0, math.pi / 2.0, "sxyz")
        # R = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        # rotation = tf.quaternion_from_matrix(R)
        # rotation = (0.0, 0.0, 0.7071067811865475, 0.7071067811865475)
        (new_orientation.x, new_orientation.y, new_orientation.z,
         new_orientation.w) = tf.quaternion_multiply(old_orientation, rotation)
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
