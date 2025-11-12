#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class FlipperNode(Node):

    def __init__(self):
        super().__init__('imu_flipper')
        self.sub = self.create_subscription(Imu, 'imu/data', self.cb_flip, 10)
        self.pub = self.create_publisher(Imu, 'imu_flipped/data', 10)

    def cb_flip(self, msg):
        # flip the y axis
        msg.orientation.z = -msg.orientation.z
        msg.angular_velocity.z = -msg.angular_velocity.z
        msg.linear_acceleration.z = -msg.linear_acceleration.z

        # invert the covariances
        # orientation
        orientation_cov = msg.orientation_covariance
        orientation_cov[2] = -orientation_cov[2]
        orientation_cov[5] = -orientation_cov[5]
        orientation_cov[6] = -orientation_cov[6]
        orientation_cov[7] = -orientation_cov[7]

        # angular_velocity
        ang_vel_cov = msg.angular_velocity_covariance
        ang_vel_cov[2] = -ang_vel_cov[2]
        ang_vel_cov[5] = -ang_vel_cov[5]
        ang_vel_cov[6] = -ang_vel_cov[6]
        ang_vel_cov[7] = -ang_vel_cov[7]

        # linear_acceleration
        lin_acc_cov = msg.linear_acceleration_covariance
        lin_acc_cov[2] = -lin_acc_cov[2]
        lin_acc_cov[5] = -lin_acc_cov[5]
        lin_acc_cov[6] = -lin_acc_cov[6]
        lin_acc_cov[7] = -lin_acc_cov[7]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = FlipperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt caught, allowing rclpy to shutdown.')
    finally:
        # node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
