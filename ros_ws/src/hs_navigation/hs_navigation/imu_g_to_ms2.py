#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuAccConv(Node):

    def __init__(self):
        super().__init__('imu_acc_conv')
        self.sub = self.create_subscription(Imu, 'imu/data', self.cb_conv, 10)
        self.pub = self.create_publisher(Imu, 'imu_conv/data', 10)
        self.g = 9.81

    def cb_conv(self, msg):
        lin_acc = msg.linear_acceleration
        lin_acc.x = lin_acc.x * self.g
        lin_acc.y = lin_acc.y * self.g
        lin_acc.z = lin_acc.z * self.g

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ImuAccConv()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Interrupt caught, allowing rclpy to shutdown.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
