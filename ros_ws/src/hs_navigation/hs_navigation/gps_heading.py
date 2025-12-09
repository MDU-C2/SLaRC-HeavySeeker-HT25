#!/usr/bin/python3

import rclpy

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from septentrio_gnss_driver.msg import AttEuler
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from math import radians, isnan
from tf_transformations import quaternion_from_euler
from copy import copy


class GPSHeadingToIMU(Node):
    def __init__(self):
        super().__init__(node_name='gps_heading')
        self.qos = 10
        self.gps_sub = self.create_subscription(
            AttEuler, 'atteuler', self.atteuler_callback, self.qos)
        self.gps_pub = self.create_publisher(Imu, 'gps/heading/imu', self.qos)

    def atteuler_callback(self, msg):
        if msg.error == 1:
            return
        imu = Imu()
        imu.header = copy(msg.header)
        orientation = Quaternion()
        angular_velocity = Vector3()
        roll = radians(num_or_default(msg.roll))
        pitch = radians(num_or_default(msg.pitch))
        yaw = radians(num_or_default(msg.heading))
        angular_velocity.x = radians(num_or_default(msg.roll_dot))
        angular_velocity.y = radians(num_or_default(msg.pitch_dot))
        angular_velocity.z = radians(num_or_default(msg.heading_dot))
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_euler(
            roll, pitch, yaw)
        imu.orientation = orientation
        imu.angular_velocity = angular_velocity
        self.gps_pub.publish(imu)


def num_or_default(val, default=0.0):
    return (val if not isnan(val) else default)


def main():
    rclpy.init()
    gps_heading = GPSHeadingToIMU()
    try:
        rclpy.spin(gps_heading)
    except (KeyboardInterrupt, ExternalShutdownException):
        gps_heading.get_logger().info('Interrupt caught, allowing rclpy to shutdown.')
    finally:
        gps_heading.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
