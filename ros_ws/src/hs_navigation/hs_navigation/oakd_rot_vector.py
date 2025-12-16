#!/usr/bin/env python3
import depthai as dai
import math
import rclpy

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import threading
import tf_transformations as tf


class OakdRotVector(Node):
    def __init__(self):
        super().__init__('oakd_rotation_vec')
        self.imu_pub = self.create_publisher(Imu, 'oakd/imu/rotation', 10)

    def publish_quaternion(self, x, y, z, w):
        msg = Imu()
        orientation = Quaternion()
        old_orientation = (x, y, z, w)
        rotation = tf.quaternion_from_euler(0.0, 0.0, math.pi / 2)
        (orientation.x, orientation.y, orientation.z, orientation.w) = tf.quaternion_multiply(rotation, old_orientation)
        msg.orientation = orientation
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = OakdRotVector()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.daemon = True
    spin_thread.start()
    try:
        # DepthAI v3-style pipeline with no explicit Device()
        with dai.Pipeline() as pipeline:
            # IMU node
            imu = pipeline.create(dai.node.IMU)

            # Enable fused ROTATION_VECTOR at 400 Hz
            imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)

            # Host output queue directly from the node (no XLinkOut in v3)
            imu_queue = imu.out.createOutputQueue(maxSize=50, blocking=False)

            # Start the pipeline (this implicitly connects to the device)
            pipeline.start()

            print("Reading IMU rotation vector and computing magnetic heading...")
            while pipeline.isRunning():
                imuData = imu_queue.get()          # blocking call
                for pkt in imuData.packets:
                    rv = pkt.rotationVector        # quaternion from BNO086

                    node.publish_quaternion(rv.i, rv.j, rv.k, rv.real)
                    
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Interrupt caught, allowing rclpy to shutdown.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
