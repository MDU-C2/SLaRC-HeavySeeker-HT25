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
import numpy as np


class OakdRotVector(Node):
    def __init__(self):
        super().__init__('oakd_rotation_vec')
        self.imu_pub = self.create_publisher(Imu, 'oakd/imu/rotation', 10)
        self.pure_imu_pub = self.create_publisher(Imu, 'oakd/imu/data', 10)

    def publish_quaternion(self, x, y, z, w):
        msg1 = Imu()
        msg2 = Imu()
        orientation1 = Quaternion()
        orientation2 = Quaternion()
        frame = 'oakd_imu_frame'

        R = tf.quaternion_matrix(tf.quaternion_inverse((x, y, z, w)))[:3, :3]
        R_wsd2enu = np.array([
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        R_enu = R_wsd2enu @ R
        q_old = tf.quaternion_from_matrix(np.pad(R_enu, ((0, 1), (0, 1)), 'constant', constant_values=0))

        (orientation1.x, orientation1.y, orientation1.z,
         orientation1.w) = tf.quaternion_multiply(q_old, tf.quaternion_about_axis(math.radians(90.0), [0, 0, 1]))
        
        (orientation2.x, orientation2.y,
         orientation2.z, orientation2.w) = q_old
        
        msg1.orientation = orientation1
        # msg1.orientation_covariance[0] = -1.0
        # msg1.orientation_covariance[4] = -1.0
        # msg1.orientation_covariance[8] = -1.0
        msg1.header.frame_id = frame
        msg2.orientation = orientation2
        msg2.header.frame_id = frame
        t = self.get_clock().now().to_msg()
        msg1.header.stamp = t
        msg2.header.stamp = t
        self.imu_pub.publish(msg1)
        self.pure_imu_pub.publish(msg2)


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
