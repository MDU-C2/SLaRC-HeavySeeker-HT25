#!/usr/bin/env python3
import depthai as dai
import math

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

            # Quaternion components
            x = rv.i
            y = rv.j
            z = rv.k
            w = rv.real

            # Quaternion -> yaw (heading) in radians
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # Convert to degrees and normalize to [0, 360)
            heading_deg = math.degrees(yaw)
            if heading_deg < 0.0:
                heading_deg += 360.0

            # This is heading vs magnetic north in the IMU's frame
            print(f"Heading vs magnetic north: {heading_deg:6.1f}°")
