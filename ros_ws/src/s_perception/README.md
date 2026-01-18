# Seeker navigation package

This package boots and runs the drivers and helpers for the sensors integrated with **seeker** UGV.

## Installation
### 1.1 Prerequisites
* OS:
  * Linux: Ubuntu 18.04 or above

* Tools:
  * compilers that support C++11
  * cmake 3.0+

* Arch:
  * x86
  * ARM


### 1.2 Instruction for Ubuntu 24.04
If you are using the [docker container](docker/README.md), steps 1 & 2 are already taken care of.

1. Dependencies:

* [CMake 3.0.0+](https://cmake.org/)
* gcc 4.8.1+
* [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

2. Install the **CMake** using apt:

```shell
$ sudo apt install cmake
```

3. Compile and install the Livox-SDK2:
From ros workspace:
```shell
$ cd ../third_party/Livox-SDK2/
$ mkdir build && cd build
$ cmake .. && make -j$(nproc --ignore=1)
$ sudo make install
```
_Don't worry about the warnings, just ignore them_

> [!NOTE] 
> The generated shared library and static library are installed to the directory of "/usr/local/lib". The header files are installed to the directory of "/usr/local/include".

To remove Livox SDK2:

```shell
$ sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
$ sudo rm -rf /usr/local/include/livox_lidar_*
```

4. Build Livox ROS drivers
From ros workspace:
```shell
$ colcon build --packages-select livox_ros_driver2 s_perception
```
5. Connecting to the lidar
* Make sure it is powered and connected.
> [!WARNING]
> Do not connect the Ethernet cable to a Power over Ethernet (PoE) port and if using multiple, make sure the lidars are not in view of each other or they will get damaged
* Set the wired IPv4 address to _192.168.10.222_ and the netmask to _255.255.255.0_ in order to be able to have a connection with the lidars, or [alter their configurations](ros_ws/src/s_perception/config/MID360_config.json).

6. Source and launch lidar driver
From ros workspace:
```shell
$ source install/setup.bash
$ ros2 launch s_perception livox_launch.py 
```

Happy debugging :)

## Quick Start
`ros2 launch s_perception s_perception.launch.py` - launches Livox Mid-360 lidar and ArduSimple GNSS modules, as well as the helper modules to integrate with `s_navigation` package.

## Launch
### [`s_perception.launch.py`](./launch/s_perception.launch.py)
Launch **seeker**'s perception stack. The stack includes ArduSimple GNSS driver, Livox Mid-360 driver and custom nodes from this package.

Arguments:

- `livox_config: relative_path` - file in `config` directory with configuration for Livox Mid-360 lidar. Default: `"MID360_config.json"`
- `ardu_config: relative_path` - file in `config` directory with configuration for ArduSimple. Default: `"ardu_config.yaml"`
- `namespace: str` - Robot namespace. Default: `"/"`

### [`ardu.launch.py`](./launch/ardu.launch.py)
Launch ArduSimple GNSS module. After launch the WebUI can be found at `192.168.3.1`.

Arguments:

- `ardu_config: relative_path` - file in `config` directory with configuration for ArduSimple. Default: `"ardu_config.yaml"`

### [`cloud2scan.launch.py`](./launch/cloud2scan.launch.py)
Launch a helper node to convert [`sensor_msgs/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) to [`sensor_msgs/LaserScan`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/LaserScan.html).

Arguments:

- `cloud_topic: str` - the topic that publishes [`sensor_msgs/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) messages. Default: `"livox/lidar_192_168_10_93"`
- `target_frame: str` - frame in the converted messages. Default: `"livox_frame"`

### [`livox_launch.py`](./launch/livox_launch.py)
Launch Livox Mid-360 driver.

Arguments:

- `xfer_format: int` - transfer format: 0 - Pointcloud2(PointXYZRTL), 1 - customized pointcloud format (better for `fast_lio`). Default: `0`
- `multi_topic: int` - multi topic: 0 - All LiDARs share the same topic, 1 - One LiDAR one topic. Default: `0`
- `data_src: int` - fata source: 0 - lidar, others - Invalid data src. Default: `0`
- `publish_freq: float` - publish frequency. Default: `10.0`
- `output_data_type: int` - output data type. Default: `0`
- `frame_id: string` - frame ID. Default: `"livox_frame"`
- `lvx_file_path: absolute_path` - LVX file path. Default: `"/home/livox/livox_test.lvx"`
- `user_config_path: absolute_path` - user configuartion file path. Default: `"/slarc_ws/install/s_perception/share/s_perception/launch/../config/MID360_config.json"`
- `cmdline_input_bd_code: str` - command line BD code. Default: `"livox0000000001"`

## Configuration files
- [`ardu_config.yaml`](./config/ardu_config.yaml) - default configuration for ArduSimple GNSS driver.
- [`MID360_config.json`](./config/MID360_config.json) - default configuration file for Livox Mid-360 lidar driver.
- [`param_config.json`](./config/param_config.json) - default configuration file for [`livox_launch.py`](./launch/livox_launch.py)

## Nodes
### [`gps_heading.py`](./s_perception/gps_heading.py)
This node extracts data provided by ArduSimple to generate [`sensor_msgs/IMU`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html) with correct orientation and the same frame id.

Subscriptions:

- `atteuler`:[`septentrio_gnss_driver/AttEuler`](https://docs.ros.org/en/ros2_packages/jazzy/api/septentrio_gnss_driver/msg/AttEuler.html) - attitude in Euler angles

Publishers:

- `gps/heading/imu`:[`sensor_msgs/IMU`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html) - converted [`sensor_msgs/IMU`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html) messages

### [`imu_g_to_ms2.py`](./s_perception/imu_g_to_ms2.py)
This node converts linear acceleration in [`sensor_msgs/IMU`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html) messages from _g_ to _m/s^2_.

Subscriptions:

- `imu/data`:[`sensor_msgs/IMU`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html) - original [`sensor_msgs/IMU`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html) messages

Publishers:

- `imu_conv/data`:[`sensor_msgs/IMU`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html) - converted [`sensor_msgs/IMU`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html) messages

### [`oakd_filter.py`](./s_perception/oakd_filter.py)
This node crops the input [`sensor_msgs/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html).

Parameters:

- `input_topic: str` - topic with [`sensor_msgs/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) to crop. Default: `"oakd_points_1_fixed"`
- `output_topic: str` - topic with cropped [`sensor_msgs/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html). Default: `"/oakd_points_1_filtered"`
- `target_frame: str` - frame ID of the cropped [`sensor_msgs/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html).
- `min_x: float` - the lower `x` border of the bounding box
- `max_x: float` - the higher `x` border of the bounding box
- `min_y: float` - the lower `y` border of the bounding box
- `max_y: float` - the higher `y` border of the bounding box
- `min_z: float` - the lower `z` border of the bounding box
- `max_z: float` - the higher `z` border of the bounding box

Subscriptions:
- `<input_topic>`:[`sensor_msgs/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) - the cloud to crop

Publishers:
- `<output_topic>`:[`sensor_msgs/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) - the cropped cloud

## Resources
### Docs
- [`livox_ros_driver2`](https://github.com/tu-darmstadt-ros-pkg/livox_ros_driver2/tree/jazzy) - Livox Mid-360 ROS2 driver
- [`septentrio_gnss_driver`] - ArduSimple ROS2 driver
### Useful links
- [Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)