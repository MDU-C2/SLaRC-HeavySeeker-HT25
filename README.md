# If you are going to clone the repository:

Since this repo uses submodules, they need to be cloned as well. To do this run:

```
git clone --recurse-submodules https://github.com/MDU-C2/SLaRC-HeavySeeker-HT25.git
```
If you've already cloned without submodules, initialize them separately:
```
git submodule update --init --recursive
```

# Running the robot
This section is a guide on how to build and run on the onboard computer. For client-side, see [this section](#running-client-side-interface)
## 1.1 Prerequisites
* OS:
  * Linux: Ubuntu Server 18.04 or above

* Tools:
  * compilers that support C++11
  * cmake 3.0+

* Arch:
  * x86
  * ARM


## 1.2 Instruction for Ubuntu Server 24.04
If you are using the [docker container](docker/README.md), steps 1 & 2 are already taken care of.

1. Dependencies:

* [CMake 3.0.0+](https://cmake.org/)
* gcc 4.8.1+
* [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

2. Install dependencies using apt:

```shell
$ sudo apt update && apt install -y \
    curl  \
    lsb-release \
    gnupg \
    build-essential \
    cmake \
    libpcl-dev \
    gh \
    python3-pip \
    pcl-tools

$ sudo apt update && apt install -y \
    ros-dev-tools \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-route \
    ros-jazzy-depthai-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-msgs \
    ros-jazzy-pcl-ros \
    ros-jazzy-septentrio-gnss-driver \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-usb-cam \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-robot-localization \
    ros-jazzy-tile-map \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-tf-transformations \
    ros-jazzy-spatio-temporal-voxel-layer
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

4. Build
From ros workspace:
```shell
$ colcon build
```
5. Connecting to the lidar
* Make sure it is powerd and connected.
> [!WARNING]
> Do not connect the ethernet cable to a Power over Ethernet (PoE) port and if using multiple, make sure the lidars are not in view of each other or they will get damaged
* Set the wired IPv4 address to _192.168.10.222_ and the netmask to _255.255.255.0_ in order to be able to have a connection with the lidars, or [alter their configurations](ros_ws/src/s_perception/config/MID360_config.json).

6. Source and launch
From ros workspace:
```shell
$ source install/setup.bash
$ ros2 launch s_bringup main.launch.py
```