# If you are going to clone the repository:

Since this repo uses submodules, they need to be cloned as well. To do this run:

```
git clone --recurse-submodules https://github.com/MDU-C2/SLaRC-HeavySeeker-HT25.git
```

# Running the lidars
## 1.1 Prerequisites
* OS:
  * Linux: Ubuntu 18.04 or above

* Tools:
  * compilers that support C++11
  * cmake 3.0+

* Arch:
  * x86
  * ARM


## 1.2 Instruction for Ubuntu 24.04
If you are using the [docker container](docker/README.md), step 1 & 2 are already taken care of.

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
5. Start the lidars
* Make sure it is powerd and connected.
> [!WARNING]
> Do not connect the ethernet cables to a Power over Ethernet (PoE) port and make sure the lidars are not in view of each other or they will get damaged
* Set the wired IPv4 address to _192.168.10.222_ and the netmask to _255.255.255.0_ in order to be able to have a connection with the lidars, or [alter their configurations](ros_ws/src/s_perception/config/MID360_config.json).

6. Connect to lidars
From ros workspace:
```shell
$ source install/setup.bash
$ ros2 launch s_perception livox_launch.py 
```

Happy debugging :)
