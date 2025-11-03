The files derived from livox_ros_driver2 (MIT) are as follows:
Commit: 6b9356cadf77084619ba406e6a0eb41163b08039
livox_ros_driver2
├── config
│   ├── display_point_cloud_ROS2.rviz
│   └── MID360_config.json
└── launch_ROS2
    └── rviz_MID360_launch.py


The following is an excerpt from livox_ros_driver2 license relevant to this package:

The following portions of the LIVOX’s Livox ROS Driver2 (“Software” referred to in the terms below) are made available to you under the terms of the MIT License provided below and is also available at https://opensource.org/licenses/MIT.  

livox_ros_driver2
├── build.sh
├── cmake
│   └── version.cmake
├── CMakeLists.txt
├── config
│   ├── display_point_cloud_ROS1.rviz
│   ├── display_point_cloud_ROS2.rviz
│   ├── HAP_config.json
│   ├── MID360_config.json
│   └── mixed_HAP_MID360_config.json
├── launch_ROS1
│   ├── msg_HAP.launch
│   ├── msg_MID360.launch
│   ├── msg_mixed.launch
│   ├── rviz_HAP.launch
│   ├── rviz_MID360.launch
│   └── rviz_mixed.launch
├── launch_ROS2
│   ├── msg_HAP_launch.py
│   ├── msg_MID360_launch.py
│   ├── rviz_HAP_launch.py
│   ├── rviz_MID360_launch.py
│   └── rviz_mixed.py
├── msg
│   ├── CustomMsg.msg
│   └── CustomPoint.msg
├── package_ROS1.xml
├── package_ROS2.xml
├── package.xml
├── README.md
└── src
    ├── call_back
    │   ├── lidar_common_callback.cpp
    │   ├── lidar_common_callback.h
    │   ├── livox_lidar_callback.cpp
    │   └── livox_lidar_callback.h
    ├── comm
    │   ├── cache_index.cpp
    │   ├── cache_index.h
    │   ├── comm.cpp
    │   ├── comm.h
    │   ├── ldq.cpp
    │   ├── ldq.h
    │   ├── lidar_imu_data_queue.cpp
    │   ├── lidar_imu_data_queue.h
    │   ├── pub_handler.cpp
    │   ├── pub_handler.h
    │   ├── semaphore.cpp
    │   └── semaphore.h
    ├── driver_node.cpp
    ├── driver_node.h
    ├── include
    │   ├── livox_ros_driver2.h
    │   ├── ros1_headers.h
    │   ├── ros2_headers.h
    │   └── ros_headers.h
    ├── lddc.cpp
    ├── lddc.h
    ├── lds.cpp
    ├── lds.h
    ├── lds_lidar.cpp
    ├── lds_lidar.h
    ├── livox_ros_driver2.cpp
    └── parse_cfg_file
        ├── parse_cfg_file.cpp
        ├── parse_cfg_file.h
        ├── parse_livox_lidar_cfg.cpp
        └── parse_livox_lidar_cfg.h

---------------------------------

The MIT License (MIT)

Copyright (c) 2022 Livox. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.