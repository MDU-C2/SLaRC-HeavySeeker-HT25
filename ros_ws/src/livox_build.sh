#!/bin/bash
# Downloand and build/install Livox-SDK2
cd /slarc_ws/src/Livox-SDK2/
cat sdk_core/logger_handler/file_manager.h | grep '#include <cstdint>' || sed -i '31i #include <cstdint>' sdk_core/logger_handler/file_manager.h
cat sdk_core/comm/define.h | grep '#include <cstdint>' || sed -i '34i #include <cstdint>' sdk_core/comm/define.h
mkdir build
cd build
cmake .. && make -j
sudo make install
cd /

# Download and build livox_ros_driver2
cd /slarc_ws/src/livox_ros_driver2
source /opt/ros/jazzy/setup.bash
./build.sh humble
cd /