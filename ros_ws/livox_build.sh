#!/bin/bash
#Synchronize and update git submodules
cd /
git submodule sync --recursive
git submodule update --init --recursive

# Downloand and build/install Livox-SDK2
cd /third_party_sdk_drivers/Livox-SDK2/
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

# Configure dynamic linker run-time bindings
echo /usr/local/lib | sudo tee /etc/ld.so.conf.d/livox-sdk2.conf
sudo ldconfig

# Source
source /slarc_ws/install/setup.bash