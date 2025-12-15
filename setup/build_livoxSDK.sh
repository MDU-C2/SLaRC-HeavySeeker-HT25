#!/bin/bash

# Build/install Livox-SDK2
cd ~/slarc/third_party_sdk_drivers/Livox-SDK2/
cat sdk_core/logger_handler/file_manager.h | grep '#include <cstdint>' || sed -i '31i #include <cstdint>' sdk_core/logger_handler/file_manager.h
cat sdk_core/comm/define.h | grep '#include <cstdint>' || sed -i '34i #include <cstdint>' sdk_core/comm/define.h
mkdir build
cd build
cmake .. && make -j
sudo make install

# Configure dynamic linker run-time bindings
echo /usr/local/lib | sudo tee /etc/ld.so.conf.d/livox-sdk2.conf
sudo ldconfig