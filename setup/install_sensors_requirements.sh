#! /bin/bash

### non ROS sensor requirements ###

# Camera

# LiDAR (Build/install livox sdk)
./build_livoxSDK.sh

# GNSS add septentio to netplan
sudo netplan set ethernets.enx1a3202991545.dhcp4=true
sudo netplan apply


### ROS Workspace ###

# Repository with ros_ws folder
cd ~
git clone https://github.com/MDU-C2/SLaRC-HeavySeeker-HT25.git slarc
cd ~/slarc
git submodule update --init --recursive

# Install all ros dependencies
cd ~/slarc/ros_ws
rosdep install --from-path src --ignore-src -y -r

#echo "source ~/slarc-heavyseeker/ros_ws/install/setup.bash" >> ~/.bashrc