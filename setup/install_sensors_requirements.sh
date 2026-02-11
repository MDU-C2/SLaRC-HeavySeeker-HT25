#! /bin/bash

### ROS Workspace ###

# Repository with ros_ws folder
cd ~
git clone https://github.com/MDU-C2/SLaRC-HeavySeeker-HT25.git slarc
cd ~/slarc
git submodule update --init --recursive

### non ROS sensor requirements ###

# Camera
sudo ufw allow 5600 # FPV Camera

# LiDAR (Build/install livox sdk)
./build_livoxSDK.sh

# GNSS add septentio to netplan
sudo netplan set ethernets.enx1a3202991545.dhcp4=true
sudo netplan apply

### ROS dependencies ###
cd ~/slarc/ros_ws
rosdep install --from-path src --ignore-src -y -r

#echo "source ~/slarc/ros_ws/install/setup.bash" >> ~/.bashrc
