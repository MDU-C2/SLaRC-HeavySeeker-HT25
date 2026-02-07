#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/sfd/dev/slarc-heavyseeker/ros_ws/install/setup.bash #change this path

zenohd &

ros2 launch s_robot robot.launch.py