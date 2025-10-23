# If you are going to clone the livox_lidar branch:
Since this branch uses submodules, they need to be cloned as well. To do this run:
```
git clone -b livox_lidar --recurse-submodules https://github.com/MDU-C2/SLaRC-HeavySeeker-HT25.git
```

# Running the lidars
First, [enter the container](docker/README.md), then build by running:
```
$ /slarc_ws/livox_build.sh
```
**To start the lidars, they need to first be powered and connected.**

Afterwards, source and start the publishers:
```
$ cd /slarc_ws
$ source install/setup.bash
$ ros2 launch hs_bringup livox_launch.py
```

To start rviz with nav2 and cartographer, open a new terminal in the container and run:
```
$ ros2 launch hs_bringup cartographer.launch.py
```

Happy debugging :)