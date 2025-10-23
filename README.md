# If you are going to clone the repository:
Since this repo uses submodules, they need to be cloned as well. To do this run:
```
git clone --recurse-submodules https://github.com/MDU-C2/SLaRC-HeavySeeker-HT25.git
```

# Running the lidars
First, [enter the container](docker/README.md), then build by running:
```
$ /slarc_ws/livox_build.sh
```
**To start the lidars, they need to first be powered and connected.**

> [!NOTE]
> You need to set the wired IPv4 address to *192.168.1.50* and the netmask to *255.255.255.0* in order to be able to have a connection with the lidars.

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

> [!NOTE]
> While it is possible to launch *cartographer* without any errors, it does not do anything useful at this moment other than run rviz with a subsriber to one of the lidars.

Happy debugging :)