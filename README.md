# If you are going to clone the repository:

Since this repo uses submodules, they need to be cloned as well. To do this run:

```
git clone --recurse-submodules https://github.com/MDU-C2/SLaRC-HeavySeeker-HT25.git
```

# Click-to-goal Foxglove Extension

This extension lets you click waypoints (currently only one point) on a map in Foxglove Desktop and
publishes them as ROS 2 messages (e.g., `/goal_geo`) for our robot stack

## Requirements

### 1. Foxglove Desktop  
Download from:  
https://foxglove.dev/download

### 2. Node.js + npm  
This extension requires Node and npm to build.

Recommended versions (used in development):

node v18.19.1
npm v9.2.0

```bash
sudo apt install nodejs npm
```

## ROS 2 Foxglove Bridge

If you run our standard Docker setup:
➡️ Already installed inside the container (no action needed).

If running natively on your system:

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

## One-time setup (install extension)

```bash
cd foxglove_extensions/click-to-goal
./setup_extension.sh
```

Restart Foxglove Desktop afterwards.

## How to use the extension

Currently only supported in simulation:
```bash
ros2 launch seeker_sim seeker_sim.launch.py model:=Rig5 world:=sonoma_raceway use_foxglove:=true
```

use_foxglove default value is set to true

## Open Foxglove Desktop

Add a new connection using:
ws://localhost:8765

Open the “Click-to-goal” panel (from the extension)

Click on the map to send waypoints as a ros topic 🎯

## Rebuilding the extension (after updates)

If you pull new changes:
```bash
cd foxglove_extensions/click-to-goal
npm run build
npm run local-install
```



# Running the lidars

First, [enter the container](docker/README.md), then build by running:

```
$ /slarc_ws/livox_build.sh
```

_Don't worry about the warnings, just ignore them_

**To start the lidars, they need to first be powered and connected.**

> [!NOTE]
> You need to set the wired IPv4 address to _192.168.10.222_ and the netmask to _255.255.255.0_ in order to be able to have a connection with the lidars.

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
> While it is possible to launch _cartographer_ without any errors, it does not do anything useful at this moment other than run rviz with a subsriber to one of the lidars.

Happy debugging :)
