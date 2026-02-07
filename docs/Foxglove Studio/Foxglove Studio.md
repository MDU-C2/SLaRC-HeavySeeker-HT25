# Foxglove Studio
Foxglove Studio is recommended to use the waypoint navigation on the UGV. 
The custom panel map panel Click-to-Goal has been written to 

## Using the shared Foxglove Studio account
A shared account has been created for Foxglove Studio due to restrictions on max users. Log in using the following proton email in the Foxglove client and then follow the link in the confirmation email.

| Email             | Password   |
|-------------------|------------|
| slarc25@proton.me | ?SLaRC2457 |


## Foxglove Bridge for ROS 2
The graphical client Foxglove Studio needs to have the ROS2 node foxglove_bridge node running. If running from the docker container the foxglove_bridge should already be installed, no action needed. Otherwise, if running ROS 2 on Ubuntu then install using

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

## Panel layout
Foxglove Studio is very flexible regarding the panel layout. For waypoint navigation, we recommend having the Click-to-Goal map panel and at least one Image panel with a first person view camera to monitor the robot's progress.

## Click-to-Goal Foxglove Extension

This extension lets you click waypoints (currently only one point) on a map in Foxglove Desktop and
publishes them as ROS 2 messages (e.g., `/goal_geo`) for our robot stack

### Requirements
Node.js and npm is requiered to build the Click-to-Goal extension and are included in the docker container. If you're not using the docker container then install using the following command.

```bash
sudo apt install nodejs npm
```

Recommended versions (used in development):
  - node v18.19.1
  - npm v9.2.0


### Setup (install extension)
Run the `setup_extension.sh` shell script to build and install the panel. 
```bash
cd foxglove_extensions/click-to-goal
./setup_extension.sh
```

Restart Foxglove Desktop afterwards.

### How to use the extension


### Open Foxglove Desktop

Add a new connection using:
ws://localhost:8765

Open the “Click-to-goal” panel (from the extension)

Click on the map to send waypoints as a ros topic 🎯
