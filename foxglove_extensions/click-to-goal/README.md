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

The folder foxglove_extensions lies in the SLaRC-HeavySeeker-HT25 folder.

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
