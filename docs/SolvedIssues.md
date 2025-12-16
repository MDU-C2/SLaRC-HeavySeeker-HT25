# MPPIController

* [Description](#description)
* [Solution](#solution)


## Description

First indicator of this issue is when rviz and gazebo load in differently every time with an variation in which things that starts or not.

If you have this issue you can expect to see an error with exit conde -4 connected to the nav2_container. You can also expect that two nodes is missing in the ros2 node list, these nodes is:

* /bt_navigator_navigate_through_poses_rclcpp_node
* /bt_navigator_navigate_to_pose_rclcpp_node

This issue follows from the MPPIController using complex algorithms with instruction that is not supported by the cpu.

## Solution

Replace the MPPIController with another one for example the RegulatedPurePursuitController. This is done in your params.yaml file in the nav2_bringup package this file can be found under [nav2_bringup_dir]/params/nav2_params.yaml, the actual change is shown below.


From this:
```yaml
FollowPath:
    plugin: "nav2_mppi_controller::MPPIController"
```
To this:
```yaml
FollowPath:
    plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
```

# Troubleshooting LiDAR Discovery

## Description
If trouble with discovery of LiDAR. Firewall can be the issue. The Viewer manual suggests disabling the firewall entirely but a workaround can be done.

## Solution

#### 1. First see if the firewall is the issue:
```bash
    sudo ufw status
    sudo ufw disable
```

#### 2. If firewall is the issue find your ethernet/USB-Ethernet interface name (it’s usually enx…):
```bash
    ip -br linkip -br link
```

#### 3. Allow Livox discovery + traffic on the LiDAR LAN only:
```bash
    IFACE=<your-ethernet-iface>   # e.g., IFACE=enx00e04c680001
```
```bash
    sudo ufw allow in  on $IFACE from 192.168.10.0/24
    sudo ufw allow out on $IFACE to   192.168.10.0/24

    sudo ufw allow in  on $IFACE to any port 56000 proto udp
    sudo ufw allow in  on $IFACE to any port 55000 proto udp
    sudo ufw allow in  on $IFACE to any port 65000 proto udp
```

#### 4. Re-enable firewall:
```bash
    sudo ufw enable
    sudo ufw status verbose
```