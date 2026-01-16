## 1. Setup
First: Make sure "lid_topic:" and "imu_topic:" is correct in config/mid360.yaml.

Second: Follow the main README.md to setup Livox-SDK2 and colcon build.

> Use '''$ ros2 launch s_perception livox_launch.py''' if you are using the physical LiDAR.

## 2. Launch 3D-SLAM with LiDAR 
```bash
    ros2 launch fast_lio mapping.launch.py rviz:=true > /dev/null 2>&1
```

## 2.1 Launch 3D-SLAM with Rosbag
```bash
    ros2 launch fast_lio mapping.launch.py rviz:=true > /dev/null 2>&1
    ros2 bag play <your_bag_dir>
```
> [!NOTE]
> (> /dev/null 2>&1) is just to silence warnings.
> (rviz:=true) default is false.
## 3. PCD (3D-map) file Save
##### Launch step (2 or 2.1) and inside Docker in another Terminal:
```bash
ros2 run pcl_ros pointcloud_to_pcd --ros-args \
  -r input:=/Laser_map -p binary:=true \
  -p prefix:=/slarc_ws/src/FAST_LIO_SLAM_ros2/PCD/map_
```

##### Pcl_ros saves every message as a new file, with each new file building on the last. So to save only the last/recent file run this command inside Docker in another Terminal:
```bash
watch -n 2 'ls -1t /slarc_ws/src/FAST_LIO_SLAM_ros2/PCD/map_*.pcd 2>/dev/null | tail -n +2 | xargs -r rm -f'
```

## 4. PCD (3D-map) file Load
```bash
pcl_viewer /slarc_ws/src/FAST_LIO_SLAM_ros2/PCD/<your_PCD-file_name>
```

##### *Tips for pcl_viewer:*
- Change what to visualize/color by pressing keyboard 1,2,3,4,5 when pcl_viewer is running. 
```
    1 is all random
    2 is X values
    3 is Y values
    4 is Z values
    5 is intensity
```


