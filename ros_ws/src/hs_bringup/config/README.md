To add new lidars, paste
```
{
    "ip" : "192.168.1.0",
    "pcl_data_type" : 1,
    "pattern_mode" : 0,
    "extrinsic_parameter" : {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "x": 0,
    "y": 0,
    "z": 0
    }
}
```

inside ```lidar_configs``` and alter the ip to the new lidar and the extrinsic parameters to where the lidar will be placed in relation to the pose of your origin.