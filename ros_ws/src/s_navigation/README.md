# Seeker navigation package

This package integrates `nav2`-based navigation stack with **seeker** UGV

## Quick Start
`ros2 launch s_navigation s_navigation.launch.py` - starts `nav2` and `robot_localization` with default configs from `s_navigation/config` directory, begins listening to `/clicked_point` to collect waypoints, activates on `start` command on `/waypoint_command`.

## Launch
### [`s_navigation.launch.py`](./launch/s_navigation.launch.py)
Launch **seeker**'s navigation stack. The stack includes `nav2_bringup`, `robot_localization` and custom nodes from this package.

Args:

- `nav2_config: relative_path` - file in `config` directory with configuration for `nav2`. Default: `"nav2_params.yaml"`
- `navsat_config: relative_path` - file in `config` directory with nodes' configuration. Default: `"dual_ekf_navsat.yaml"`
- `use_sim_time: [True, False]` - use clock source from simulation (topic named `/clock`). Default: `False`
- `namespace: str` - Robot namespace. Default: `""`

### [`s_navsat.launch.py`](./launch/s_navsat.launch.py)
Launch `robot_localization` nodes. Creates `map` -> `odom` -> `base_link` transform.

Args:

- `namespace: str` - Robot namespace. Default: `""`
- `use_sim_time: [True, False]` - use clock source from simulation (topic named `/clock`). Default: `False`
- `navsat_config_arg: relative_path` - file in `config` directory with nodes' configuration. Default: `"dual_ekf_navsat.yaml"`

## Configuration files
- [`dual_ekf_navsat.yaml`](./config/dual_ekf_navsat.yaml) - default configuration for `robot_localization` with local (`odom` -> `base_link`) and global (`map` -> `odom`) ekf nodes and `navsatr_transform` node to process [`gps_msgs/GPSFix`](https://docs.ros.org/en/jazzy/p/gps_msgs/msg/GPSFix.html) messages from GNSS 
- [`nav2_params.yaml`](./config/nav2_params.yaml) - default configuration for `nav2_bringup` for a differential drive robot
- [`nav2_params_sim.yaml`](./config/nav2_params_sim.yaml) - configuration for `nav2_bringup` for a differential drive robot in simulation
- [`simulation_ekf.yaml`](./config/simulation_ekf.yaml) - configuration for `robot_localization` with local (`odom` -> `base_link`) and global (`map` -> `odom`) ekf nodes and `navsatr_transform` node to process [`gps_msgs/GPSFix`](https://docs.ros.org/en/jazzy/p/gps_msgs/msg/GPSFix.html) messages from GNSS in simulation

## Nodes
### [`waypoint_command_node.py`](./s_navigation/waypoint_command_node.py)
This node converts [`geometry_msgs/PointStamped`](https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/PointStamped.html) in _wgs84_ coordinates and redirects it to `nav2`

Subscribtions:

- `/clicked_point`:[`geometry_msgs/PointStamped`](https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/PointStamped.html) - waypoint in _wgs84_ coordinates

Publishers:

- `/activate_autonomous_drive`:[`std_msgs/Bool`](https://docs.ros.org/en/jazzy/p/std_msgs/msg/Bool.html)- starts or stop autonomous navigation
- `/waypoint_progress`:[`s_msgs/WaypointProgress`](../s_msgs/msg/WaypointProgress.msg) - reports current navigation status

Services:
- `/waypoint_command`:[`s_msgs/WaypointCommand`](../s_msgs/msg/WaypointCommandMsgs.msg) - controls waypoint navigation with commands

## Resources
### Docs
- [`robot_localization`](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [`nav2`](https://docs.nav2.org/index.html)
### Useful links
- [`robot_localization` sensor setup](http://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html)
- [`Nav2` concepts](https://docs.nav2.org/concepts/index.html)
- [`Nav2` First-Time Robot Setup Guide](https://docs.nav2.org/setup_guides/index.html)
- [Overview of `nav2`'s plugins](https://docs.nav2.org/plugins/index.html)
