from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from ament_index_python.packages import get_package_share_directory

import os
from launch.utilities import perform_substitutions
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression




def generate_launch_description():
    namespace = "s_sim"

    # ---------------------- Launch arguments ------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="sonoma_raceway",
        description="World filename located under this packages worlds/ directory.",
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="Rig5",
        description="Model FOLDER located under one of the subfolders of this packages model/ directory.\n"
        "   OBS the model files should always be named model.sdf and have a model.config\n"
        "   The folder should also contain a materials and meshes folder even if empty\n"
        "   See src/model/UGV/Husky as an example.",
    )

    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value="false",
        description="Start foxglove_bridge. If set to false rviz and mapviz will be used instead.",
    )

    spawn_coordinates_arg = DeclareLaunchArgument(
        "Spawn",
        default_value="0.0 0.0 2.0 0.0 0.0 0.0",
        description="The XYZ coordinates and RPY to spawn the robot at. Sepreated by spaces.\n" \
    )


    # ------------------ Paths to package resources (world, models, configs) -------------------

    Robot_description_launch = PathJoinSubstitution([
        FindPackageShare('s_description'),
        'launch', 's_description.launch.py'
    ])

    # Resolve the package share directory and point to the world, model, and config folders.
    world_root = PathJoinSubstitution(
        [
            get_package_share_directory("s_simulation"),
            "worlds",
            [LaunchConfiguration("world"),".sdf"],
        ]
    )

    config_root = PathJoinSubstitution(
        [get_package_share_directory("s_simulation"), "config"]
    )

    foxglove_xml_root = os.path.join(
        get_package_share_directory("foxglove_bridge"),
        "launch",
        "foxglove_bridge_launch.xml",
    )

    rviz_config_root = PathJoinSubstitution(
        [get_package_share_directory("s_simulation"), "rviz", "simulation_demo.rviz"]
    )

    navigation_launch_root = PathJoinSubstitution(
        [
            get_package_share_directory("hs_navigation"),
            'launch',
            'hs_navigation.launch.py',
        ]
        )

    scan_launch_root = PathJoinSubstitution(
            [
                get_package_share_directory("s_perception"), #Fix this!!!
                'launch',
                'cloud2scan.launch.py',
            ]
        )


    # Set GZ_SIM_RESOURCE_PATH to include the model paths

    sdf_roots = [
        os.path.join(get_package_share_directory("s_description"), "model", "Sensors"),
        os.path.join(get_package_share_directory("s_description"), "model", "Rigs"),
        os.path.join(get_package_share_directory("s_description"), "model", "Assemblies"),
        os.path.join(get_package_share_directory("s_description"), "model", "UGV"),
        os.path.join(get_package_share_directory("s_simulation"), "model", "world_models")    ]

    new_paths = [p for p in sdf_roots if os.path.isdir(p)]

    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    value_parts = new_paths + ([existing] if existing else [])
    merged_value = os.pathsep.join(value_parts)

    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=merged_value
    )

    # Ensure the gz process inherits the same model/resource path. Pass an env
    start_env = os.environ.copy()

    # update common resource/model env vars used by gz / gazebo
    start_env.update({
        "GZ_SIM_RESOURCE_PATH": merged_value,
        "GZ_RESOURCE_PATH": merged_value,
        "GZ_MODEL_PATH": merged_value,
        "GAZEBO_MODEL_PATH": merged_value,
    })

    #---------------------- Launch descriptions ------------------------------


    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_xml_root),
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
    )

    model = LaunchConfiguration('model')

    launch_Robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(Robot_description_launch),
        launch_arguments={
            'namespace':   namespace,
            'model':       model,
            'use_sim_time': 'True',
            'use_joint_state_publisher': 'True'
        }.items()
    )

    navigation_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_root),
        launch_arguments={
            'use_map':   'True',
            'rviz_config': rviz_config_root,
            'use_sim_time': 'True'
        }.items()
    )

    scan_converter_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(scan_launch_root),
        launch_arguments={
            'cloud_topic':  '/lidar_points_fixed',
            'target_frame':   'mid360_lidar_link',
        }.items()
    )





    # --- Processes launched via shell commands (not ROS 2 nodes) ---

    # Start Gazebo Harmonic (gz sim) with the selected world.
    # Note: Here '-r' is used to start the simulation; behavior can vary between tools.
    start_gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_root],
        #cmd=["gz", "sim", world_root],
        output="screen",
        env=start_env,
    )


    spawn_x = PythonExpression(["'", LaunchConfiguration("Spawn"), "'.split()[0]"])
    spawn_y = PythonExpression(["'", LaunchConfiguration("Spawn"), "'.split()[1]"])
    spawn_z = PythonExpression(["'", LaunchConfiguration("Spawn"), "'.split()[2]"])
    spawn_roll = PythonExpression(["'", LaunchConfiguration("Spawn"), "'.split()[3]"])
    spawn_pitch = PythonExpression(["'", LaunchConfiguration("Spawn"), "'.split()[4]"])
    spawn_yaw = PythonExpression(["'", LaunchConfiguration("Spawn"), "'.split()[5]"])



    spawn_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            LaunchConfiguration("world"), 
            "-file",
            LaunchConfiguration("model"),
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-R",
            spawn_roll,
            "-P",
            spawn_pitch,
            "-Y",
            spawn_yaw,
            "--allow_renaming",
            "0",
        ],
        output="screen",
    )


    # --------------------------------- ROS 2 nodes --------------------------------


    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist",
            "/odometry/wheel@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/world/sonoma_raceway/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            "/lidar_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/oakd/rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/oakd/rgbd/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
            "/oakd/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/livox/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "--ros-args",
            "-r",
            "/lidar_points/points:=/lidar_points",
            "-r",
            "/oakd/rgbd/image:=/oakd_image",
            "-r",
            "/oakd/rgbd/points:=/oakd_points",
            "-r",
            "/world/sonoma_raceway/clock:=/clock"
        ],
        output="screen",
    )

    # Attach gnss plugin frame to the rig
    gnss_to_rig_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # x  y  z   roll pitch yaw   parent_frame   child_frame
        arguments=["0.0","0.0","0.0","0.0","0.0","0.0","gnss_link", [model, "/gnss_link/gnss"]],
        output="screen",
        name="gnss_to_rig_static_tf"
    )

    # Custom node in the seeker_sim package (executable: cloud_frame_relay).
    # receives point clouds and republishes them as scan data over a different topic.
    relay_bridge = Node(
        package="s_simulation",
        executable="cloud_frame_relay.py",
        name="cloud_frame_relay",
    )





    delay_spawn = TimerAction(
        period=3.0,
        actions=[spawn_cmd],
    )

    # Starting order
    ld = LaunchDescription()

    ld.add_action(world_arg)
    ld.add_action(model_arg)
    ld.add_action(use_foxglove_arg)
    ld.add_action(spawn_coordinates_arg)

    ld.add_action(set_gz_path)
    ld.add_action(start_gz)
    ld.add_action(launch_Robot_description)
    ld.add_action(delay_spawn)
    ld.add_action(gnss_to_rig_tf)
    ld.add_action(bridge)
    ld.add_action(relay_bridge)
    ld.add_action(scan_converter_launch_description)
    ld.add_action(navigation_launch_description)
    ld.add_action(foxglove_bridge_launch)

    return ld
