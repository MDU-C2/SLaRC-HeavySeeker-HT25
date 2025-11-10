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

import os, subprocess
from launch.utilities import perform_substitutions
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():
    # --- Launch arguments ---
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="example_2",
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


    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="seeker_sim_config.rviz",
        description="Config filename located under this packages config/ directory.",
    )



    # --- Paths to package resources (world, models, configs) ---
    Robot_description_launch = PathJoinSubstitution([
        FindPackageShare('seeker_sim'),
        'launch', 'description.launch.py'
    ])



    # Resolve the package share directory and point to the world, model, and config folders.
    world_root = PathJoinSubstitution(
        [
            get_package_share_directory("seeker_sim"),
            "worlds",
            [LaunchConfiguration("world"),".sdf"],
        ]
    )



    config_root = PathJoinSubstitution(
        [get_package_share_directory("seeker_sim"), "config"]
    )



    # RViz2 configuration file (which views/panels to load)
    # make sure the file is located in src/seeker_sim/config folder
    rviz_config_root = PathJoinSubstitution(
        [config_root, LaunchConfiguration("rviz_config")]
    )

    
    sdf_roots = [
        os.path.join(get_package_share_directory("seeker_sim"), "model", "Sensors"),
        os.path.join(get_package_share_directory("seeker_sim"), "model", "Rigs"),
        os.path.join(get_package_share_directory("seeker_sim"), "model", "Assemblies"),
        os.path.join(get_package_share_directory("seeker_sim"), "model", "UGV"),
        os.path.join(get_package_share_directory("seeker_sim"), "model", "world_models")    ]

    new_paths = [p for p in sdf_roots if os.path.isdir(p)]

    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    value_parts = new_paths + ([existing] if existing else [])
    merged_value = os.pathsep.join(value_parts)

    # Set GZ_SIM_RESOURCE_PATH to include the model paths

    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=merged_value
    )

    # Log the resolved resource path so we can debug model:// lookups at runtime
    log_gz_path = LogInfo(msg=["GZ_SIM_RESOURCE_PATH=", merged_value])

    # Ensure the gz process inherits the same model/resource path. Pass an env
    # dict to the ExecuteProcess so we are certain the simulator sees these
    # variables (some simulator invocations read GZ_RESOURCE_PATH / GZ_MODEL_PATH).
    start_env = os.environ.copy()
    # update common resource/model env vars used by gz / gazebo
    start_env.update({
        "GZ_SIM_RESOURCE_PATH": merged_value,
        "GZ_RESOURCE_PATH": merged_value,
        "GZ_MODEL_PATH": merged_value,
        "GAZEBO_MODEL_PATH": merged_value,
    })


    launch_Robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(Robot_description_launch),
        launch_arguments={
            'model':       LaunchConfiguration('model'),
            'simulation':  "true",
            'world':       LaunchConfiguration('world'),
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

    # Start RViz2 with the given .rviz configuration.
    start_rviz = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config_root],
        output="screen",
    )




    spawn_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            #"example_2",
            LaunchConfiguration("world"), #should be this
            "-file",
            LaunchConfiguration("model"),
            "-x",
            "100.0",
            "-y",
            "100.0",
            "-z",
            "11.8",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
            "--allow_renaming",
            "0",
        ],
        output="screen",
    )


    # --- ROS 2 nodes ---


    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            "/lidar_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/oakd/rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/oakd/rgbd/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "--ros-args",
            "-r",
            "/lidar_points/points:=/lidar_points",
            "-r",
            "/oakd/rgbd/image:=/oakd_image",
            "-r",
            "/oakd/rgbd/points:=/oakd_points",
        ],
        output="screen",
    )




    # Custom node in the seeker_sim package (executable: cloud_frame_relay).
    # receives point clouds and republishes them under a different frame/namespace.
    relay_bridge = Node(
        package="seeker_sim",
        executable="cloud_frame_relay",
        name="cloud_frame_relay",
    )


    # Delays the start of Rviz to ensure all previous nodes are up and running before.
    delay_start_rviz = TimerAction(
        period=3.0, actions=[start_rviz],
    )


    delay_spawn = TimerAction(
        period=3.0,
        actions=[spawn_cmd],
    )

    # Starting order
    ld = LaunchDescription()

    ld.add_action(world_arg)
    ld.add_action(model_arg)
    ld.add_action(rviz_config_arg)

    ld.add_action(set_gz_path)

    
    ld.add_action(log_gz_path)
    ld.add_action(launch_Robot_description)
    
    ld.add_action(start_gz)
    
    ld.add_action(bridge)
    ld.add_action(relay_bridge)

    ld.add_action(delay_spawn)
    ld.add_action(delay_start_rviz)

    return ld

