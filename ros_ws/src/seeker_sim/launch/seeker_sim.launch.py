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
    # --- Launch arguments ---
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


    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="seeker_sim_config.rviz",
        description="Config filename located under description packages rviz/ directory.",
    )

    use_foxglove = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Start foxglove_bridge",
    )

    spawn_coordinates = DeclareLaunchArgument(
        "Spawn_XYZ_RPY",
        default_value="0.0 0.0 0.0 0.0 0.0 0.0",
        description="The X Y Z coordinates and RPY to spawn the robot at. Sepreated by spaces.\n" \
        "Recommendations:\n" \
        "   sonoma_raceway: 280 -138 7.75 0 0 2.49",
    )

    #280 -138 7.75



    # --- Paths to package resources (world, models, configs) ---
    Robot_description_launch = PathJoinSubstitution([
        FindPackageShare('hs_description'),
        'launch', 'hs_description.launch.py'
    ])



    # Resolve the package share directory and point to the world, model, and config folders.
    world_root = PathJoinSubstitution(
        [
            get_package_share_directory("seeker_sim"),
            "worlds",
            [LaunchConfiguration("world"),".sdf"],
        ]
    )


    navigation_launch_file = PathJoinSubstitution(
        [
            get_package_share_directory("hs_navigation"),
            "launch",
            "hs_navigation.launch.py"
        ]
    )

    config_root = PathJoinSubstitution(
        [get_package_share_directory("seeker_sim"), "config"]
    )

    foxglove_xml_path = os.path.join(
        get_package_share_directory("foxglove_bridge"),
        "launch",
        "foxglove_bridge_launch.xml",
    )



    # RViz2 configuration file (which views/panels to load)
    # make sure the file is located in src/seeker_sim/config folder
    rviz_config_root = PathJoinSubstitution(
        [get_package_share_directory("hs_description"), "rviz", LaunchConfiguration("rviz_config")]
    )


    sdf_roots = [
        os.path.join(get_package_share_directory("hs_description"), "model", "Sensors"),
        os.path.join(get_package_share_directory("hs_description"), "model", "Rigs"),
        os.path.join(get_package_share_directory("hs_description"), "model", "Assemblies"),
        os.path.join(get_package_share_directory("hs_description"), "model", "UGV"),
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


    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_xml_path),
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
    )


    launch_Robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(Robot_description_launch),
        launch_arguments={
            'model':       LaunchConfiguration('model'),
            'use_rviz':   'False',
            'rviz_params': rviz_config_root,
            'use_joint_state_publisher': 'True'
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
    # start_rviz = ExecuteProcess(
    #     cmd=["rviz2", "-d", rviz_config_root],
    #     output="screen",
    # )

    spawn_x = PythonExpression(["'", LaunchConfiguration("Spawn_XYZ_RPY"), "'.split()[0]"])
    spawn_y = PythonExpression(["'", LaunchConfiguration("Spawn_XYZ_RPY"), "'.split()[1]"])
    spawn_z = PythonExpression(["'", LaunchConfiguration("Spawn_XYZ_RPY"), "'.split()[2]"])
    spawn_roll = PythonExpression(["'", LaunchConfiguration("Spawn_XYZ_RPY"), "'.split()[3]"])
    spawn_pitch = PythonExpression(["'", LaunchConfiguration("Spawn_XYZ_RPY"), "'.split()[4]"])
    spawn_yaw = PythonExpression(["'", LaunchConfiguration("Spawn_XYZ_RPY"), "'.split()[5]"])



    spawn_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            LaunchConfiguration("world"), #should be this
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


    # --- ROS 2 nodes ---


    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            #"/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist", #Use this if using teleoptwist_keyboard pkg
            "/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist",
            "/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            "/lidar_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/oakd/rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/oakd/rgbd/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
            "/oakd/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
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

    # Navigation related
    ld.add_action(mapviz_launch_description)
    ld.add_action(navigation_launch_description)
    ld.add_action(spawn_coordinates)

    ld.add_action(launch_Robot_description)
    
    ld.add_action(start_gz)

    ld.add_action(bridge)
    ld.add_action(relay_bridge)

    ld.add_action(delay_spawn)
    

    ld.add_action(use_foxglove)
    ld.add_action(foxglove_bridge_launch)

    return ld

