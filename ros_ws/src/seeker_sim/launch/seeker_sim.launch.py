from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    LogInfo,
    TimerAction,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import (
    EnvironmentVariable,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
    TextSubstitution,
)
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os
from seeker_sim.convert_xacro_to_sdf import convert_xacro_to_sdf


def prepare_model(context, *args, **kwargs):
    model_name = LaunchConfiguration("model").perform(context)
    sdf_path = convert_xacro_to_sdf(model_name)
    print(f"[INFO] Using generated SDF: {sdf_path}")
    return []


def generate_launch_description():
    # --- Launch arguments ---
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="example_2.sdf",
        description="World filename located under this packages worlds/ directory.",
    )

    # Enabels fullpath description to world
    world_path_arg = DeclareLaunchArgument(
        "world_path",
        default_value="",
        description="Absolute path to a .sdf world. If set (non-empty), overrides the world filename.",
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="Rig4",
        description="Model FOLDER located under one of the subfolders of this packages model/ directory.\n"
        "   OBS the model files should always be named model.sdf and have a model.config\n"
        "   The folder should also contain a materials and meshes folder even if empty\n"
        "   See src/model/UGV/Husky as an example.",
    )

    # Enabels fullpath description to model
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Absolute path to a model folder If set (non-empty), overrides the world filename."
        "\n     The folder should follow the structure listed above",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="seeker_sim_config_points.rviz",
        description="Config filename located under this packages config/ directory.",
    )

    # Enabels fullpath description to rviz config file
    rviz_config_path_arg = DeclareLaunchArgument(
        "rviz_config_path",
        default_value="",
        description="Absolute path to a .rviz file If set (non-empty), overrides the world filename.",
    )

    # --- Paths to package resources (world, models, configs) ---
    # Resolve the package share directory and point to the world, model, and config folders.
    world_root = PathJoinSubstitution(
        [
            get_package_share_directory("seeker_sim"),
            "worlds",
            LaunchConfiguration("world"),
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
    ]

    new_paths = [p for p in sdf_roots if os.path.isdir(p)]

    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    value_parts = new_paths + ([existing] if existing else [])
    merged_value = os.pathsep.join(value_parts)

    # Set GZ_SIM_RESOURCE_PATH to include the model paths

    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=merged_value
    )

    # --- Check if absolute path argument was passed

    have_full_world_path = IfCondition(
        PythonExpression(['"', LaunchConfiguration("world_path"), '" != ""'])
    )
    no_full_world_path = UnlessCondition(
        PythonExpression(['"', LaunchConfiguration("world_path"), '" != ""'])
    )

    have_full_model_path = IfCondition(
        PythonExpression(['"', LaunchConfiguration("model_path"), '" != ""'])
    )
    no_full_model_path = UnlessCondition(
        PythonExpression(['"', LaunchConfiguration("model_path"), '" != ""'])
    )

    have_full_rviz_config_path = IfCondition(
        PythonExpression(['"', LaunchConfiguration("rviz_config_path"), '" != ""'])
    )
    no_full_rviz_config_path = UnlessCondition(
        PythonExpression(['"', LaunchConfiguration("rviz_config_path"), '" != ""'])
    )

    # --- Processes launched via shell commands (not ROS 2 nodes) ---

    # Start Gazebo Harmonic (gz sim) with the selected world.
    # Note: Here '-r' is used to start the simulation; behavior can vary between tools.
    start_gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_root],
        #cmd=["gz", "sim", world_root],
        output="screen",
        condition=no_full_world_path,
    )

    start_gz_absolute_path = ExecuteProcess(
        cmd=["gz", "sim", "-r", LaunchConfiguration("world_path")],
        output="screen",
        condition=have_full_world_path,
    )

    # Start RViz2 with the given .rviz configuration.
    start_rviz = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config_root],
        output="screen",
        condition=no_full_rviz_config_path,
    )

    start_rviz_absolute_path = ExecuteProcess(
        cmd=["rviz2", "-d", LaunchConfiguration("rviz_config_path")],
        output="screen",
        condition=have_full_rviz_config_path,
    )

    model_uri = PythonExpression(["'model://' + '", LaunchConfiguration("model"), "'"])

    spawn_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            "example_2",
            "-file",
            model_uri,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.2",
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

    # Open a separate terminal window with teleop_twist_keyboard so you can drive the robot.
    # Remap cmd_vel to Gazebos Husky command topic.
    teleop_cmd = (
        "source /opt/ros/jazzy/setup.bash; "
        "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
        "--ros-args -r cmd_vel:=/model/husky/cmd_vel; "
        "exec bash"  # Keeps the terminal window open after node termination
    )

    teleop = ExecuteProcess(
        #cmd=["gnome-terminal", "--", "bash", "-lc", teleop_cmd], output="screen"
        cmd=[teleop_cmd], output="screen"
    )

    # --- ROS 2 nodes ---

    # ros_gz_bridge::parameter_bridge
    # Bridges messages between Gazebo (GZ Transport) and ROS 2 types:
    #  - Husky velocity command (Twist <-> gz.msgs.Twist)
    #  - Odometry (Odometry <-> gz.msgs.Odometry)
    #  - /clock (simulated time to ROS 2)
    #  - Lidar point cloud (PointCloud2 <-> gz.msgs.PointCloudPacked)
    # Also remaps /lidar/mid360/points/points -> /lidar/mid360/points.

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/husky/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/model/husky/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            "/lidar/mid360/points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "--ros-args",
            "-r",
            "/lidar/mid360/points/points:=/lidar/mid360/points",
        ],
        output="screen",
    )

    # Custom node in the seeker_sim package (executable: cloud_frame_relay).
    # receives point clouds and republishes them under a different frame/namespace.
    simple_bridge = Node(
        package="seeker_sim",
        namespace="sim1",
        executable="cloud_frame_relay",
        name="sim",
    )

    # Publish a static transform between base_link and lidar.
    # Args: x y z roll pitch yaw parent child
    # Places the lidar 0.716 m above base_link with zero rotation.
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.716", "0", "0", "0", "base_link", "lidar"],
        output="screen",
    )

    # Delays the start of Rviz to ensure all previous nodes are up and running before.
    delay_start_rviz = TimerAction(
        period=3.0, actions=[start_rviz], condition=no_full_rviz_config_path
    )

    delay_start_rviz_path = TimerAction(
        period=3.0,
        actions=[start_rviz_absolute_path],
        condition=have_full_rviz_config_path,
    )

    delay_spawn = TimerAction(
        period=3.0,
        actions=[spawn_cmd],
    )

    # Starting order
    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(world_path_arg)

    ld.add_action(model_arg)
    ld.add_action(model_path_arg)

    ld.add_action(rviz_config_arg)
    ld.add_action(rviz_config_path_arg)

    ld.add_action(set_gz_path)
    ld.add_action(simple_bridge)

    ld.add_action(start_gz)
    # ld.add_action(start_gz_absolute_path)

    ld.add_action(lidar_tf)
    ld.add_action(bridge)

    ld.add_action(OpaqueFunction(function=prepare_model))

    #ld.add_action(teleop)
    ld.add_action(delay_spawn)
    ld.add_action(delay_start_rviz)
    ld.add_action(delay_start_rviz_path)

    return LaunchDescription([ld])
