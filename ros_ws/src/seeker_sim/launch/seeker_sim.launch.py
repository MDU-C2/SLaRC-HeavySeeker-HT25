from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import subprocess
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os, subprocess
from seeker_sim.convert_xacro_to_sdf import convert_xacro_to_sdf
from launch.utilities import perform_substitutions




def convert_model(context,*,model_dir, **kwargs):


    model_dir_str = perform_substitutions(context, [model_dir])
    print(f"[INFO] string {model_dir_str}")

    sdf_file_path   = os.path.join(model_dir_str, "model.sdf")
    model_xacro_file   = os.path.join(model_dir_str, "model.sdf.xacro")
    print(f"[INFO] Converting {model_xacro_file} → {sdf_file_path}")

    # Run xacro directly on the SDF.xacro and write result to model.sdf
    # This is basically: xacro model.sdf.xacro -o model.sdf
    with open(sdf_file_path, "w") as sdf_out:
        subprocess.run(
            ["xacro", model_xacro_file],
            check=True,
            stdout=sdf_out
        )

    print(f"[INFO] Done. Wrote {sdf_file_path}")
    return [SetLaunchConfiguration("model", sdf_file_path)]



def robot_state_generator(context, *args, **kwargs):
    with open(LaunchConfiguration("model").perform(context), 'r') as infp:
            robot_desc = infp.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(robot_desc, value_type=str),
        }],
        output='screen',
    )
    return [robot_state_pub]
    


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
        default_value="Rig5",
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
        default_value="seeker_sim_oakd_lidar.rviz",
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

    model_root = PathJoinSubstitution(
        [get_package_share_directory("seeker_sim"), "model","Assemblies", LaunchConfiguration("model")]
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
            LaunchConfiguration("model"),
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
    simple_bridge = Node(
        package="seeker_sim",
        executable="cloud_frame_relay",
        name="cloud_frame_relay",
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

    ld.add_action(bridge)

    ld.add_action(OpaqueFunction(function=convert_model,kwargs={"model_dir": model_root}))
    ld.add_action(OpaqueFunction(function=robot_state_generator))


    ld.add_action(delay_spawn)
    ld.add_action(delay_start_rviz)
    ld.add_action(delay_start_rviz_path)

    return LaunchDescription([ld])
