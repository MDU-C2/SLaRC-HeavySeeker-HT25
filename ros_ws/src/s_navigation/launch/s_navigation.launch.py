
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    LogInfo,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # slam_dir = get_package_share_directory("slam_toolbox")
    s_nav_dir = get_package_share_directory("s_navigation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    config_dir = PathJoinSubstitution(
        [
            get_package_share_directory("s_navigation"),
            "config"]
    )

    nav2_config_arg = DeclareLaunchArgument(
        "nav2_config",
        default_value="nav2_params.yaml",
        description="Name of config file for Nav2, located the the config folder this package",
    )

    navsat_config_arg = DeclareLaunchArgument(
        "navsat_config",
        default_value="dual_ekf_navsat.yaml",
        description="Name of config file for Navsat, located the the config folder this package",
    )

    use_simtime_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true, should be used for simulation only",
        choices=["True", "False"],
    )

    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
        )

    # nav2_config= PathJoinSubstitution(
    #     [
    #         get_package_share_directory("nav2_bringup"),
    #         "config",
    #         LaunchConfiguration("nav2_config"),]

    # Robot localization node using world and map ekf
    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([s_nav_dir, "launch", "s_navsat.launch.py"])),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration('use_sim_time')),
            ("navsat_config_arg", LaunchConfiguration('navsat_config'))
        ],
    )

    # slam_toolbox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([slam_dir, "launch", "online_async_launch.py"])),
    #     launch_arguments=[
    #         ("use_sim_time", LaunchConfiguration('use_sim_time')),
    #         ("namespace", LaunchConfiguration('namespace')),
    #     ],
    # )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_bringup_dir, "launch", "navigation_launch.py"])),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration('use_sim_time')),
            ("namespace", LaunchConfiguration('namespace')),
            ("params_file", PathJoinSubstitution([config_dir, LaunchConfiguration('nav2_config')])), # nav2 wants abs path
        ],
    )


    waypoint_command_node = Node(
        package="s_navigation",
        executable="waypoint_command_node.py",
        name="waypoint_command",
        output="screen",
        parameters=[],
        remappings=[
            ("waypoints", "waypoints"),
            ("waypoint_status", "waypoint_status"),
        ],
    )


    actions = [
        #PushROSNamespace(namespace), what is this?
        robot_localization_launch,
        waypoint_command_node,
        #TimerAction(period=5.0, actions=[slam_toolbox_launch]),
        TimerAction(period=10.0, actions=[nav2_bringup_launch]),
        LogInfo(msg=["s_navigation_launch: Launching with nav2_config: ", PathJoinSubstitution([config_dir, LaunchConfiguration('nav2_config')])]),
    ]
    hs = GroupAction(actions)

    ld = LaunchDescription()
    ld.add_action(nav2_config_arg)
    ld.add_action(navsat_config_arg)
    ld.add_action(use_simtime_arg)
    ld.add_action(namespace)

    ld.add_action(hs)

    return ld
