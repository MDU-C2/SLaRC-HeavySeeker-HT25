from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, PushROSNamespace
from launch_ros.descriptions import ComposableNode

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_dir = get_package_share_directory('hs_bringup')

    oakd_params = LaunchConfiguration('oakd_params')
    namespace = LaunchConfiguration('namespace')

    ARGUMENTS = [
        DeclareLaunchArgument('oakd_params', default_value=[PathJoinSubstitution(
            [bringup_dir, 'config', 'oakd_pro']), '.yaml'], description='Parameters for Depthai ROS driver'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace')
    ]

    namespaced_params = RewrittenYaml(
        source_file=oakd_params,
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )

    node = ComposableNodeContainer(
        name='oakd_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depthai_ros_driver',
                plugin='depthai_ros_driver::Camera',
                name='oakd',
                parameters=[namespaced_params]
            ),
        ],
        output='screen',
    )

    actions = [PushROSNamespace(namespace), node]
    oakd = GroupAction(actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(oakd)
    return ld
