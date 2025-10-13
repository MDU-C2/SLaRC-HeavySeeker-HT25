from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    bringup_dir = get_package_share_directory('hs_bringup')

    namespace = LaunchConfiguration('namespace')

    ARGUMENTS = [DeclareLaunchArgument(
        'namespace', default_value='', description='Robot namespace')]

    oakd_launch_file = PathJoinSubstitution(
        [bringup_dir, 'launch', 'oakd.launch.py'])

    actions = [
        PushRosNamespace(namespace),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [oakd_launch_file]), launch_arguments=[('namespace', namespace)])
    ]

    hs = GroupAction(actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(hs)
    return ld
