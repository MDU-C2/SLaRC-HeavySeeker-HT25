import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition



def generate_launch_description():

    pkg_share = get_package_share_directory("s_bringup")

    config_path = os.path.join(pkg_share, "config", "s_config.yaml")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    config_params = config.get("my_node", {}).get("ros__parameters", {})
    namespace = config_params.get("namespace", "seeker")
    model = config_params.get("model", "Rig5")
    use_JSP = config_params.get("use_JSP", "false")
    use_simtime = config_params.get("use_simtime", "false")
    nav2_config = config_params.get("nav2_config", "")
    navsat_config = config_params.get("navsat_config", "")
    livox_config = config_params.get("livox_config", "")
    ardu_config = config_params.get("ardu_config", "")
    camera_config = config_params.get("camera_config", "")
    camera_serever_config = config_params.get("camera_serever_config", "")

    #-------------------Paths to other launch files-------------------
    perception_dir = PathJoinSubstitution(
        [
            get_package_share_directory("s_perception"),
            "launch", "s_perception.launch.py",]
    )

    cameras_dir = PathJoinSubstitution(
        [
            get_package_share_directory("s_cameras"),
            "launch", "s_cameras.launch.py",]
    )

    fast_lio_dir = PathJoinSubstitution(
        [
            get_package_share_directory("FAST_LIO_SLAM_ros2"),
            "launch", "mapping.launch.py",]
    )    

    navigation_dir = PathJoinSubstitution(
        [
            get_package_share_directory("s_navigation"),
            "launch", "s_navigation.launch.py",]
    )

    description_dir = PathJoinSubstitution(
        [
            get_package_share_directory("s_description"),
            "launch", "s_description.launch.py",]
    )

    teleop_control_dir = PathJoinSubstitution(
        [
            get_package_share_directory("teleop_control"),
            "launch", "teleop_joy.launch.py",]
    )

    robot_dir = PathJoinSubstitution(
        [
            get_package_share_directory("s_robot"),
            "launch", "s_robot.launch.py",]
    )

    #-------------------Arguments------------------

    control_arg = DeclareLaunchArgument(
        "control_mode",
        default_value="autonomous",
        description="Control mode for the robot, e.g., teleop or manual",
        choices=["teleop", "manual", "autonomous"],
    )

    # perception_arg = DeclareLaunchArgument()
    # cameras_arg = DeclareLaunchArgument()
    # fast_lio_arg = DeclareLaunchArgument()
    # navigation_arg = DeclareLaunchArgument()
    # description_arg = DeclareLaunchArgument()
    # teleop_control_arg = DeclareLaunchArgument()
    # robot_arg = DeclareLaunchArgument()

    #-------------------Conditions------------------

    any_condition = IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("control_mode"), "' in ['autonomous', 'manual', 'teleop']"]
        )
    )

    auto_or_teleop_condition = IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("control_mode"), "' in ['autonomous', 'teleop']"]
        )
    )
    
    manual_condition = IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("control_mode"), "' in ['manual']"]
        )
    )

    teleop_condition = IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("control_mode"), "' in ['teleop']"]
        )
    )

    autonomous_condition = IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("control_mode"), "' in ['autonomous']"]
        )
    )
    
    #-------------------Launch Description------------------
    perception_des = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_dir),
        # launch_arguments=perception_arg,
        condition=auto_or_teleop_condition
    )

    cameras_des = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cameras_dir),
        # launch_arguments=cameras_arg,
        condition=auto_or_teleop_condition
    )

    fast_lio_des = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_dir),
        # launch_arguments=fast_lio_arg,
        condition=auto_or_teleop_condition
    )

    navigation_des = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_dir),
        # launch_arguments=navigation_arg,
        condition=autonomous_condition
    )

    description_des = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_dir),
        # launch_arguments=description_arg,
        condition=auto_or_teleop_condition
    )

    teleop_control_des = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_control_dir),
        # launch_arguments=teleop_control_arg,
        condition=any_condition
    )

    robot_des = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_dir),
        # launch_arguments=robot_arg,
        condition=any_condition
    )


    return LaunchDescription([
        control_arg,
        perception_des,
        cameras_des,
        fast_lio_des,
        navigation_des,
        description_des,
        teleop_control_des,
        robot_des,
    ])