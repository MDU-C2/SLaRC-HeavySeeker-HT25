import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    #Not finished yet
    perception_dir = PathJoinSubstitution(
        [
            get_package_share_directory("s_perception"),
            "launch", "s_perception.launch.py",]
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

    control_dir = PathJoinSubstitution(
        [
            get_package_share_directory("s_control"),
            "launch", "s_control.launch.py",]
    )

    robot_dir = PathJoinSubstitution(
        [
            get_package_share_directory("s_robot"),
            "launch", "s_robot.launch.py",]
    )

    #-------------------Arguments------------------

    control_arg = DeclareLaunchArgument(
        "control_mode",
        default_value="teleop",
        description="Control mode for the robot, e.g., teleop or manual",
    )

    #-------------------Conditions------------------
    auto_or_manual_condition = IfCondition(
    PythonExpression(
        ["'", LaunchConfiguration("control_mode"), "' in ['autonomous', 'manual']"]
    )
)
    
    manual_condition = IfCondition(
    PythonExpression(
        ["'", LaunchConfiguration("control_mode"), "' in ['manual']"]
    )
    )

    #-------------------Processes------------------

    rviz_proc = ExecuteProcess(
    cmd=["rviz2"],
    output="screen",
    condition=manual_condition,
)

    passed_log = LogInfo(
        msg=["passed"],
        condition=auto_or_manual_condition,
    )
    
    #-------------------Launch Description------------------
    ld = LaunchDescription()
    ld.add_action(control_arg)
    ld.add_action(rviz_proc)
    ld.add_action(passed_log)

    return LaunchDescription([ld])
