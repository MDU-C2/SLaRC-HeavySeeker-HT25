from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import subprocess
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
    ExecuteProcess,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,

)
from ament_index_python.packages import get_package_share_directory
import os, subprocess
from launch.utilities import perform_substitutions
from launch.actions import LogInfo

def convert_model(context,*,model_dir, **kwargs):


    model_dir_str = perform_substitutions(context, [model_dir])
    print(f"[INFO] string {model_dir_str}")

    sdf_file_path   = os.path.join(model_dir_str, "model.sdf")
    model_xacro_file   = os.path.join(model_dir_str, "model.sdf.xacro")

    if not os.path.isfile(model_xacro_file):
        print(f"[WARN] No xacro file found at {model_xacro_file}, skipping conversion.")
        return [SetLaunchConfiguration("model", sdf_file_path)]
    
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


    model_arg = DeclareLaunchArgument(
        "model",
        default_value="Rig5",
        description="name of model FOLDER located under one of the subfolders of this packages model/Assemblies directory.\n",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="seeker_sim_config.rviz",
        description="Config filename located under this packages config/ directory.",
    )

    rviz_use = DeclareLaunchArgument(
        "rviz_use",
        default_value="false",
        description="set to true to launch rviz on startup",
    )

    model_root = PathJoinSubstitution(
        [get_package_share_directory("seeker_sim"), "model","Assemblies", LaunchConfiguration("model")]
    )

    rviz_config_root = PathJoinSubstitution(
        [get_package_share_directory("seeker_sim"), "config", LaunchConfiguration("rviz_config")]
    )

    start_rviz = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config_root],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz_use")),
    )

    ld = LaunchDescription() 

    ld.add_action(model_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(rviz_use)

    ld.add_action(OpaqueFunction(function=convert_model,kwargs={"model_dir": model_root}))

    ld.add_action(OpaqueFunction(function=robot_state_generator))
    ld.add_action(start_rviz)


    return ld