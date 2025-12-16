# Copyright 2021 Clearpath Robotics, Inc.
# Modifications copyright 2025 Swedish Land-based Robotics Centre
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)
# @author Mattias Tidström (slarc25@proton.me)

import os
import subprocess

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction, GroupAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch.utilities import perform_substitutions
from launch.conditions import IfCondition


def convert_model(context, *, model_dir, **kwargs):
    model_dir_str = perform_substitutions(context, [model_dir])
    print(f'[INFO] string {model_dir_str}')

    sdf_file_path = PathJoinSubstitution(
        [model_dir_str, 'model.sdf']).perform(context=context)
    model_xacro_file = PathJoinSubstitution(
        [model_dir_str, 'model.sdf.xacro']).perform(context=context)

    if not os.path.isfile(model_xacro_file):
        print(
            f'[WARN] No xacro file found at {model_xacro_file}, skipping conversion.')
        return [SetLaunchConfiguration("model", sdf_file_path)]

    print(f"[INFO] Converting {model_xacro_file} → {sdf_file_path}")
    # Run xacro directly on the SDF.xacro and write result to model.sdf
    # This is basically: xacro model.sdf.xacro -o model.sdf
    with open(sdf_file_path, 'w') as sdf_out:
        subprocess.run(
            ['xacro', model_xacro_file],
            check=True,
            stdout=sdf_out
        )

    print(f'[INFO] Done. Wrote {sdf_file_path}')
    return [SetLaunchConfiguration('model', sdf_file_path)]


def robot_state_generator(context, *args, **kwargs):
    with open(LaunchConfiguration('model').perform(context), 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(robot_desc, value_type=str),
        }],
        output='screen',
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(robot_desc, value_type=str)}],
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher'))
    )

    return [robot_state_publisher_node, joint_state_publisher_node]


def generate_launch_description():
    description_dir = get_package_share_directory('s_description')

    ARGUMENTS = [DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
                 DeclareLaunchArgument(
                     'model', default_value='Rig7', description='Name of model FOLDER located under one of the subfolders of this packages model/Assemblies directory'),
                 DeclareLaunchArgument('use_sim_time', default_value='False',
                                       description='Use clock from simulation', choices=['True', 'False']),
                 DeclareLaunchArgument('use_joint_state_publisher', default_value='False', description='Run joint state publisher', choices=['True', 'False'])]

    namespace = LaunchConfiguration('namespace')

    model_root = PathJoinSubstitution(
        [description_dir, 'model', 'Assemblies', LaunchConfiguration('model')]
    )

    generate_model = OpaqueFunction(
        function=convert_model,
        kwargs={'model_dir': model_root}
    )

    generate_description = OpaqueFunction(
        function=robot_state_generator
    )


    actions = [
        PushROSNamespace(namespace=namespace),
        generate_model,
        generate_description
    ]

    hs = GroupAction(actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(hs)
    return ld
