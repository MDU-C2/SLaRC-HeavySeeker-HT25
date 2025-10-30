#!/usr/bin/env python3
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
# @author Vladislav Saburov (slarc25@proton.me)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    bringup_dir = get_package_share_directory("hs_bringup")
    camera_dir = get_package_share_directory("hs_cameras")
    namespace = LaunchConfiguration("namespace")

    ARGUMENTS = [
        DeclareLaunchArgument(
            "namespace", default_value="", description="Robot namespace"
        )
    ]
    cameras_launch_file = PathJoinSubstitution([camera_dir, "launch", "cameras.launch.py"])
    oakd_launch_file = PathJoinSubstitution([bringup_dir, "launch", "oakd.launch.py"])

    septentrio_launch_file = PathJoinSubstitution(
        [bringup_dir, "launch", "rover_node.launch.py"]
    )

    actions = [
        PushRosNamespace(namespace),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([cameras_launch_file]),
        #    launch_arguments=[("namespace", namespace)],
        #),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([oakd_launch_file]),
            launch_arguments=[("namespace", namespace)],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([septentrio_launch_file]), launch_arguments=[]
        ),
    ]

    hs = GroupAction(actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(hs)
    return ld
