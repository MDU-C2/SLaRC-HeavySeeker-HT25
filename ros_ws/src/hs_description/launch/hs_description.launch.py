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

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = get_package_share_directory('hs_description')
    xacro_file = PathJoinSubstitution(
        [description_dir, 'sdf', 'hs.sdf.xacro'])

    ARGUMENTS = [DeclareLaunchArgument(
        'namespace', default_value='hs', description='Robot namespace')]

    namespace = LaunchConfiguration('namespace')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(
            ['xacro ', ' ', xacro_file, ' ', 'namespace:=', namespace])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(
            ['xacro ', ' ', xacro_file, ' ', 'namespace:=', namespace])}]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    return ld
