from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare argument for joystick driver choice
    joy_driver_arg = DeclareLaunchArgument(
        'driver',
        default_value='joy',  
        description='Choose joystick driver: "joy"'
    )

    driver = LaunchConfiguration('driver')



    return LaunchDescription([
        joy_driver_arg,

        # --- Joy driver ---
        Node(
            package='joy',
            executable='joy_node',
            name='joy_driver',
            output='screen',
        ),

        # --- Joy Linux driver ---
        #Node(
        #    package='joy_linux',
        #    executable='joy_linux_node',
        #    name='joy_driver_linux',
        #    output='screen',
        #),

        # Teleop joy
        Node(
            package='teleop_control',
            executable='teleop_joy.py',
            name='teleop_joy',
            output='screen'
        ),

    ])
