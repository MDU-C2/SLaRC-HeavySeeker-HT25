#from distro import name
from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, LogInfo, TimerAction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart


def generate_launch_description():


    # ─── Launch Arguments ───────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='example.sdf',
        description='World file to load (from seeker_sim/worlds/)'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='/Husky/model.sdf', #husky
        description='Model name to spawn in the world'
    )

    world_name_arg = DeclareLaunchArgument(
    'world_name', default_value='example',
    description='Name of the world inside the SDF (often "default")'
    )
    name_arg = DeclareLaunchArgument('name', default_value='husky', description='Spawned model name')

    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.2')
    roll_arg  = DeclareLaunchArgument('roll',  default_value='0.0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0')
    yaw_arg   = DeclareLaunchArgument('yaw',   default_value='0.0')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    world_name = LaunchConfiguration('world_name')
    name = LaunchConfiguration('name')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    roll, pitch, yaw = LaunchConfiguration('roll'), LaunchConfiguration('pitch'), LaunchConfiguration('yaw')


    # ─── Paths ──────────────────────────────────────────────────────────    
    world_root = PathJoinSubstitution([
        get_package_share_directory('seeker_sim'),
        'worlds', world
    ])

    model_root = PathJoinSubstitution([
        get_package_share_directory('seeker_sim'), 'model'
    ])

    config_root = PathJoinSubstitution([
        get_package_share_directory('seeker_sim'), 'config'
    ])

    rviz_config_root = PathJoinSubstitution([
        config_root, 'seeker_sim_config.rviz'
    ])

    model_file = PathJoinSubstitution([
    get_package_share_directory('seeker_sim'), 'model', model  # model is your existing LaunchConfiguration('model')
    ])

    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ':', model_root
        ]
    )

    # ─── Nodes and Processes ────────────────────────────────────────────
    start_gz = ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_root],   # t.ex. 'gz sim my_world.sdf'
            output='screen'
        )
    
    start_rviz = ExecuteProcess(
            cmd=['rviz2','-d', rviz_config_root], 
            output='screen',
            
        )
    
    spawn_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', world_name,
            '-file', model_file,
            '-name', name,
            '-x', x, '-y', y, '-z', z,
            '-R', roll, '-P', pitch, '-Y', yaw,
            '--allow_renaming', '0'
        ],
        output='screen'
    )
    
    teleop_cmd = (
        'source /opt/ros/jazzy/setup.bash; '
        'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
        '--ros-args -r cmd_vel:=/model/husky/cmd_vel; '
        'exec bash'  # håll fönstret öppet
    )
    
    teleop = ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-lc', teleop_cmd],
            output='screen'
        )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/husky/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/husky/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/lidar/mid360/points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '--ros-args', '-r',
            '/lidar/mid360/points/points:=/lidar/mid360/points'
        ],
        output='screen'
    )

    simple_bridge = Node(
            package='seeker_sim',
            namespace='sim1',
            executable='cloud_frame_relay', #kanske byt till cloud_frame_relay.py
            name='sim'
        )
    
    lidar_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0.716','0','0','0','base_link','lidar'],
            output='screen'
        )
    
    delay_start_rviz = TimerAction(
        period= 3.0,
        actions=[start_rviz]
    )
    
    # Fire spawn only once gz is up (a small extra delay is still fine)
    spawn_when_gz_is_ready = RegisterEventHandler(
        OnProcessStart(
            target_action=start_gz,
            on_start=[TimerAction(period=1.5, actions=[spawn_cmd])]
        )
    )


    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(model_arg)
    ld.add_action(world_name_arg)
    ld.add_action(name_arg)
    ld.add_action(x_arg); ld.add_action(y_arg); ld.add_action(z_arg)
    ld.add_action(roll_arg); ld.add_action(pitch_arg); ld.add_action(yaw_arg)

    ld.add_action(set_gz_path)
    ld.add_action(start_gz)
    ld.add_action(spawn_when_gz_is_ready)
    ld.add_action(lidar_tf) 
    ld.add_action(bridge)
    ld.add_action(simple_bridge)
    
    ld.add_action(teleop)
    ld.add_action(delay_start_rviz)
    
    


    return LaunchDescription([ld])
