import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_simulation')
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    xacro_file = os.path.join(pkg_share, 'models', 'amr.xacro')

    # Launch Gazebo (headless)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Bridge clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/warehouse_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/world/warehouse_world/clock', '/clock')],
        output='screen'
    )

    robots = [
        {'name': 'amr1', 'x': '-5.0', 'y': '4.0', 'yaw': '0.0'},
        {'name': 'amr2', 'x': '5.0', 'y': '-4.0', 'yaw': '3.14159'},
        {'name': 'amr3', 'x': '-5.0', 'y': '-4.0', 'yaw': '0.0'}
    ]

    nodes = [gazebo, clock_bridge]

    for robot in robots:
        name = robot['name']

        # Robot State Publisher
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file, f' robot_name:={name}/']),
                'use_sim_time': True
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        )

        # Spawn Entity
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', name,
                '-topic', f'/{name}/robot_description',
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', '0.1',
                '-Y', robot['yaw']
            ],
            output='screen'
        )

        # ROS-GZ Bridge for this robot
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=name,
            arguments=[
                f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{name}/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
                f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            remappings=[
                (f'/{name}/tf', 'tf')
            ],
            output='screen'
        )

        # Read and modify nav2_params.yaml as text per user instructions
        nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
        with open(nav2_params_file, 'r') as f:
            robot_yaml_text = f.read()

        # Replace frames and topics carefully
        robot_yaml_text = robot_yaml_text.replace('base_footprint', f'{name}/base_footprint')
        robot_yaml_text = robot_yaml_text.replace('global_frame: odom', f'global_frame: {name}/odom')
        robot_yaml_text = robot_yaml_text.replace('odom_frame_id: "odom"', f'odom_frame_id: "{name}/odom"')
        robot_yaml_text = robot_yaml_text.replace('odom_topic: /odom', f'odom_topic: /{name}/odom')
        robot_yaml_text = robot_yaml_text.replace('odom_topic: "odom"', f'odom_topic: "{name}/odom"')
        robot_yaml_text = robot_yaml_text.replace('/scan', f'/{name}/scan')

        # Prefix root keys with namespace
        lines = robot_yaml_text.split('\n')
        new_lines = []
        for line in lines:
            if line and not line[0].isspace() and ':' in line:
                new_lines.append(f'{name}/{line}')
            else:
                new_lines.append(line)
        
        tmp_params = f'/tmp/nav2_params_{name}.yaml'
        with open(tmp_params, 'w') as f:
            f.write('\n'.join(new_lines))

        # Nav2 Bringup
        nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'nav2_bringup.launch.py')
            ),
            launch_arguments=[
                ('namespace', name),
                ('use_sim_time', 'true'),
                ('initial_pose_x', robot['x']),
                ('initial_pose_y', robot['y']),
                ('initial_pose_yaw', robot['yaw']),
                ('params_file', tmp_params)
            ]
        )

        nodes.extend([rsp, spawn, bridge, nav2])

    return LaunchDescription(nodes)
