"""
Multi-Robot Warehouse Simulation Master Launch Script
Brings up:
  - Gazebo Sim running warehouse.sdf (1.15m choke point)
  - ROS-Gazebo clock and sensor bridges
  - 3 Namespaced AMRs ('amr1', 'amr2', 'amr3') with independent robot_state_publishers
  - Isolated Nav2 bringup instances per robot
  - P2P Intent Broadcaster & Decentralized Coordinator per robot
  - Dynamic Battery Simulator per robot
  - Contract Net Protocol (CNP) Dynamic Task Allocator per robot
  - Fleet Dashboard Server & Live Telemetry Bridge on http://localhost:8080
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def update_params_dict(data, name):
    """Recursively updates frame and topic references in Nav2 parameter dicts for a given robot namespace."""
    if isinstance(data, dict):
        new_dict = {}
        for k, v in data.items():
            if k in ['base_frame_id', 'robot_base_frame']:
                new_dict[k] = f'{name}/base_footprint'
            elif k in ['global_frame_id', 'global_frame']:
                new_dict[k] = f'{name}/odom' if v == 'odom' else v
            elif k == 'odom_frame_id':
                new_dict[k] = f'{name}/odom'
            elif k == 'odom_topic':
                new_dict[k] = f'/{name}/odom'
            elif k == 'scan_topic':
                new_dict[k] = f'/{name}/scan'
            elif k == 'topic' and isinstance(v, str) and 'scan' in v:
                new_dict[k] = f'/{name}/scan'
            elif k == 'topic' and isinstance(v, str) and 'odom' in v:
                new_dict[k] = f'/{name}/odom'
            else:
                new_dict[k] = update_params_dict(v, name)
        return new_dict
    elif isinstance(data, list):
        return [update_params_dict(item, name) for item in data]
    else:
        return data


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_simulation')
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    xacro_file = os.path.join(pkg_share, 'models', 'amr.xacro')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    twist_mux_params_file = os.path.join(pkg_share, 'config', 'twist_mux_config.yaml')

    # Launch Gazebo
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
        {'name': 'amr1', 'x': '-4.0', 'y': '0.0', 'yaw': '0.0'},
        {'name': 'amr2', 'x': '4.0', 'y': '0.0', 'yaw': '3.14159'},
        {'name': 'amr3', 'x': '1.0', 'y': '-4.0', 'yaw': '1.5708'}
    ]

    nodes = [gazebo, clock_bridge]

    # Load base nav2 params template
    with open(nav2_params_file, 'r') as f:
        base_params = yaml.safe_load(f)

    for robot in robots:
        name = robot['name']

        # Robot State Publisher
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', xacro_file, f' robot_name:={name}/']), value_type=str),
                'use_sim_time': True
            }],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
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
                (f'/{name}/tf', '/tf')
            ],
            output='screen'
        )

        # Build custom YAML dictionary for this robot namespace
        robot_params = {}
        for key, value in base_params.items():
            namespaced_key = f"{name}/{key}"
            robot_params[namespaced_key] = update_params_dict(value, name)

        tmp_params = f'/tmp/nav2_params_{name}.yaml'
        with open(tmp_params, 'w') as f:
            yaml.dump(robot_params, f)

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

        # Intent Broadcaster
        broadcaster = Node(
            package='amr_simulation',
            executable='intent_broadcaster.py',
            namespace=name,
            output='screen',
            parameters=[{'use_sim_time': True}]
        )

        # Decentralized Coordinator
        coordinator = Node(
            package='amr_simulation',
            executable='decentralized_coordinator.py',
            namespace=name,
            output='screen',
            parameters=[{'use_sim_time': True}]
        )

        # Battery Simulator
        battery_sim = Node(
            package='amr_simulation',
            executable='battery_simulator.py',
            namespace=name,
            output='screen',
            parameters=[{'use_sim_time': True}]
        )

        # CNP Task Allocator
        task_alloc = Node(
            package='amr_simulation',
            executable='task_allocator_cnp.py',
            namespace=name,
            output='screen',
            parameters=[{'use_sim_time': True}]
        )

        # twist_mux: multiplexes cmd_vel_nav (Nav2) and cmd_vel_coord (coordinator)
        # onto cmd_vel (consumed by Gazebo bridge). Coordinator has higher priority.
        mux = Node(
            package='twist_mux',
            executable='twist_mux',
            namespace=name,
            output='screen',
            parameters=[twist_mux_params_file, {'use_sim_time': True}],
            remappings=[('cmd_vel_out', 'cmd_vel')]
        )

        nodes.extend([rsp, spawn, bridge, nav2, broadcaster, coordinator, mux, battery_sim, task_alloc])

    # Fleet Dashboard Server & Live Telemetry Bridge
    dashboard_server = Node(
        package='amr_simulation',
        executable='fleet_dashboard_server.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    nodes.append(dashboard_server)

    return LaunchDescription(nodes)
