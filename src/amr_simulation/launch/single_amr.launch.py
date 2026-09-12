import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
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
            elif k == 'map_topic':
                new_dict[k] = f'/{name}/map'
            else:
                new_dict[k] = update_params_dict(v, name)
        return new_dict
    elif isinstance(data, list):
        return [update_params_dict(item, name) for item in data]
    else:
        return data

def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('namespace').perform(context)
    x = LaunchConfiguration('initial_pose_x').perform(context)
    y = LaunchConfiguration('initial_pose_y').perform(context)
    yaw = LaunchConfiguration('initial_pose_yaw').perform(context)

    pkg_share = get_package_share_directory('amr_simulation')
    xacro_file = os.path.join(pkg_share, 'models', 'amr.xacro')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    twist_mux_params_file = os.path.join(pkg_share, 'config', 'twist_mux_config.yaml')

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name,
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file, f' robot_name:={name}/']), value_type=str),
            'use_sim_time': True,
            'publish_frequency': 10.0
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
            '-x', x,
            '-y', y,
            '-z', '0.1',
            '-Y', yaw
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
            f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )


    # Build custom YAML dictionary for this robot namespace
    with open(nav2_params_file, 'r') as f:
        base_params = yaml.safe_load(f)

    robot_params = {'/**': {}}
    for key, value in base_params.items():
        robot_params['/**'][key] = update_params_dict(value, name)

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
            ('initial_pose_x', x),
            ('initial_pose_y', y),
            ('initial_pose_yaw', yaw),
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

    # twist_mux
    mux = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=name,
        output='screen',
        parameters=[twist_mux_params_file, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', 'cmd_vel')]
    )

    return [rsp, spawn, bridge, nav2, broadcaster, coordinator, mux, battery_sim, task_alloc]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Robot namespace'),
        DeclareLaunchArgument('initial_pose_x', description='Initial X position'),
        DeclareLaunchArgument('initial_pose_y', description='Initial Y position'),
        DeclareLaunchArgument('initial_pose_yaw', description='Initial Yaw orientation'),
        OpaqueFunction(function=launch_setup)
    ])
