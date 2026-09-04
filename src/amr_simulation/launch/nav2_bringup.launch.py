import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('amr_simulation')
    map_file = os.path.join(pkg_share, 'maps', 'warehouse_map.yaml')

    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str == 'true'
    namespace = LaunchConfiguration('namespace').perform(context)
    initial_pose_x = LaunchConfiguration('initial_pose_x').perform(context)
    initial_pose_y = LaunchConfiguration('initial_pose_y').perform(context)
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    prefix = f'{namespace}/' if namespace else ''

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator'
    ]

    nodes = [
        # Static transform: map -> namespace/odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            namespace=namespace,
            arguments=['--x', initial_pose_x, '--y', initial_pose_y, '--z', '0.0',
                       '--yaw', initial_pose_yaw, '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', 'map', '--child-frame-id', f'{prefix}odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen'
        ),

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),

        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),

        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),

        # Behavior server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]
        )
    ]

    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory('amr_simulation')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('initial_pose_x', default_value='-5.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='4.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('params_file', default_value=nav2_params_file),
        OpaqueFunction(function=launch_setup)
    ])
