"""
Nav2 Bringup Launch Script for AMR Simulation.

This launch file initializes and configures the ROS 2 Nav2 navigation stack 
components for an Autonomous Mobile Robot (AMR). It sets up:
  - Static transform publisher (map -> odom)
  - Map Server (loading static grid map)
  - Controller Server (local trajectory generation / path following)
  - Planner Server (global path planning)
  - Behavior Server (recovery behaviors like back-up, spin, wait)
  - BT Navigator (Behavior Tree task execution & orchestration)
  - Lifecycle Manager (automated node configuration & activation)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """
    Evaluates launch context dynamically and returns the configured ROS 2 nodes.

    This function handles runtime parameter evaluation (such as namespaces, initial poses,
    and parameter file paths) and constructs Node descriptors for all Nav2 pipeline entities.
    """
    # Locate package share directory and map configuration
    pkg_share = get_package_share_directory('amr_simulation')
    map_file = os.path.join(pkg_share, 'maps', 'warehouse_map.yaml')

    # Evaluate launch configurations dynamically from the launch context
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str == 'true'
    namespace = LaunchConfiguration('namespace').perform(context)
    initial_pose_x = LaunchConfiguration('initial_pose_x').perform(context)
    initial_pose_y = LaunchConfiguration('initial_pose_y').perform(context)
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    # Compute namespace prefix for frame IDs (supports multi-robot setups)
    prefix = f'{namespace}/' if namespace else ''

    # Define nodes subject to Nav2 lifecycle management
    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator'
    ]

    # Instantiate ROS 2 Node declarations for the navigation system
    nodes = [
        # =========================================================================
        # Static Transform Publisher: map -> <namespace>/odom
        # Publishes the fixed frame transformation offset representing the robot's
        # initial starting pose in the global map frame.
        # =========================================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            namespace=namespace,
            arguments=['--x', initial_pose_x, '--y', initial_pose_y, '--z', '0.0',
                       '--yaw', initial_pose_yaw, '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', 'map', '--child-frame-id', f'{prefix}odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
            output='screen'
        ),

        # =========================================================================
        # Map Server: nav2_map_server
        # Serves static occupancy grid map data from the specified YAML map file.
        # =========================================================================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]
        ),

        # =========================================================================
        # Controller Server: nav2_controller
        # Runs local trajectory planners (e.g., DWB/TEB) to track global paths
        # and compute velocity commands (cmd_vel) while avoiding dynamic obstacles.
        # =========================================================================
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static'),
                        ('cmd_vel', 'cmd_vel_nav')]
        ),

        # =========================================================================
        # Planner Server: nav2_planner
        # Computes global paths from the current robot pose to target goal pose
        # using algorithms like NavFn or Smac Planner across the global costmap.
        # =========================================================================
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]
        ),

        # =========================================================================
        # Behavior Server: nav2_behaviors
        # Manages recovery and auxiliary behaviors (such as spin, backup, wait,
        # and clear costmaps) when navigation gets stuck or encounters errors.
        # =========================================================================
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]
        ),

        # =========================================================================
        # BT Navigator: nav2_bt_navigator
        # Orchestrates high-level navigation logic using Behavior Trees (BT),
        # coordinating planning, control, and recovery execution tasks.
        # =========================================================================
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': os.path.join(pkg_share, 'config', 'navigate_to_pose.xml')
            }],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]
        ),

        # =========================================================================
        # Lifecycle Manager: nav2_lifecycle_manager
        # Transition manager that automatically configures and activates all registered
        # Nav2 lifecycle nodes in sequence, bringing the stack up to active state.
        # =========================================================================
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
    """
    Main entry point for ROS 2 launch system.

    Declares launch arguments for simulation time, robot namespace, initial spawn poses,
    and parameter files, then defers node generation to launch_setup via OpaqueFunction.
    """
    # Locate default Nav2 parameters YAML configuration
    pkg_share = get_package_share_directory('amr_simulation')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Declare configurable launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo/clock) time if true'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace for multi-robot navigation'
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='-5.0',
            description='Initial x position of the robot in the map frame'
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='4.0',
            description='Initial y position of the robot in the map frame'
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial yaw orientation (in radians) of the robot'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file,
            description='Full path to the ROS 2 parameters file to use for all launched nodes'
        ),
        # Evaluate launch setup function with resolved launch arguments context
        OpaqueFunction(function=launch_setup)
    ])
