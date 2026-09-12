"""
Single-Robot Warehouse World Launch Script
Spawns the warehouse SDF world and a single unnamespaced AMR baseline
with sensor bridges and robot_state_publisher.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('amr_simulation')
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    xacro_file = os.path.join(pkg_share, 'models', 'amr.xacro')
    twist_mux_params_file = os.path.join(pkg_share, 'config', 'twist_mux_config.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 for visualization'
    )

    # Process xacro
    robot_description_config = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    robot_description = {'robot_description': robot_description_config}

    # Launch Gazebo with our warehouse world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawn AMR
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'amr',
                   '-x', '-5.0',
                   '-y', '4.0',
                   '-z', '0.1']
    )

    # ROS-GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/warehouse_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/world/warehouse_world/clock', '/clock')
        ],
        output='screen'
    )

    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2_bringup.launch.py')
        )
    )

    # RViz2 Bringup
    rviz_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rviz.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Twist Mux
    mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[twist_mux_params_file, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', 'cmd_vel')]
    )

    # Benchmark Logger
    benchmark_logger = Node(
        package='amr_simulation',
        executable='benchmark_logger.py',
        name='benchmark_logger',
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge,
        nav2_bringup,
        rviz_bringup,
        mux,
        benchmark_logger
    ])