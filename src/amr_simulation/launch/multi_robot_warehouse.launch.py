import os
import signal
import subprocess
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def cleanup_zombie_nav2():
    """Cleanly terminates lingering Nav2 processes from prior runs before spawning new ones."""
    try:
        current_pid = os.getpid()
        # Strictly target binaries located in /opt/ros/humble/lib/nav2_*
        res = subprocess.run(
            ['pgrep', '-f', '/opt/ros/humble/lib/nav2_'],
            capture_output=True, text=True
        )
        if res.stdout:
            pids = [int(p.strip()) for p in res.stdout.split() if p.strip().isdigit()]
            killed = 0
            for pid in pids:
                if pid != current_pid:
                    try:
                        os.kill(pid, signal.SIGKILL)
                        killed += 1
                    except ProcessLookupError:
                        pass
            if killed > 0:
                print(f"[multi_robot_warehouse] Cleaned up {killed} lingering Nav2 process(es) from prior session.")
                time.sleep(0.5)
    except Exception as e:
        print(f"[multi_robot_warehouse] Note: Nav2 pre-cleanup skipped: {e}")

def generate_launch_description():
    cleanup_zombie_nav2()
    pkg_share = get_package_share_directory('amr_simulation')

    # Declare Launch Arguments
    declare_launch_gazebo = DeclareLaunchArgument('launch_gazebo', default_value='true', description='Launch Gazebo')
    declare_launch_amr1 = DeclareLaunchArgument('launch_amr1', default_value='true', description='Launch AMR1')
    declare_launch_amr2 = DeclareLaunchArgument('launch_amr2', default_value='true', description='Launch AMR2')
    declare_launch_amr3 = DeclareLaunchArgument('launch_amr3', default_value='true', description='Launch AMR3')
    declare_launch_dashboard = DeclareLaunchArgument('launch_dashboard', default_value='true', description='Launch Dashboard')
    declare_launch_rviz = DeclareLaunchArgument('launch_rviz', default_value='false', description='Launch RViz2')
    declare_headless = DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo in headless server mode')

    # Launch Gazebo Environment
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_environment.launch.py')
        ),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
        condition=IfCondition(LaunchConfiguration('launch_gazebo'))
    )

    # Launch AMR1
    amr1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'single_amr.launch.py')
        ),
        launch_arguments={
            'namespace': 'amr1',
            'initial_pose_x': '-4.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '0.0'
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_amr1'))
    )

    # Launch AMR2
    amr2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'single_amr.launch.py')
        ),
        launch_arguments={
            'namespace': 'amr2',
            'initial_pose_x': '4.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '3.14159'
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_amr2'))
    )

    # Launch AMR3
    amr3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'single_amr.launch.py')
        ),
        launch_arguments={
            'namespace': 'amr3',
            'initial_pose_x': '1.0',
            'initial_pose_y': '-4.0',
            'initial_pose_yaw': '1.5708'
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_amr3'))
    )

    # Launch Dashboard
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'fleet_dashboard.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_dashboard'))
    )

    # Launch RViz2
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rviz.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    # Benchmark Logger
    benchmark_logger = Node(
        package='amr_simulation',
        executable='benchmark_logger.py',
        name='benchmark_logger',
        output='screen'
    )

    return LaunchDescription([
        declare_launch_gazebo,
        declare_launch_amr1,
        declare_launch_amr2,
        declare_launch_amr3,
        declare_launch_dashboard,
        declare_launch_rviz,
        declare_headless,
        gazebo,
        amr1,
        amr2,
        amr3,
        dashboard,
        rviz,
        benchmark_logger
    ])
