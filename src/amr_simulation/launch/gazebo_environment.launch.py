import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('amr_simulation')
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/warehouse_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/world/warehouse_world/clock', '/clock')],
        output='screen'
    )

    return LaunchDescription([gazebo, clock_bridge])
