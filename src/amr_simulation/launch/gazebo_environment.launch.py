import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('amr_simulation')
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    headless_str = LaunchConfiguration('headless').perform(context)

    gz_args = f'-r -s --render-engine ogre {world_path}' if headless_str == 'true' else f'-r --render-engine ogre {world_path}'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/warehouse_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/world/warehouse_world/clock', '/clock')],
        output='screen'
    )

    return [gazebo, clock_bridge]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo in headless server mode (-s)'),
        OpaqueFunction(function=launch_setup)
    ])

