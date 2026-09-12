from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    dashboard_server = Node(
        package='amr_simulation',
        executable='fleet_dashboard_server.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([dashboard_server])
