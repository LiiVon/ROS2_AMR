from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_tasks",
                executable="site_config_manager_node",
                name="site_config_manager_node",
                output="screen",
            )
        ]
    )
