import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("legkilo")
    default_config = os.path.join(pkg_share, "config", "hilti.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "legkilo.rviz")

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to legkilo YAML config",
            ),
            Node(
                package="legkilo",
                executable="legkilo_node",
                name="legkilo_node",
                output="screen",
                arguments=["--config_file", config_file],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                prefix=["nice"],
                arguments=["-d", rviz_config],
            ),
        ]
    )
