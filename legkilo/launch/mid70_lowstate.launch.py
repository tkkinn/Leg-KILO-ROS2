import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_prefix = get_package_prefix("legkilo")
    pkg_share = get_package_share_directory("legkilo")
    legkilo_exec = os.path.join(pkg_prefix, "lib", "legkilo", "legkilo_node")
    default_config = os.path.join(pkg_share, "config", "mid70_lowstate.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "legkilo_robot.rviz")
    urdf_file = os.path.join(pkg_share, "robot_urdf", "go2_description", "urdf", "go2.urdf")

    with open(urdf_file, "r", encoding="utf-8") as urdf_fp:
        robot_description = urdf_fp.read()

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to legkilo YAML config",
            ),
            ExecuteProcess(cmd=[legkilo_exec, "--config_file", config_file], output="screen"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                prefix=["nice"],
                arguments=["-d", rviz_config],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "publish_frequency": 1000.0,
                        "robot_description": robot_description,
                    }
                ],
            ),
        ]
    )
