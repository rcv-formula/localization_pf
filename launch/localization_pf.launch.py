from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory("localization_pf")) / "config" / "config.yaml"
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="localization_pf",
                executable="localization_pf_node",
                name="localization_pf",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
