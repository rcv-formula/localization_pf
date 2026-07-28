from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _source_dir():
    """Return the package source dir (<ws>/src/localization_pf) if this is a
    dev workspace, else the installed share dir. Reading config/map from source
    means edits take effect on the next launch without a rebuild."""
    share = Path(get_package_share_directory("localization_pf")).resolve()
    for base in [share, *share.parents]:
        candidate = base / "src" / "localization_pf"
        if (candidate / "config" / "config.yaml").exists():
            return candidate
    return share


def generate_launch_description():
    src = _source_dir()
    default_params = str(src / "config" / "config.yaml")
    default_map_dir = str(src / "map")

    params_file = LaunchConfiguration("params_file")
    map_dir = LaunchConfiguration("map_dir")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            # 자체 맵 로더가 읽을 폴더. 기본은 패키지 map/ 폴더.
            DeclareLaunchArgument("map_dir", default_value=default_map_dir),
            Node(
                package="localization_pf",
                executable="localization_pf_node",
                name="localization_pf",
                output="screen",
                parameters=[
                    params_file,
                    {"map_loader.map_dir": map_dir},
                ],
            ),
        ]
    )
