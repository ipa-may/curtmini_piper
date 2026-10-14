"""Curt Mini with Piper H and AGX gripper."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = Path(get_package_share_directory("curtmini_piper_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_arm_hardware", default_value="false", choices=["true", "false"]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch" / "bringup.launch.py")),
            launch_arguments={
                "arm_model": "piper_h",
                "start_arm_hardware": LaunchConfiguration("start_arm_hardware"),
            }.items(),
        ),
    ])
