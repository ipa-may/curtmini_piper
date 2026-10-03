from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


ARM_PREFIX = "piper_"


def _default_piper_joint_limits_file():
    return str(
        Path(get_package_share_directory("agx_arm_urdf"))
        / "piper"
        / "config"
        / "joint_position_limits.yaml"
    )


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _launch_setup(context):
    description_share = Path(
        get_package_share_directory("curtmini_piper_description")
    )
    moveit_share = Path(
        get_package_share_directory("curtmini_piper_moveit_config")
    )
    mappings = {
        "simulation": "False",
        "arm_prefix": ARM_PREFIX,
        "piper_joint_limits_file": LaunchConfiguration(
            "piper_joint_limits_file"
        ).perform(context),
    }
    moveit_config = (
        MoveItConfigsBuilder(
            "curtmini_piper",
            package_name="curtmini_piper_moveit_config",
        )
        .robot_description(
            file_path=str(
                description_share
                / "urdf"
                / "curtmini_piper_moveit.urdf.xacro"
            ),
            mappings=mappings,
        )
        .robot_description_semantic(
            file_path="config/curtmini_piper.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d",
                str(moveit_share / "config" / "moveit.rviz"),
            ],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {"use_sim_time": _as_bool(context, "use_sim_time")},
            ],
            remappings=[("joint_states", "/joint_states")],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "false"],
                description="Use the simulation clock.",
            ),
            DeclareLaunchArgument(
                "piper_joint_limits_file",
                default_value=_default_piper_joint_limits_file(),
                description="YAML file containing Piper URDF joint limits.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
