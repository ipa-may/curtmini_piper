"""Start the shared MoveIt planning server for the robot or Gazebo."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


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
    model = LaunchConfiguration("arm_model").perform(context)
    description_share = Path(
        get_package_share_directory("curtmini_piper_description")
    )
    config_package = (
        f"curtmini_{model}_gripper_moveit_config"
        if model != "piper" else "curtmini_piper_moveit_config"
    )
    config_share = Path(
        get_package_share_directory(config_package)
    )
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    if model != "piper" and controller_mode == "sim":
        raise RuntimeError("Gazebo controllers are not configured for Piper H/L yet")
    controller_file = config_share / "config" / f"moveit_controllers_{controller_mode}.yaml"
    mappings = {
        "simulation": "False",
        "arm_prefix": "piper_",
        "arm_model": model,
        "use_hokuyo": str(_as_bool(context, "use_hokuyo")),
        "piper_joint_limits_file": LaunchConfiguration(
            "piper_joint_limits_file"
        ).perform(context),
    }
    builder = (
        MoveItConfigsBuilder(
            "curtmini_piper",
            package_name=config_package,
        )
        .robot_description(
            file_path=str(
                description_share / "urdf" / "curtmini_piper.urdf.xacro"
            ),
            mappings=mappings,
        )
        .robot_description_semantic(file_path="config/curtmini_piper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path=str(controller_file))
    )
    if _as_bool(context, "use_sensors_3d"):
        builder.sensors_3d(
            file_path=str(
                Path(get_package_share_directory("curtmini_piper_moveit_config"))
                / "config" / "sensors_3d.yaml"
            )
        )
    moveit_config = builder.to_moveit_configs()
    if not _as_bool(context, "use_sensors_3d"):
        moveit_config.sensors_3d = {}

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": True,
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                    "monitor_dynamics": False,
                    "use_sim_time": _as_bool(context, "use_sim_time"),
                },
            ],
            remappings=[("joint_states", "/joint_states")],
            additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_model",
                default_value="piper",
                choices=["piper", "piper_h", "piper_l"],
            ),
            DeclareLaunchArgument(
                "controller_mode",
                default_value="real",
                choices=["real", "sim"],
                description="Select the real or Gazebo arm trajectory controller.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "use_hokuyo",
                default_value=LaunchConfiguration("use_sim_time"),
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "use_sensors_3d",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "piper_joint_limits_file",
                default_value=_default_piper_joint_limits_file(),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
