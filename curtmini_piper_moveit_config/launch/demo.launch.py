"""Run MoveIt with a mock Piper controller and RViz, without hardware."""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_piper_joint_limits_file():
    return str(
        Path(get_package_share_directory("agx_arm_urdf"))
        / "piper"
        / "config"
        / "joint_position_limits.yaml"
    )


def _launch_setup(context):
    description_share = Path(
        get_package_share_directory("curtmini_piper_description")
    )
    config_share = Path(
        get_package_share_directory("curtmini_piper_moveit_config")
    )
    joint_limits_file = LaunchConfiguration(
        "piper_joint_limits_file"
    ).perform(context)
    mappings = {
        "arm_prefix": "piper_",
        "piper_joint_limits_file": joint_limits_file,
    }
    robot_description = xacro.process_file(
        str(description_share / "urdf" / "curtmini_piper.urdf.xacro"),
        mappings=mappings,
    ).toxml()
    arm_control_description = xacro.process_file(
        str(description_share / "urdf" / "piper_arm_control.urdf.xacro"),
        mappings=mappings,
    ).toxml()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace="piper",
            output="screen",
            remappings=[("joint_states", "/joint_states")],
            parameters=[
                {"robot_description": arm_control_description},
                str(config_share / "config" / "demo_ros2_controllers.yaml"),
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/piper/controller_manager",
                "--controller-manager-timeout",
                "60",
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "arm_controller",
                "--controller-manager",
                "/piper/controller_manager",
                "--controller-manager-timeout",
                "60",
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(config_share / "launch" / "move_group.launch.py")
            ),
            launch_arguments={
                "controller_mode": "real",
                "use_sim_time": "false",
                "use_hokuyo": "false",
                "use_sensors_3d": "false",
                "piper_joint_limits_file": joint_limits_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(config_share / "launch" / "moveit_rviz.launch.py")
            ),
            launch_arguments={
                "use_sim_time": "false",
                "use_hokuyo": "false",
                "piper_joint_limits_file": joint_limits_file,
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
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
