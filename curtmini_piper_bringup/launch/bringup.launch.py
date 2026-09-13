import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


ARM_PREFIX = "piper_"
ARM_NAMESPACE = "piper"


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _three_values(context, name):
    value = LaunchConfiguration(name).perform(context)
    values = value.replace(",", " ").split()
    if len(values) != 3:
        raise RuntimeError(
            f"Launch argument '{name}' must contain exactly three values"
        )
    return values


def _float_array_literal(values):
    try:
        float_values = [float(value) for value in values]
    except ValueError as error:
        raise RuntimeError("TCP offset values must be numeric") from error
    return str(float_values)


def _find_serial_device(prefix, default):
    matches = [name for name in os.listdir("/dev") if name.startswith(prefix)]
    return matches[0] if len(matches) == 1 else default


def _build_moveit_config(context):
    description_share = Path(
        get_package_share_directory("curtmini_piper_description")
    )
    bringup_share = Path(
        get_package_share_directory("curtmini_piper_bringup")
    )
    mount_xyz = " ".join(_three_values(context, "arm_mount_xyz"))
    mount_rpy = " ".join(_three_values(context, "arm_mount_rpy"))
    tcp_xyz = " ".join(_three_values(context, "tcp_offset_xyz"))
    tcp_rpy = " ".join(_three_values(context, "tcp_offset_rpy"))

    mappings = {
        "simulation": "False",
        "arm_prefix": ARM_PREFIX,
        "arm_mount_xyz": mount_xyz,
        "arm_mount_rpy": mount_rpy,
        "tcp_offset_xyz": tcp_xyz,
        "tcp_offset_rpy": tcp_rpy,
    }

    full_description_file = (
        description_share / "urdf" / "curtmini_piper.urdf.xacro"
    )
    moveit_description_file = (
        description_share / "urdf" / "curtmini_piper_moveit.urdf.xacro"
    )
    full_robot_description = xacro.process_file(
        str(full_description_file), mappings=mappings
    ).toxml()

    moveit_config = (
        MoveItConfigsBuilder(
            "curtmini_piper",
            package_name="curtmini_piper_moveit_config",
        )
        .robot_description(
            file_path=str(moveit_description_file),
            mappings=mappings,
        )
        .robot_description_semantic(
            file_path="config/curtmini_piper.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .trajectory_execution(
            file_path=str(
                bringup_share / "config" / "moveit_controllers.yaml"
            )
        )
        .to_moveit_configs()
    )
    return moveit_config, full_robot_description


def _base_actions(context):
    if not _as_bool(context, "start_base"):
        return []

    curt_share = Path(get_package_share_directory("curt_mini"))
    ipa_share = Path(get_package_share_directory("ipa_ros2_control"))

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ipa_share / "launch" / "ros2_control.launch.py")
            ),
            launch_arguments={
                "controllers_file": str(curt_share / "config" / "ros2_control.yaml")
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(curt_share / "bringup" / "start_controller.launch.py")
            )
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[
                str(curt_share / "config" / "twist_mux.yaml"),
                {"use_sim_time": False},
            ],
            remappings=[("/cmd_vel_out", "/base_controller/cmd_vel")],
        ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "topic",
                "pub",
                "--rate",
                "20",
                "--print",
                "0",
                "/zero_twist/cmd_vel",
                "geometry_msgs/msg/TwistStamped",
                "{header: {stamp: now, frame_id: 'base_link'}}",
            ],
            output="log",
        ),
    ]

    if _as_bool(context, "start_joystick"):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(curt_share / "bringup" / "joystick.launch.py")
                )
            )
        )

    if _as_bool(context, "start_imu"):
        serial_device = _find_serial_device(
            "ttyLPMS", "ttyLPMSCA3D00510053"
        )
        actions.append(
            Node(
                package="openzen_driver",
                namespace="imu",
                executable="openzen_node",
                output="screen",
                parameters=[
                    {"sensor_interface": "LinuxDevice"},
                    {"sensor_name": f"devicefile:/dev/{serial_device}"},
                    {"frame_id": "imu_link"},
                ],
            )
        )

    return actions


def _arm_actions(context):
    description_share = Path(
        get_package_share_directory("curtmini_piper_description")
    )
    bringup_share = Path(
        get_package_share_directory("curtmini_piper_bringup")
    )
    use_hardware = _as_bool(context, "start_arm_hardware")

    arm_control_description = ParameterValue(
        Command(
            [
                "xacro ",
                str(
                    description_share
                    / "urdf"
                    / "piper_arm_control.urdf.xacro"
                ),
                f" arm_prefix:={ARM_PREFIX}",
            ]
        ),
        value_type=str,
    )
    joint_states_target = (
        "control/joint_states_prefixed" if use_hardware else "/joint_states"
    )

    actions = [
        Node(
            package="curtmini_piper_bringup",
            executable="robot_description_publisher.py",
            name="robot_description_publisher",
            namespace=ARM_NAMESPACE,
            output="screen",
            parameters=[
                {"robot_description": arm_control_description}
            ],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=ARM_NAMESPACE,
            output="screen",
            parameters=[
                str(
                    bringup_share
                    / "config"
                    / "piper_ros2_controllers.yaml"
                ),
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "--controller-ros-args",
                f"-r joint_states:={joint_states_target}",
                "joint_state_broadcaster",
                "--controller-manager",
                f"/{ARM_NAMESPACE}/controller_manager",
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
                f"/{ARM_NAMESPACE}/controller_manager",
                "--controller-manager-timeout",
                "60",
            ],
        ),
    ]

    if not use_hardware:
        return actions

    xyz = _three_values(context, "tcp_offset_xyz")
    rpy = _three_values(context, "tcp_offset_rpy")
    tcp_offset = _float_array_literal(xyz + rpy)
    agx_ctrl_share = Path(get_package_share_directory("agx_arm_ctrl"))

    actions.extend(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        agx_ctrl_share
                        / "launch"
                        / "start_single_agx_arm.launch.py"
                    )
                ),
                launch_arguments={
                    "namespace": ARM_NAMESPACE,
                    "can_port": LaunchConfiguration("can_port"),
                    "arm_type": "piper",
                    "effector_type": "none",
                    "auto_enable": LaunchConfiguration("auto_enable_arm"),
                    "speed_percent": LaunchConfiguration(
                        "arm_speed_percent"
                    ),
                    "tcp_offset": tcp_offset,
                    "control_enabled": "false",
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
            Node(
                package="curtmini_piper_bringup",
                executable="joint_state_prefix_bridge.py",
                name="joint_state_prefix_bridge",
                namespace=ARM_NAMESPACE,
                output="screen",
                parameters=[{"prefix": ARM_PREFIX}],
            ),
            Node(
                package="agx_arm_moveit",
                executable="agx_arm_control_gate",
                name="piper_control_gate",
                output="screen",
                parameters=[
                    {
                        "status_topics": [
                            "/piper/arm_controller/"
                            "follow_joint_trajectory/_action/status"
                        ],
                        "gate_service_name": "/piper/control_enable",
                    }
                ],
            ),
        ]
    )
    return actions


def _launch_setup(context):
    moveit_config, full_robot_description = _build_moveit_config(context)
    moveit_share = Path(
        get_package_share_directory("curtmini_piper_moveit_config")
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": full_robot_description}
            ],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                move_group_configuration,
            ],
            remappings=[("joint_states", "/joint_states")],
            additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        ),
    ]

    actions.extend(_base_actions(context))
    actions.extend(_arm_actions(context))

    if _as_bool(context, "use_rviz"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
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
                ],
                remappings=[("joint_states", "/joint_states")],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_base",
                default_value="true",
                choices=["true", "false"],
                description="Start Curt Mini hardware and drive controllers.",
            ),
            DeclareLaunchArgument(
                "start_joystick",
                default_value="true",
                choices=["true", "false"],
                description="Start Curt Mini joystick teleoperation.",
            ),
            DeclareLaunchArgument(
                "start_imu",
                default_value="true",
                choices=["true", "false"],
                description="Start the Curt Mini OpenZen IMU driver.",
            ),
            DeclareLaunchArgument(
                "start_arm_hardware",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Connect to the AGX arm. False keeps MoveIt on mock "
                    "hardware for visualization and planning."
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Start RViz with the MoveIt panel.",
            ),
            DeclareLaunchArgument(
                "can_port",
                default_value="can0",
                description="CAN interface used by the Piper arm.",
            ),
            DeclareLaunchArgument(
                "auto_enable_arm",
                default_value="true",
                choices=["true", "false"],
                description="Automatically enable the Piper arm.",
            ),
            DeclareLaunchArgument(
                "arm_speed_percent",
                default_value="100",
                description="Piper hardware speed percentage.",
            ),
            DeclareLaunchArgument(
                "arm_mount_xyz",
                default_value="0 0 0.18",
                description="Piper mount translation from Curt Mini chassis.",
            ),
            DeclareLaunchArgument(
                "arm_mount_rpy",
                default_value="0 0 0",
                description="Piper mount rotation from Curt Mini chassis.",
            ),
            DeclareLaunchArgument(
                "tcp_offset_xyz",
                default_value="0.0 0.0 0.0",
                description="TCP translation from piper_link6.",
            ),
            DeclareLaunchArgument(
                "tcp_offset_rpy",
                default_value="0.0 0.0 0.0",
                description="TCP rotation from piper_link6.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                choices=["debug", "info", "warn", "error", "fatal"],
                description="AGX arm log level.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
