from pathlib import Path
from functools import partial

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


ARM_PREFIX = "piper_"
ARM_NAMESPACE = "piper"
GRIPPER_MODELS = ("piper_h", "piper_l", "piper_x")


def _arm_model(context):
    return LaunchConfiguration("arm_model").perform(context)


def _moveit_package(model):
    if model in GRIPPER_MODELS:
        return f"curtmini_{model}_gripper_moveit_config"
    return "curtmini_piper_moveit_config"


def _default_piper_joint_limits_file():
    return str(
        Path(get_package_share_directory("agx_arm_urdf"))
        / "piper"
        / "config"
        / "joint_position_limits.yaml"
    )


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _float_array_literal(values):
    try:
        float_values = [float(value) for value in values]
    except ValueError as error:
        raise RuntimeError("TCP offset values must be numeric") from error
    return str(float_values)


def _tcp_offset_literal(model):
    geometry_file = (
        Path(get_package_share_directory("curtmini_piper_description"))
        / "config" / "geometry.yaml"
    )
    with geometry_file.open(encoding="utf-8") as stream:
        tcp = yaml.safe_load(stream)["tcp_offset"]
    values = [tcp["position"][axis] for axis in ("x", "y", "z")]
    if model in GRIPPER_MODELS:
        values[2] += 0.1425
    values += [tcp["rotation"][axis] for axis in ("r", "p", "y")]
    return _float_array_literal(values)


def _build_moveit_config(context):
    model = _arm_model(context)
    description_share = Path(
        get_package_share_directory("curtmini_piper_description")
    )
    moveit_share = Path(
        get_package_share_directory(_moveit_package(model))
    )
    shared_moveit_share = Path(
        get_package_share_directory("curtmini_piper_moveit_config")
    )
    piper_joint_limits_file = LaunchConfiguration(
        "piper_joint_limits_file"
    ).perform(context)

    mappings = {
        "simulation": "False",
        "arm_prefix": ARM_PREFIX,
        "arm_model": model,
        "piper_joint_limits_file": piper_joint_limits_file,
    }

    full_description_file = (
        description_share / "urdf" / "curtmini_piper.urdf.xacro"
    )
    full_robot_description = xacro.process_file(
        str(full_description_file), mappings=mappings
    ).toxml()

    moveit_config = (
        MoveItConfigsBuilder(
            "curtmini_piper",
            package_name=_moveit_package(model),
        )
        .robot_description(
            file_path=str(full_description_file),
            mappings=mappings,
        )
        .robot_description_semantic(
            file_path="config/curtmini_piper.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .sensors_3d(
            file_path=str(shared_moveit_share / "config" / "sensors_3d.yaml")
        )
        .trajectory_execution(
            file_path=str(
                moveit_share / "config" / "moveit_controllers_real.yaml"
            )
        )
        .to_moveit_configs()
    )
    return moveit_config, full_robot_description


def _arm_actions(context):
    model = _arm_model(context)
    has_gripper = model in GRIPPER_MODELS
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
                f" arm_model:={model}",
                " piper_joint_limits_file:=",
                LaunchConfiguration("piper_joint_limits_file"),
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
                    / (
                        "piper_gripper_ros2_controllers.yaml"
                        if has_gripper else "piper_ros2_controllers.yaml"
                    )
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

    if has_gripper:
        actions.append(
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "gripper_controller",
                    "--controller-manager",
                    f"/{ARM_NAMESPACE}/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
            )
        )

    if not use_hardware:
        return actions

    tcp_offset = _tcp_offset_literal(model)
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
                    "arm_type": model,
                    "effector_type": "agx_gripper" if has_gripper else "none",
                    "auto_enable": LaunchConfiguration("auto_enable_arm"),
                    "speed_percent": LaunchConfiguration(
                        "arm_speed_percent"
                    ),
                    "fast_mode": LaunchConfiguration("fast_mode"),
                    "tcp_offset": tcp_offset,
                    # Homing owns control until measured feedback and the
                    # mock-backed controller state have been synchronized.
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
                parameters=[{"prefix": ARM_PREFIX, "commands_enabled": False}],
            ),
        ]
    )
    return actions


def _after_homing(event, context, ready_actions):
    if event.returncode != 0:
        return [LogInfo(msg="ERROR: Arm homing failed. MoveIt remains stopped; inspect /piper feedback and homing logs.")]
    return [LogInfo(msg="Arm homing verified; starting MoveIt."), *ready_actions]


def _launch_setup(context):
    model = _arm_model(context)
    moveit_config, full_robot_description = _build_moveit_config(context)
    shared_moveit_share = Path(
        get_package_share_directory("curtmini_piper_moveit_config")
    )

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": full_robot_description}
            ],
        ),
    ]

    ready_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(shared_moveit_share / "launch" / "move_group.launch.py")
            ),
            launch_arguments={
                "controller_mode": "real",
                "use_sim_time": "false",
                "use_hokuyo": "false",
                "use_sensors_3d": "true",
                "arm_model": model,
                "piper_joint_limits_file": LaunchConfiguration(
                    "piper_joint_limits_file"
                ),
            }.items(),
        ),
    ]

    if _as_bool(context, "use_rviz"):
        ready_actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=[
                    "-d",
                    str(shared_moveit_share / "config" / "moveit.rviz"),
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

    use_hardware = _as_bool(context, "start_arm_hardware")
    if use_hardware and not _as_bool(context, "control_enabled"):
        # Start the trajectory-based gate only after homing, so it cannot
        # close the driver gate or react to mock synchronization goals early.
        ready_actions.insert(0, Node(
            package="agx_arm_moveit",
            executable="agx_arm_control_gate",
            name="piper_control_gate",
            output="screen",
            parameters=[{
                "status_topics": ["/piper/arm_controller/follow_joint_trajectory/_action/status"],
                "gate_service_name": "/piper/control_enable",
            }],
        ))

    if use_hardware:
        homing = Node(
            package="curtmini_piper_bringup",
            executable="home_arm.py",
            name="home_arm",
            namespace=ARM_NAMESPACE,
            output="screen",
            parameters=[
                LaunchConfiguration("homing_config"),
                moveit_config.robot_description,
                {"prefix": ARM_PREFIX,
                 "has_gripper": model in GRIPPER_MODELS,
                 "release_control": _as_bool(context, "control_enabled")},
            ],
        )
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=homing,
            on_exit=partial(_after_homing, ready_actions=ready_actions),
        )))
        actions.extend(_arm_actions(context))
        actions.append(homing)
    else:
        actions.extend(_arm_actions(context))
        actions.extend(ready_actions)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_model",
                default_value="piper",
                choices=["piper", "piper_h", "piper_l", "piper_x"],
                description="Arm model mounted on Curt Mini.",
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
                "control_enabled",
                default_value="true",
                choices=["true", "false"],
                description="Keep driver control enabled after verified homing; false uses the trajectory gate afterward.",
            ),
            DeclareLaunchArgument(
                "homing_config",
                default_value=str(Path(get_package_share_directory("curtmini_piper_bringup")) / "config" / "homing.yaml"),
                description="Hardware startup home pose, speed, feedback tolerances and timeouts.",
            ),
            DeclareLaunchArgument(
                "fast_mode",
                default_value="false",
                choices=["true", "false"],
                description="Use MIT/JS move_js commands; unsafe on Piper firmware before 1.8-5 (mode switches can drop the arm).",
            ),
            DeclareLaunchArgument(
                "piper_joint_limits_file",
                default_value=_default_piper_joint_limits_file(),
                description="YAML file containing Piper URDF joint limits.",
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
