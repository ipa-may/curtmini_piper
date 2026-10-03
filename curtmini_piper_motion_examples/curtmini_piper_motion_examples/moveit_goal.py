# Copyright 2026 Fraunhofer IPA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass
import os
from pathlib import Path
import sys
import traceback
from typing import Sequence

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder
import rclpy
from rclpy.logging import get_logger


JOINT_NAMES = [f"piper_joint{index}" for index in range(1, 7)]
CONTROLLERS = {
    "simulation": "arm_controller",
    "hardware": "piper/arm_controller",
}


@dataclass
class GoalConfig:
    goal_type: str
    controller_mode: str
    joint_positions: list[float]
    pose_position: list[float]
    pose_orientation: list[float]
    plan_only: bool
    planning_pipeline: str
    planner_id: str
    planning_attempts: int
    planning_time: float
    velocity_scaling: float
    acceleration_scaling: float
    use_sim_time: bool


def _as_float_list(value, name: str) -> list[float]:
    if not isinstance(value, Sequence) or isinstance(value, str):
        raise ValueError(f"{name} must be a sequence")
    return [float(item) for item in value]


def _controller_configuration(controller_name: str) -> dict:
    return {
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/"
            "MoveItSimpleControllerManager"
        ),
        "moveit_simple_controller_manager": {
            "controller_names": [controller_name],
            controller_name: {
                "type": "FollowJointTrajectory",
                "joints": JOINT_NAMES,
                "action_ns": "follow_joint_trajectory",
                "default": True,
            },
        },
    }


def build_moveit_config(config: GoalConfig) -> dict:
    description_share = Path(
        get_package_share_directory("curtmini_piper_description")
    )
    mappings = {
        "simulation": "False",
        "arm_prefix": "piper_",
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
        .planning_pipelines(
            default_planning_pipeline=config.planning_pipeline,
            pipelines=[config.planning_pipeline],
            load_all=False,
        )
        .to_moveit_configs()
    )
    moveit_config.sensors_3d = {}

    config_dict = moveit_config.to_dict()
    config_dict.update(
        _controller_configuration(CONTROLLERS[config.controller_mode])
    )
    config_dict["planning_pipelines"] = {
        "pipeline_names": [config.planning_pipeline]
    }
    config_dict["plan_request_params"] = {
        "planning_attempts": config.planning_attempts,
        "planning_pipeline": config.planning_pipeline,
        "planner_id": config.planner_id,
        "max_velocity_scaling_factor": config.velocity_scaling,
        "max_acceleration_scaling_factor": config.acceleration_scaling,
        "planning_time": config.planning_time,
    }
    config_dict["use_sim_time"] = config.use_sim_time
    config_dict.update(
        {
            "qos_overrides./clock.subscription.depth": 1,
            "qos_overrides./clock.subscription.durability": "volatile",
            "qos_overrides./clock.subscription.history": "keep_last",
            "qos_overrides./clock.subscription.reliability": "best_effort",
        }
    )
    return config_dict


def _read_config() -> GoalConfig:
    node = rclpy.create_node("curtmini_piper_moveit_goal_params")
    try:
        node.declare_parameter("goal_type", "joints")
        node.declare_parameter("controller_mode", "simulation")
        node.declare_parameter(
            "joint_positions", [0.0, 0.5, -0.7, 0.0, 0.8, 0.0]
        )
        node.declare_parameter("pose_position", [0.25, 0.0, 0.30])
        node.declare_parameter(
            "pose_orientation", [0.0, 0.7071068, 0.0, 0.7071068]
        )
        node.declare_parameter("plan_only", True)
        node.declare_parameter("planning_pipeline", "ompl")
        node.declare_parameter(
            "planner_id", "RRTConnectkConfigDefault"
        )
        node.declare_parameter("planning_attempts", 1)
        node.declare_parameter("planning_time", 5.0)
        node.declare_parameter("velocity_scaling", 0.1)
        node.declare_parameter("acceleration_scaling", 0.1)

        config = GoalConfig(
            goal_type=node.get_parameter("goal_type").value,
            controller_mode=node.get_parameter("controller_mode").value,
            joint_positions=_as_float_list(
                node.get_parameter("joint_positions").value,
                "joint_positions",
            ),
            pose_position=_as_float_list(
                node.get_parameter("pose_position").value,
                "pose_position",
            ),
            pose_orientation=_as_float_list(
                node.get_parameter("pose_orientation").value,
                "pose_orientation",
            ),
            plan_only=bool(node.get_parameter("plan_only").value),
            planning_pipeline=node.get_parameter(
                "planning_pipeline"
            ).value,
            planner_id=node.get_parameter("planner_id").value,
            planning_attempts=int(
                node.get_parameter("planning_attempts").value
            ),
            planning_time=float(
                node.get_parameter("planning_time").value
            ),
            velocity_scaling=float(
                node.get_parameter("velocity_scaling").value
            ),
            acceleration_scaling=float(
                node.get_parameter("acceleration_scaling").value
            ),
            use_sim_time=bool(node.get_parameter("use_sim_time").value),
        )
    finally:
        node.destroy_node()

    validate_config(config)
    return config


def validate_config(config: GoalConfig) -> None:
    if config.goal_type not in ("joints", "pose"):
        raise ValueError("goal_type must be either 'joints' or 'pose'")
    if config.controller_mode not in CONTROLLERS:
        raise ValueError(
            f"controller_mode must be one of {tuple(CONTROLLERS)}"
        )
    if len(config.joint_positions) != len(JOINT_NAMES):
        raise ValueError("joint_positions must contain 6 values")
    if len(config.pose_position) != 3:
        raise ValueError("pose_position must contain 3 values")
    if len(config.pose_orientation) != 4:
        raise ValueError("pose_orientation must contain 4 values")
    if not 0.0 < config.velocity_scaling <= 1.0:
        raise ValueError("velocity_scaling must be in (0.0, 1.0]")
    if not 0.0 < config.acceleration_scaling <= 1.0:
        raise ValueError("acceleration_scaling must be in (0.0, 1.0]")


def _set_goal(moveit, planning_component, config: GoalConfig) -> None:
    if config.goal_type == "joints":
        robot_state = RobotState(moveit.get_robot_model())
        robot_state.joint_positions = dict(
            zip(JOINT_NAMES, config.joint_positions)
        )
        robot_state.update()
        planning_component.set_goal_state(robot_state=robot_state)
        return

    pose = PoseStamped()
    pose.header.frame_id = "piper_base_link"
    pose.pose.position.x = config.pose_position[0]
    pose.pose.position.y = config.pose_position[1]
    pose.pose.position.z = config.pose_position[2]
    pose.pose.orientation.x = config.pose_orientation[0]
    pose.pose.orientation.y = config.pose_orientation[1]
    pose.pose.orientation.z = config.pose_orientation[2]
    pose.pose.orientation.w = config.pose_orientation[3]
    planning_component.set_goal_state(
        pose_stamped_msg=pose,
        pose_link="piper_tcp_link",
    )


def run_goal(
    config: GoalConfig,
    moveit_keepalive: list | None = None,
) -> bool:
    logger = get_logger("curtmini_piper_moveit_goal")
    moveit = MoveItPy(
        node_name="curtmini_piper_moveit_py",
        config_dict=build_moveit_config(config),
    )
    if moveit_keepalive is not None:
        moveit_keepalive.append(moveit)

    planning_component = moveit.get_planning_component("arm")
    planning_component.set_start_state_to_current_state()
    _set_goal(moveit, planning_component, config)

    plan_parameters = PlanRequestParameters(moveit, "")
    plan_result = planning_component.plan(
        single_plan_parameters=plan_parameters
    )
    if not plan_result:
        logger.error("Planning failed")
        return False

    logger.info("Planning succeeded")
    if config.plan_only:
        logger.info("plan_only is true; trajectory was not executed")
        return True

    controller_name = CONTROLLERS[config.controller_mode]
    logger.info(f"Executing with controller {controller_name}")
    moveit.execute(
        plan_result.trajectory,
        controllers=[controller_name],
    )
    logger.info("Execution command finished")
    return True


def main(args=None):
    rclpy.init(args=args)
    exit_code = 1
    moveit_keepalive = []
    try:
        success = run_goal(
            _read_config(),
            moveit_keepalive=moveit_keepalive,
        )
        exit_code = 0 if success else 1
    except BaseException:
        traceback.print_exc()
    finally:
        rclpy.shutdown()

    # Jazzy MoveItPy tears down a detached C++ executor unsafely.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)


if __name__ == "__main__":
    main()
