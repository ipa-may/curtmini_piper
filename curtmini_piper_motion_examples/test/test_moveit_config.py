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

from dataclasses import replace
import importlib.util
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
import pytest

from curtmini_piper_motion_examples.moveit_goal import build_moveit_config
from curtmini_piper_motion_examples.moveit_goal import GoalConfig


def _goal_config() -> GoalConfig:
    return GoalConfig(
        goal_type="joints",
        controller_mode="simulation",
        joint_positions=[0.0] * 6,
        pose_position=[0.25, 0.0, 0.3],
        pose_orientation=[0.0, 0.0, 0.0, 1.0],
        plan_only=True,
        planning_pipeline="ompl",
        planner_id="RRTConnectkConfigDefault",
        planning_attempts=1,
        planning_time=5.0,
        velocity_scaling=0.1,
        acceleration_scaling=0.1,
        use_sim_time=True,
    )


def test_clock_qos_overrides_are_explicit():
    moveit_config = build_moveit_config(_goal_config())

    assert moveit_config[
        "qos_overrides./clock.subscription.durability"
    ] == "volatile"
    assert moveit_config[
        "qos_overrides./clock.subscription.reliability"
    ] == "best_effort"


def test_plan_request_parameters_use_default_namespace():
    moveit_config = build_moveit_config(_goal_config())

    assert moveit_config["plan_request_params"] == {
        "planning_attempts": 1,
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
        "planning_time": 5.0,
    }


@pytest.mark.parametrize("enabled", [True, False])
def test_lidar_link_and_collision_follow_configuration(enabled):
    config = build_moveit_config(replace(_goal_config(), use_hokuyo=enabled))
    robot = ET.fromstring(config["robot_description"])
    lidar = robot.find("./link[@name='hokuyo_link']")
    assert (lidar is not None) == enabled
    if enabled:
        assert lidar.find("collision") is not None
        assert robot.find("./joint[@name='hokuyo_joint']") is not None


@pytest.mark.parametrize("enabled", [True, False])
def test_rviz_and_moveitpy_share_lidar_model(enabled):
    launch_file = Path(get_package_share_directory("curtmini_piper_moveit_config"))
    spec = importlib.util.spec_from_file_location(
        "moveit_rviz_launch", launch_file / "launch/moveit_rviz.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        "use_sim_time": "true", "use_hokuyo": str(enabled).lower(),
        "piper_joint_limits_file": module._default_piper_joint_limits_file(),
    })
    with patch.object(module, "Node") as node:
        module._launch_setup(context)
    rviz_description = node.call_args.kwargs["parameters"][0]["robot_description"]
    config = build_moveit_config(replace(_goal_config(), use_hokuyo=enabled))
    for description in (rviz_description, config["robot_description"]):
        robot = ET.fromstring(description)
        assert (robot.find("./link[@name='hokuyo_link']") is not None) == enabled
