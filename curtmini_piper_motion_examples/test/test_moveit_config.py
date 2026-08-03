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
        arm_mount_xyz=[0.0, 0.0, 0.18],
        arm_mount_rpy=[0.0] * 3,
        tcp_offset_xyz=[0.0] * 3,
        tcp_offset_rpy=[0.0] * 3,
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
