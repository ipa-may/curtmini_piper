"""Unit tests for the hardware-independent helpers of home_arm.py."""

import importlib.util
import math
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "home_arm.py"
_spec = importlib.util.spec_from_file_location("home_arm", SCRIPT)
home_arm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(home_arm)

LIMITS = [
    (-2.618, 2.618),
    (0.0, 3.14159),
    (-2.96706, 0.0),
    (-1.745, 1.745),
    (-1.22, 1.22),
    (-2.0944, 2.0944),
]


def _description(prefix="piper_", limits=LIMITS, extra=""):
    joints = "".join(
        f'<joint name="{prefix}joint{index}" type="revolute">'
        f'<limit lower="{lower}" upper="{upper}" effort="100" velocity="3"/>'
        "</joint>"
        for index, (lower, upper) in enumerate(limits, start=1)
    )
    return f'<robot name="test">{joints}{extra}</robot>'


def test_joint_limits_reads_prefixed_joints_in_order():
    extra = '<joint name="piper_gripper" type="prismatic"><limit lower="0" upper="0.1"/></joint>'
    assert home_arm.joint_limits(_description(extra=extra), "piper_") == LIMITS


def test_joint_limits_rejects_missing_joint():
    with pytest.raises(KeyError):
        home_arm.joint_limits(_description(limits=LIMITS[:5]), "piper_")


def test_joint_limits_rejects_wrong_prefix():
    with pytest.raises(KeyError):
        home_arm.joint_limits(_description(prefix=""), "piper_")


@pytest.mark.parametrize("bad", [(1.0, 1.0), (1.0, -1.0), (float("nan"), 1.0), (-1.0, float("inf"))])
def test_joint_limits_rejects_invalid_range(bad):
    limits = [bad] + LIMITS[1:]
    with pytest.raises(ValueError):
        home_arm.joint_limits(_description(limits=limits), "piper_")


def test_inside_limits():
    home = [0.0, math.radians(5.0), math.radians(-5.0), 0.0, 0.0, 0.0]
    assert home_arm.inside_limits(home, LIMITS)
    # Resting pose observed on the real arm: slightly past the joint2/joint3 stops.
    resting = [0.0, -0.00888373, 0.00169297, 0.0, 0.0, 0.0]
    assert not home_arm.inside_limits(resting, LIMITS)


def test_smooth_positions_endpoints_and_clamping():
    start, target = [0.0, -0.01, 0.002], [0.0, 0.0873, -0.0873]
    assert home_arm.smooth_positions(start, target, 0.0, 2.0) == start
    assert home_arm.smooth_positions(start, target, -1.0, 2.0) == start
    assert home_arm.smooth_positions(start, target, 2.0, 2.0) == pytest.approx(target)
    assert home_arm.smooth_positions(start, target, 10.0, 2.0) == pytest.approx(target)
    midpoint = home_arm.smooth_positions(start, target, 1.0, 2.0)
    assert midpoint == pytest.approx([(a + b) / 2 for a, b in zip(start, target)])


def test_smooth_positions_is_monotonic_and_respects_speed_bound():
    # run() sizes the duration with the quintic's peak derivative (1.875), so the
    # commanded speed must never exceed max_speed_deg_s.
    speed = math.radians(5.0)
    start, target = [0.0], [math.radians(30.0)]
    duration = 1.875 * abs(target[0] - start[0]) / speed
    steps = 2000
    dt = duration / steps
    previous = start[0]
    peak = 0.0
    for step in range(1, steps + 1):
        (value,) = home_arm.smooth_positions(start, target, step * dt, duration)
        assert value >= previous
        peak = max(peak, (value - previous) / dt)
        previous = value
    assert peak <= speed * (1.0 + 1e-6)
    assert peak == pytest.approx(speed, rel=1e-3)
