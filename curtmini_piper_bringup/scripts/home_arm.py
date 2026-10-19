#!/usr/bin/env python3
"""Home from measured feedback, synchronize mock controllers, then release control."""

import math
import signal
import time
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint


class HomingError(RuntimeError):
    pass


def joint_limits(description, prefix):
    joints = {joint.get("name"): joint for joint in ET.fromstring(description).iter("joint")}
    limits = []
    for index in range(1, 7):
        limit = joints[f"{prefix}joint{index}"].find("limit")
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
            raise ValueError(f"Invalid joint{index} limits")
        limits.append((lower, upper))
    return limits


def inside_limits(positions, limits):
    return all(lo <= value <= hi for value, (lo, hi) in zip(positions, limits))


def smooth_positions(start, target, elapsed, duration):
    # Quintic interpolation: zero velocity/acceleration at either end.
    u = min(1.0, max(0.0, elapsed / duration))
    fraction = u**3 * (10.0 + u * (-15.0 + 6.0 * u))
    return [a + fraction * (b - a) for a, b in zip(start, target)]


class HomeArm(Node):
    def __init__(self):
        super().__init__("home_arm")
        defaults = {
            "robot_description": "",
            "prefix": "piper_",
            "has_gripper": True,
            "release_control": True,
            "home_joint_positions_deg": [0.0, 5.0, -5.0, 0.0, 0.0, 0.0],
            "max_speed_deg_s": 5.0,
            "position_tolerance_deg": 0.5,
            "settle_motion_deg": 0.1,
            "settle_time": 1.0,
            "startup_timeout": 30.0,
            "motion_timeout": 30.0,
            "feedback_timeout": 0.5,
            "command_rate": 50.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        params = {name: self.get_parameter(name).value for name in defaults}
        self.prefix = params["prefix"]
        self.has_gripper = params["has_gripper"]
        self.release_control = params["release_control"]
        self.target = [math.radians(value) for value in params["home_joint_positions_deg"]]
        self.limits = joint_limits(params["robot_description"], self.prefix)
        for name in ("max_speed_deg_s", "position_tolerance_deg", "settle_motion_deg",
                     "settle_time", "startup_timeout", "motion_timeout", "feedback_timeout", "command_rate"):
            if not math.isfinite(params[name]) or params[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.speed = math.radians(params["max_speed_deg_s"])
        self.tolerance = math.radians(params["position_tolerance_deg"])
        self.settle_motion = math.radians(params["settle_motion_deg"])
        self.settle_time = params["settle_time"]
        self.startup_timeout = params["startup_timeout"]
        self.motion_timeout = params["motion_timeout"]
        self.feedback_timeout = params["feedback_timeout"]
        if len(self.target) != 6 or not all(
            lo + self.tolerance < value < hi - self.tolerance
            for value, (lo, hi) in zip(self.target, self.limits)
        ):
            raise ValueError("Home pose must have six finite positions inside the limits, with tolerance margin")

        self.names = [f"joint{i}" for i in range(1, 7)]
        self.feedback_names = self.names + (["gripper"] if self.has_gripper else [])
        self.feedback = None
        self.feedback_at = 0.0
        self.feedback_stamp = 0
        self.commands = None
        self.commands_at = 0.0
        self.ramp = None
        self.hold = None
        self.monitor_feedback = False
        self.motion_requested = False
        self.settle_since = None
        self.settle_anchor = None
        self.goal_handles = []

        # Position mode (move_j), never MIT/JS: Piper firmware before 1.8-5 cannot
        # switch between the two seamlessly and the arm can drop on a switch.
        self.publisher = self.create_publisher(JointState, "control/move_j", 1)
        self.create_subscription(JointState, "feedback/joint_states", self._feedback_callback, 1)
        self.create_subscription(JointState, "control/joint_states_prefixed", self._commands_callback, 1)
        self.bridge_gate = self.create_client(SetBool, "bridge_enable")
        self.driver_gate = self.create_client(SetBool, "control_enable")
        self.controllers = [(ActionClient(self, FollowJointTrajectory,
                                         "arm_controller/follow_joint_trajectory"), self.names)]
        if self.has_gripper:
            self.controllers.append((ActionClient(self, FollowJointTrajectory,
                                                 "gripper_controller/follow_joint_trajectory"), ["gripper"]))
        self.create_timer(1.0 / params["command_rate"], self._publish_command)

    @staticmethod
    def _positions(msg, names):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return None
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in names):
            return None
        return {name: positions[name] for name in names}

    def _feedback_callback(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        positions = self._positions(msg, self.feedback_names)
        if positions is None or abs(age) > self.feedback_timeout:
            self.feedback = None
            self.settle_since = None
            return
        if stamp <= self.feedback_stamp:
            return  # Replayed/duplicate messages must not refresh the watchdog.
        self.feedback, self.feedback_at, self.feedback_stamp = positions, time.monotonic(), stamp
        actual = [positions[name] for name in self.names]
        if self.ramp is None or time.monotonic() < self.ramp[2] + self.ramp[3]:
            return
        if not self._at_home(actual):
            self.settle_since = None
        elif self.settle_since is None or max(
            abs(a - b) for a, b in zip(actual, self.settle_anchor)
        ) > self.settle_motion:
            self.settle_since = time.monotonic()
            self.settle_anchor = actual

    def _commands_callback(self, msg):
        names = [self.prefix + name for name in self.feedback_names]
        self.commands = self._positions(msg, names)
        self.commands_at = time.monotonic()

    def _fresh_feedback(self):
        return self.feedback is not None and time.monotonic() - self.feedback_at < self.feedback_timeout

    def _at_home(self, actual):
        return inside_limits(actual, self.limits) and all(
            abs(a - b) <= self.tolerance for a, b in zip(actual, self.target)
        )

    def _check_feedback(self):
        if not self._fresh_feedback():
            raise HomingError("Hardware joint feedback is missing, invalid or stale")

    def _wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            if self.monitor_feedback:
                self._check_feedback()
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise HomingError(f"Timed out: {description}")
            rclpy.spin_once(self, timeout_sec=0.02)
        raise HomingError("ROS shutdown during homing")

    def _future(self, future, timeout, description):
        self._wait(future.done, timeout, description)
        return future.result()

    def _gate(self, client, enabled):
        response = self._future(client.call_async(SetBool.Request(data=enabled)),
                                3.0, f"setting {client.srv_name} to {enabled}")
        if not response.success:
            raise HomingError(response.message)

    def _publish_command(self):
        if self.ramp is None and self.hold is None:
            return
        self._check_feedback()
        positions = self.hold
        if self.ramp is not None:
            start, target, started, duration = self.ramp
            positions = smooth_positions(start, target, time.monotonic() - started, duration)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = positions
        self.publisher.publish(msg)

    def _synchronize_controllers(self, measured):
        # These actions move only the mock system: bridge forwarding stays blocked.
        for client, names in self.controllers:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [self.prefix + name for name in names]
            point = JointTrajectoryPoint()
            point.positions = [measured[name] for name in names]
            point.velocities = [0.0] * len(names)
            point.time_from_start.sec = 1
            goal.trajectory.points = [point]
            handle = self._future(client.send_goal_async(goal), 3.0, "controller synchronization acceptance")
            if not handle.accepted:
                raise HomingError("Controller rejected synchronization goal")
            self.goal_handles.append(handle)
            result = self._future(handle.get_result_async(), 5.0, "controller synchronization result")
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise HomingError(f"Controller synchronization failed: {result.result.error_string}")
        self._wait(lambda: self._commands_match(measured), 3.0, "synchronized controller output")

    def _commands_match(self, measured):
        if self.commands is None or time.monotonic() - self.commands_at >= self.feedback_timeout:
            return False
        return all(abs(self.commands[self.prefix + name] - measured[name]) < 1e-6
                   for name in self.feedback_names)

    def run(self):
        self.get_logger().info("Homing: waiting for driver, feedback, bridge and controllers; normal commands blocked")
        self._wait(lambda: self._fresh_feedback()
                   and all(client.service_is_ready() for client in (self.bridge_gate, self.driver_gate))
                   and all(client.server_is_ready() for client, _ in self.controllers)
                   and self.publisher.get_subscription_count() > 0,
                   self.startup_timeout, "startup readiness")
        self.monitor_feedback = True
        self._gate(self.bridge_gate, False)
        self._gate(self.driver_gate, False)
        start = [self.feedback[name] for name in self.names]
        # The maximum derivative of the quintic blend is 1.875.
        duration = max(1.0, 1.875 * max(abs(a - b) for a, b in zip(start, self.target)) / self.speed)
        if duration + self.settle_time >= self.motion_timeout:
            raise HomingError("Home move cannot fit the configured speed and motion timeout")
        self.get_logger().info(f"Homing toward {[round(math.degrees(x), 2) for x in self.target]} degrees")
        self.motion_requested = True
        self._gate(self.driver_gate, True)
        self.ramp = (start, self.target, time.monotonic(), duration)
        self._wait(lambda: self.settle_since is not None
                   and time.monotonic() - self.settle_since >= self.settle_time,
                   self.motion_timeout, "home pose inside limits and settled")

        measured = dict(self.feedback)
        self.ramp = None
        self.hold = [measured[name] for name in self.names]
        self._publish_command()
        self.get_logger().info("Home reached; synchronizing arm and gripper controllers with hardware feedback")
        self._synchronize_controllers(measured)
        actual = [self.feedback[name] for name in self.names]
        if not self._at_home(actual) or any(abs(a - b) > self.tolerance for a, b in zip(actual, self.hold)):
            raise HomingError("Hardware moved away from the home pose during controller synchronization")
        self._gate(self.driver_gate, False)
        self.hold = None
        self._gate(self.bridge_gate, True)
        self._gate(self.driver_gate, self.release_control)
        self.get_logger().info("Homing complete: hardware verified and controller handoff complete")

    def _hold_measured_pose(self):
        # The driver's emergency_stop uses move_js on firmware before 1.8-5, which
        # would switch to MIT mode; hold the measured pose in position mode instead.
        if not self.motion_requested or not self._fresh_feedback():
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [self.feedback[name] for name in self.names]
        self.publisher.publish(msg)

    def abort(self):
        self.ramp = self.hold = None
        self.monitor_feedback = False
        for handle in self.goal_handles:
            handle.cancel_goal_async()
        for client, request in (
            (self.bridge_gate, SetBool.Request(data=False)),
            (self.driver_gate, SetBool.Request(data=False)),
        ):
            if client is self.driver_gate:
                self._hold_measured_pose()
            try:
                if client.service_is_ready():
                    self._future(client.call_async(request), 1.0, f"stopping via {client.srv_name}")
            except Exception as error:
                self.get_logger().error(str(error))
        self.get_logger().error("Homing failed: bridge/driver blocking requested; MoveIt will not start")


def _interrupt(signum, frame):
    raise KeyboardInterrupt()


def main(args=None):
    # Keep the ROS context alive long enough to request a hold on interruption.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _interrupt)
    node = None
    result = 1
    try:
        node = HomeArm()
        node.run()
        result = 0
    except (Exception, KeyboardInterrupt) as error:
        if node is not None:
            node.get_logger().error(f"Homing aborted: {error}")
            node.abort()
        else:
            print(f"Invalid homing configuration: {error}", flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
