#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePrefixBridge(Node):
    """Translate joint names at the unprefixed AGX hardware boundary."""

    def __init__(self):
        super().__init__("joint_state_prefix_bridge")
        self.declare_parameter("prefix", "piper_")
        self.declare_parameter("feedback_input", "feedback/joint_states")
        self.declare_parameter("feedback_output", "/joint_states")
        self.declare_parameter(
            "command_input", "control/joint_states_prefixed"
        )
        self.declare_parameter("command_output", "control/joint_states")

        self._prefix = self.get_parameter("prefix").value
        feedback_input = self.get_parameter("feedback_input").value
        feedback_output = self.get_parameter("feedback_output").value
        command_input = self.get_parameter("command_input").value
        command_output = self.get_parameter("command_output").value

        self._feedback_pub = self.create_publisher(
            JointState, feedback_output, 10
        )
        self._command_pub = self.create_publisher(
            JointState, command_output, 10
        )
        self.create_subscription(
            JointState, feedback_input, self._feedback_callback, 10
        )
        self.create_subscription(
            JointState, command_input, self._command_callback, 10
        )

        self.get_logger().info(
            f"Translating AGX joint names with prefix '{self._prefix}'"
        )

    @staticmethod
    def _select(values, indices, name_count):
        if len(values) != name_count:
            return []
        return [values[index] for index in indices]

    def _translated_message(self, msg, names, indices):
        translated = JointState()
        translated.header = msg.header
        translated.name = names
        translated.position = self._select(
            msg.position, indices, len(msg.name)
        )
        translated.velocity = self._select(
            msg.velocity, indices, len(msg.name)
        )
        translated.effort = self._select(
            msg.effort, indices, len(msg.name)
        )
        return translated

    def _feedback_callback(self, msg):
        indices = list(range(len(msg.name)))
        names = [
            name if name.startswith(self._prefix) else self._prefix + name
            for name in msg.name
        ]
        self._feedback_pub.publish(
            self._translated_message(msg, names, indices)
        )

    def _command_callback(self, msg):
        indices = [
            index
            for index, name in enumerate(msg.name)
            if name.startswith(self._prefix)
        ]
        if not indices:
            return
        names = [
            msg.name[index][len(self._prefix):]
            for index in indices
        ]
        self._command_pub.publish(
            self._translated_message(msg, names, indices)
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePrefixBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
