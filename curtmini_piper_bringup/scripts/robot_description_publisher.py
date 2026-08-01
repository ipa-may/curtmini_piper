#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    """Publish a namespaced ros2_control description with transient durability."""

    def __init__(self):
        super().__init__("robot_description_publisher")
        self.declare_parameter("robot_description", "")
        robot_description = self.get_parameter("robot_description").value
        if not robot_description:
            raise RuntimeError("robot_description parameter is empty")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            String, "robot_description", qos
        )
        self._message = String(data=robot_description)
        self._publisher.publish(self._message)
        self.get_logger().info(
            "Published transient robot_description"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
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
