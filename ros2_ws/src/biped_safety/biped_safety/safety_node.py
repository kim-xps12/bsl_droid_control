# Copyright 2024, All rights reserved.
#
# Licensed under the MIT License.

import rclpy
from rclpy.node import Node


class BipedSafetyNode(Node):
    """Safety monitoring node for joint limits and posture checks."""

    def __init__(self) -> None:
        super().__init__('biped_safety_node')


def main(args=None):
    rclpy.init(args=args)
    node = BipedSafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
