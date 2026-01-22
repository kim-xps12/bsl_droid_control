"""Subscriber node for pub_sub_python package."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """A minimal ROS 2 subscriber node that subscribes to String messages."""

    def __init__(self):
        """Initialize the subscriber node."""
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscriber node has been started')

    def listener_callback(self, msg):
        """Process received messages."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Entry point for the subscriber node."""
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        minimal_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
