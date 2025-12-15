import rclpy
from rclpy.node import Node


class ThrottleNode(Node):
    """Placeholder node for ESC throttle control."""

    def __init__(self) -> None:
        super().__init__('throttle_node')
        self.get_logger().info('ThrottleNode initialized. TODO: wire up PWM control for ESC.')
        # TODO: integrate with actual PWM interface once ROS 2 environment is ready.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrottleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
