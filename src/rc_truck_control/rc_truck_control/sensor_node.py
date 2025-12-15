import rclpy
from rclpy.node import Node


class SensorNode(Node):
    """Placeholder node for sensor data acquisition."""

    def __init__(self) -> None:
        super().__init__('sensor_node')
        self.get_logger().info('SensorNode initialized. TODO: configure sensor publishers.')
        # TODO: wire up actual sensor polling and message publishing.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
