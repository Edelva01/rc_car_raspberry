import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range


class RangeSensorNode(Node):
    """Publishes distance sensor readings."""

    def __init__(self) -> None:
        super().__init__('range_sensor_node')
        self.declare_parameter('frame_id', 'range_link')
        self.declare_parameter('field_of_view', math.radians(15.0))
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 4.0)
        self.publisher = self.create_publisher(Range, 'range', 10)
        self.timer = self.create_timer(0.5, self._publish_stub)
        self.get_logger().info('Range sensor node started (publishing stub data).')
        # TODO: replace stub publishing with actual sensor driver I/O.

    def _publish_stub(self) -> None:
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = float(self.get_parameter('field_of_view').value)
        msg.min_range = float(self.get_parameter('min_range').value)
        msg.max_range = float(self.get_parameter('max_range').value)
        msg.range = 1.0
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RangeSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
