import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class CameraNode(Node):
    """Publishes camera frames from the onboard camera."""

    def __init__(self) -> None:
        super().__init__('camera_node')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('encoding', 'rgb8')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / 10.0, self._publish_stub)
        self.get_logger().info('Camera node started (publishing stub frames).')
        # TODO: integrate actual camera capture pipeline when hardware is available.

    def _publish_stub(self) -> None:
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        encoding = self.get_parameter('encoding').value
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.width = width
        msg.height = height
        msg.encoding = encoding
        msg.step = width * 3
        msg.is_bigendian = 0
        msg.data = bytes(width * height * 3)
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
