import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from rc_truck_interfaces.msg import BehaviorState


class VisionPipelineNode(Node):
    """Consumes camera images and publishes high-level perception cues."""

    def __init__(self) -> None:
        super().__init__('vision_pipeline_node')
        self.publisher = self.create_publisher(BehaviorState, 'perception/behavior_hint', 10)
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self._on_image,
            10,
        )
        self.get_logger().info('Vision pipeline node ready (stub processing).')
        self._toggle = False

    def _on_image(self, msg: Image) -> None:
        self._toggle = not self._toggle
        hint = BehaviorState()
        hint.stamp = msg.header.stamp
        hint.active_behavior = 'lane_follow' if self._toggle else 'explore'
        self.publisher.publish(hint)
        # TODO: replace stub logic with actual image processing pipeline.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionPipelineNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
