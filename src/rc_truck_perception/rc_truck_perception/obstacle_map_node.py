import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range

from rc_truck_interfaces.msg import VehicleState


class ObstacleMapNode(Node):
    """Fuses range data and produces a simple vehicle state estimate."""

    def __init__(self) -> None:
        super().__init__('obstacle_map_node')
        self.publisher = self.create_publisher(VehicleState, 'perception/vehicle_state', 10)
        self.subscription = self.create_subscription(
            Range,
            'range',
            self._on_range,
            10,
        )
        self.get_logger().info('Obstacle map node ready (stub mapping).')

    def _on_range(self, msg: Range) -> None:
        state = VehicleState()
        state.stamp = msg.header.stamp
        state.speed_mps = 0.0
        state.steering_angle = 0.0
        state.is_stuck = msg.range < (msg.min_range + 0.05)
        self.publisher.publish(state)
        # TODO: fuse multiple range readings and produce richer state estimates.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
