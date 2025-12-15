import rclpy
from rclpy.node import Node

from rc_truck_interfaces.msg import BehaviorState, ThrottleCommand, SteeringCommand, VehicleState


class BehaviorManagerNode(Node):
    """Simple behavior selector that produces throttle and steering commands."""

    def __init__(self) -> None:
        super().__init__('behavior_manager_node')
        self._last_behavior: BehaviorState | None = None
        self._last_state: VehicleState | None = None
        self.create_subscription(
            BehaviorState,
            'perception/behavior_hint',
            self._on_behavior_hint,
            10,
        )
        self.create_subscription(
            VehicleState,
            'perception/vehicle_state',
            self._on_vehicle_state,
            10,
        )
        self._throttle_pub = self.create_publisher(ThrottleCommand, 'throttle_cmd', 10)
        self._steering_pub = self.create_publisher(SteeringCommand, 'steering_cmd', 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info('Behavior manager node online (stub policy).')

    def _on_behavior_hint(self, msg: BehaviorState) -> None:
        self._last_behavior = msg

    def _on_vehicle_state(self, msg: VehicleState) -> None:
        self._last_state = msg

    def _tick(self) -> None:
        throttle = ThrottleCommand()
        steering = SteeringCommand()
        now = self.get_clock().now().to_msg()
        throttle.stamp = now
        steering.stamp = now

        if self._last_state and self._last_state.is_stuck:
            throttle.throttle_percent = 0.0
            steering.steering_angle = 0.0
        else:
            throttle.throttle_percent = 30.0
            behavior_name = self._last_behavior.active_behavior if self._last_behavior else 'lane_follow'
            steering.steering_angle = 0.2 if behavior_name == 'explore' else 0.0

        self._throttle_pub.publish(throttle)
        self._steering_pub.publish(steering)
        # TODO: replace stub policy with behavior tree or state machine.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BehaviorManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
