import rclpy
from rclpy.node import Node

from rc_truck_interfaces.msg import SteeringCommand


class ServoPwmNode(Node):
    """Drives the steering servo based on SteeringCommand messages."""

    def __init__(self) -> None:
        super().__init__('servo_pwm_node')
        self.declare_parameter('servo_gpio_pin', 19)
        self.declare_parameter('pwm_frequency', 50)
        self.declare_parameter('center_duty_cycle', 7.5)
        self.declare_parameter('range_duty_cycle', 2.5)
        self.subscription = self.create_subscription(
            SteeringCommand,
            'steering_cmd',
            self._on_steering_cmd,
            10,
        )
        self.get_logger().info('Servo PWM node initialized (hardware stubs in place).')
        # TODO: initialize servo PWM driver when hardware is attached.

    def _on_steering_cmd(self, msg: SteeringCommand) -> None:
        center = self.get_parameter('center_duty_cycle').value
        span = self.get_parameter('range_duty_cycle').value
        duty = center + span * (msg.steering_angle)
        self.get_logger().debug(
            'Received steering %.2f -> duty %.2f (timestamp %r)',
            msg.steering_angle,
            duty,
            msg.stamp,
        )
        # TODO: drive the servo PWM signal using the computed duty cycle.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoPwmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
