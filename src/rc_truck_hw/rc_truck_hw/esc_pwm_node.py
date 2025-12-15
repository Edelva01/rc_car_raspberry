import rclpy
from rclpy.node import Node

from rc_truck_interfaces.msg import ThrottleCommand


class EscPwmNode(Node):
    """Converts ThrottleCommand messages into ESC PWM output."""

    def __init__(self) -> None:
        super().__init__('esc_pwm_node')
        self.declare_parameter('esc_gpio_pin', 18)
        self.declare_parameter('pwm_frequency', 50)
        self.declare_parameter('min_duty_cycle', 5.0)
        self.declare_parameter('max_duty_cycle', 10.0)
        self.subscription = self.create_subscription(
            ThrottleCommand,
            'throttle_cmd',
            self._on_throttle_cmd,
            10,
        )
        self.get_logger().info('ESC PWM node initialized (hardware stubs in place).')
        # TODO: initialize GPIO/PWM backend when running on hardware.

    def _on_throttle_cmd(self, msg: ThrottleCommand) -> None:
        min_duty = self.get_parameter('min_duty_cycle').value
        max_duty = self.get_parameter('max_duty_cycle').value
        duty = min_duty + (max_duty - min_duty) * (msg.throttle_percent / 100.0)
        duty = max(min_duty, min(max_duty, duty))
        self.get_logger().debug(
            'Received throttle %.1f%% -> duty %.2f (timestamp %r)',
            msg.throttle_percent,
            duty,
            msg.stamp,
        )
        # TODO: forward the duty cycle to the configured PWM channel.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EscPwmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
