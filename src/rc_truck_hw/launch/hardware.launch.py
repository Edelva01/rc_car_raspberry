from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='rc_truck_hw',
            executable='esc_pwm_node',
            name='esc_pwm_node',
            output='screen',
            parameters=[{'esc_gpio_pin': 18}],
        ),
        Node(
            package='rc_truck_hw',
            executable='servo_pwm_node',
            name='servo_pwm_node',
            output='screen',
            parameters=[{'servo_gpio_pin': 19}],
        ),
        Node(
            package='rc_truck_hw',
            executable='range_sensor_node',
            name='range_sensor_node',
            output='screen',
        ),
        Node(
            package='rc_truck_hw',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),
    ])
