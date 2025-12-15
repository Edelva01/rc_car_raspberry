from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launches throttle, servo, and sensor nodes."""
    # TODO: configure node parameters once hardware interfaces are complete.
    return LaunchDescription([
        Node(
            package='rc_truck_control',
            executable='throttle_node',
            name='throttle_node',
            output='screen',
        ),
        Node(
            package='rc_truck_control',
            executable='servo_node',
            name='servo_node',
            output='screen',
        ),
        Node(
            package='rc_truck_control',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
        ),
    ])
