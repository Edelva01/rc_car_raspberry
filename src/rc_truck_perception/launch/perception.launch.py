from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='rc_truck_perception',
            executable='vision_pipeline_node',
            name='vision_pipeline_node',
            output='screen',
        ),
        Node(
            package='rc_truck_perception',
            executable='obstacle_map_node',
            name='obstacle_map_node',
            output='screen',
        ),
    ])
