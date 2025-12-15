from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='rc_truck_navigation',
            executable='behavior_manager_node',
            name='behavior_manager_node',
            output='screen',
        ),
        Node(
            package='rc_truck_navigation',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
        ),
    ])
