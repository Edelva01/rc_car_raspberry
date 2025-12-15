from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    hardware_config = PathJoinSubstitution([
        FindPackageShare('rc_truck_bringup'),
        'config',
        'hardware.yaml',
    ])

    return LaunchDescription([
        Node(
            package='rc_truck_hw',
            executable='esc_pwm_node',
            name='esc_pwm_node',
            output='screen',
            parameters=[hardware_config],
        ),
        Node(
            package='rc_truck_hw',
            executable='servo_pwm_node',
            name='servo_pwm_node',
            output='screen',
            parameters=[hardware_config],
        ),
        Node(
            package='rc_truck_hw',
            executable='range_sensor_node',
            name='range_sensor_node',
            output='screen',
            parameters=[hardware_config],
        ),
        Node(
            package='rc_truck_hw',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[hardware_config],
        ),
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
        Node(
            package='rc_truck_control',
            executable='throttle_node',
            name='throttle_supervisor',
            output='screen',
        ),
        Node(
            package='rc_truck_control',
            executable='servo_node',
            name='servo_supervisor',
            output='screen',
        ),
        Node(
            package='rc_truck_control',
            executable='sensor_node',
            name='sensor_supervisor',
            output='screen',
        ),
    ])
