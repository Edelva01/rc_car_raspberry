from math import sin

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PathPlannerNode(Node):
    """Publishes a stub navigation path for downstream controllers."""

    def __init__(self) -> None:
        super().__init__('path_planner_node')
        self.publisher = self.create_publisher(Path, 'planned_path', 10)
        self.timer = self.create_timer(2.0, self._publish_path)
        self.get_logger().info('Path planner node online (publishing stub path).')
        self._seq = 0

    def _publish_path(self) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        poses = []
        for i in range(10):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(i)
            pose.pose.position.y = float(sin(0.3 * (self._seq + i)))
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        path.poses = poses
        self.publisher.publish(path)
        self._seq += 1
        # TODO: integrate real-time planning driven by goals and obstacles.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
