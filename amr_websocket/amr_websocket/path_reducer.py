import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from amr_websocket_interfaces.msg import WebsocketPath
import numpy as np

def douglas_peucker(points, epsilon):
    if len(points) < 3:
        return points

    first_point = points[0]
    last_point = points[-1]
    max_dist = 0
    index = 0

    for i in range(1, len(points) - 1):
        point = points[i]
        dist = np.abs(np.cross(last_point[:2] - first_point[:2], first_point[:2] - point[:2])) / np.linalg.norm(last_point[:2] - first_point[:2])
        if dist > max_dist:
            max_dist = dist
            index = i

    if max_dist > epsilon:
        left_simplified = douglas_peucker(points[:index+1], epsilon)
        right_simplified = douglas_peucker(points[index:], epsilon)
        return left_simplified[:-1] + right_simplified
    else:
        return [first_point, last_point]

class PathSimplifier(Node):
    def __init__(self):
        super().__init__('path_simplifier')
        self.subscription = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.publisher = self.create_publisher(WebsocketPath, 'websocket_path', 10)
        self.epsilon = 0.2  # Adjust this value to control simplification level

    def path_callback(self, msg: Path):
        if len(msg.poses) < 3:
            self.publisher.publish(msg)
            return

        points = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] for pose in msg.poses])
        simplified_points = douglas_peucker(points, self.epsilon)

        simplified_msg = WebsocketPath()
        simplified_msg.header = msg.header
        simplified_msg.poses = [Point(x=pose[0], y=pose[1], z=pose[2]) for pose in simplified_points]
        # simplified_msg.poses = [PoseStamped(header=msg.header, pose=PoseStamped().pose) for _ in simplified_points]
        
        # for i, pose in enumerate(simplified_msg.poses):
        #     pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = simplified_points[i]
        
        self.publisher.publish(simplified_msg)
        self.get_logger().info(f'Reduced path points from {len(msg.poses)} to {len(simplified_msg.poses)}')

def main():
    rclpy.init()
    node = PathSimplifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
