import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
import rclpy.time
from sensor_msgs.msg import Joy

class KeepoutPointPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")
        self.get_logger().info('PathPublisher node started')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Depth of 1 to simulate latched topic
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.keepout_point_publisher = self.create_publisher(Path, 'websocket_path', 10)
        self.path_sub = self.create_subscription(Path, "/plan", self.path_callback, 10)
        self.create_timer(1.0, self.path_publisher)

        self.robot_tf = None
        self.path = Path()
        self.path_received = False
        self.index = 0
        self.sep = 5
    
    def path_callback(self, path):
        test_path = []
        self.path = path
        self.path_received = True
        while self.index < len(self.path.poses) - self.sep:
            x1 = self.path.poses[self.index].pose.position.x
            x2 = self.path.poses[self.index + self.sep].pose.position.x
            y1 = self.path.poses[self.index].pose.position.y
            y2 = self.path.poses[self.index + self.sep].pose.position.y
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            # print(f"path diff {self.index}: x: {x2 - x1}, y: {y2 - y1}, d: {distance}")
            test_path.append(distance)
            self.index += self.sep
        print(f"max: {max(test_path)}, min: {min(test_path)}")
        self.index = 0
        # print(self.path)
    def path_publisher(self):
        
        if self.path_received:
            self.keepout_point_publisher.publish(self.path)
            self.path_received = False
def main():
    rclpy.init(args=None)
    tf_listener = KeepoutPointPublisher()
    try:
        rclpy.spin(tf_listener)
    except KeyboardInterrupt:
        tf_listener.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()