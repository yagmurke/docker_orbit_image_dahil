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
        super().__init__("keepout_point_publisher")
        self.get_logger().info('KeepoutPointPublisher node started')
        self.base_link_frame = "base_link"
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Depth of 1 to simulate latched topic
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.tf_sub = self.create_subscription(TransformStamped, 'tf_websocket', self.tf_callback, 10)
        self.keepout_point_publisher = self.create_publisher(Path, 'keepout_points', qos_profile)

        self.robot_tf = None
        self.keepout_points = Path()
        self.keepout_point_publisher.publish(self.keepout_points)
    
    def joy_callback(self, msg:Joy):
        if msg.buttons[4] == 1:
            if self.robot_tf is not None:
                paths = self.keepout_points.poses
                if len(paths) > 0:
                    last_point = paths[-1]
                    x0 = last_point.pose.position.x
                    y0 = last_point.pose.position.y
                    x1 = self.robot_tf.transform.translation.x
                    y1 = self.robot_tf.transform.translation.y
                    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
                    if d <= 0.5:
                        self.keepout_points.poses.pop()
                        self.get_logger().info("Removed keepout point due to collision")
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.base_link_frame
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.pose.position.x = self.robot_tf.transform.translation.x
                pose_stamped.pose.position.y = self.robot_tf.transform.translation.y
                self.keepout_points.poses.append(pose_stamped)
                self.keepout_points.header.frame_id = 'map'
                self.keepout_points.header.stamp = self.get_clock().now().to_msg()
                self.keepout_point_publisher.publish(self.keepout_points)
    
    def tf_callback(self, msg:TransformStamped):
        if msg.child_frame_id == self.base_link_frame:
            self.robot_tf = msg
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