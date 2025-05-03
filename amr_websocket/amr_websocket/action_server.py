import rclpy
import time
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

class ActionServer(Node):
    def __init__(self):
        super().__init__("action_server_node")
        self.action_node = BasicNavigator("action_server_goal")

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "initialpose",
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped, 
            'websocket_goal',
            self.goal_callback,
            10
        )

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            'websocket_initialpose',
            self.initialpose_callback,
            10
        )

    
    def goal_callback(self, goal:PoseStamped):
        self.get_logger().info("Received goal")
        goal_pose = goal
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        succ = self.action_node.goToPose(goal_pose)
    
    def initialpose_callback(self, initialpose:PoseWithCovarianceStamped):
        initialpose_msg = initialpose
        initialpose_msg.header.frame_id = 'map'
        initialpose_msg.header.stamp = self.get_clock().now().to_msg()
        self.initialpose_pub.publish(initialpose_msg)

def main():
    rclpy.init(args=None)
    action_server_node = ActionServer()
    try:
        rclpy.spin(action_server_node)
    except KeyboardInterrupt:
        action_server_node.action_node.destroy_node()
        action_server_node.destroy_node()
        rclpy.shutdown()