from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

class Rviz2Publisher(Node):
    def __init__(self):
        super().__init__('rviz2_publisher')
        self.get_logger().info('**** Control_Publisher initialized ****')

        self.navigator = BasicNavigator('rviz2_navigator')

        self.pose_msg = PoseWithCovarianceStamped()
        self.initalpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
    
    def publish_initialpose(self, position:list, orientation:list):
        self.pose_msg.header.frame_id = 'map'
        self.pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg.pose.pose.position.x = position[0]
        self.pose_msg.pose.pose.position.y = position[1]
        self.pose_msg.pose.pose.position.z = position[2]
        self.pose_msg.pose.pose.orientation.x = orientation[0]
        self.pose_msg.pose.pose.orientation.y = orientation[1]
        self.pose_msg.pose.pose.orientation.w = orientation[2]
        self.pose_msg.pose.pose.orientation.z = orientation[3]
        self.initalpose_pub.publish(self.pose_msg)
    
    def send_goal(self, position:list, orientation:list):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = position[0]
        goal_pose.pose.position.y = position[1]
        goal_pose.pose.position.z = position[2]
        goal_pose.pose.orientation.x = orientation[0]
        goal_pose.pose.orientation.y = orientation[1]
        goal_pose.pose.orientation.w = orientation[2]
        goal_pose.pose.orientation.z = orientation[3]
        succ = self.navigator.goToPose(goal_pose)
        return succ
