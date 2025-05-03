import rclpy
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from action_msgs.srv import CancelGoal

from PyQt5.QtCore import QThread, pyqtSignal

class Rviz2NodeThread(QThread):
    map_msg = pyqtSignal(OccupancyGrid)
    costmap_msg = pyqtSignal(OccupancyGrid)
    laserscan_msg = pyqtSignal(LaserScan)
    path_msg = pyqtSignal(Path)
    transform_msg = pyqtSignal(str, str, TransformStamped)

    def __init__(self):
        super(Rviz2NodeThread, self).__init__()
        self.running = True
        self.node = Node("rviz2_node_thread")
        self.rviz2_navigator = BasicNavigator("rviz2_navigator")
        self.laser_time = time.process_time()

        ####Declare parameters ####
        self.node.declare_parameters(
            namespace="",
            parameters=[
                ("map_topic", "/map"),
                ("costmap_topic", "/global_costmap/costmap"),
                ("laserscan_topic", "/scan"),
                ("path_topic", "/plan"),
                ("map_frame", "map"),
                ("laser_frame", "laser"),
                ("base_link_frame", "base_link")
            ]
        )

        self.map_topic = self.node.get_parameter("map_topic").get_parameter_value().string_value
        self.costmap_topic = self.node.get_parameter("costmap_topic").get_parameter_value().string_value
        self.laserscan_topic = self.node.get_parameter("laserscan_topic").get_parameter_value().string_value
        self.path_topic = self.node.get_parameter("path_topic").get_parameter_value().string_value
        self.map_frame = self.node.get_parameter("map_frame").get_parameter_value().string_value
        self.laser_frame = self.node.get_parameter("laser_frame").get_parameter_value().string_value
        self.base_link_frame = self.node.get_parameter("base_link_frame").get_parameter_value().string_value

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Depth of 1 to simulate latched topic
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        ####Subscriptions ####
        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile
        )

        self.costmap_sub = self.node.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            qos_profile
        )

        self.laserscan_sub = self.node.create_subscription(
            LaserScan,
            self.laserscan_topic,
            self.laserscan_callback,
            10
        )

        self.path_sub = self.node.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        ###Publishers ####
        self.initialpose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped,
            "initialpose",
            10
        )

        ####Tf Transform timer ####
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self.node)
        self.timer = self.node.create_timer(0.1, self.tf_timer)

        # Goal client
        self.cancel_cli = self.node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        while not self.cancel_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Cancel goal service not available, waiting ....")
        self.req = CancelGoal.Request()
    
    def map_callback(self, msg):
        self.map_msg.emit(msg)
    
    def costmap_callback(self, msg):
        self.costmap_msg.emit(msg)
    
    def laserscan_callback(self, msg):
        self.laserscan_msg.emit(msg)
    
    def path_callback(self, msg):
        self.path_msg.emit(msg)
    
    def tf_timer(self):
        frame_pairs = [
            (self.base_link_frame, self.map_frame),
            (self.laser_frame, self.map_frame)
        ]
        frame_pairs = [
            (self.base_link_frame, self.map_frame),
            (self.laser_frame, self.map_frame)
        ]

        for target_frame, fixed_frame in frame_pairs:
            try:
                t = self.buffer.lookup_transform(
                    fixed_frame,
                    target_frame,
                    rclpy.time.Time()
                )
                self.transform_msg.emit(target_frame, fixed_frame, t)
            except TransformException as e:
                self.node.get_logger().info(f"Can't transform from {target_frame} to {fixed_frame}")
    
    def publish_initialpose(self, position:list, orientation:list):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = position[0]
        pose_msg.pose.pose.position.y = position[1]
        pose_msg.pose.pose.position.z = position[2]
        pose_msg.pose.pose.orientation.x = orientation[0]
        pose_msg.pose.pose.orientation.y = orientation[1]
        pose_msg.pose.pose.orientation.w = orientation[2]
        pose_msg.pose.pose.orientation.z = orientation[3]
        self.initialpose_pub.publish(pose_msg)
    
    def send_goal(self, position:list, orientation:list):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = position[0]
        goal_pose.pose.position.y = position[1]
        goal_pose.pose.position.z = position[2]
        goal_pose.pose.orientation.x = orientation[0]
        goal_pose.pose.orientation.y = orientation[1]
        goal_pose.pose.orientation.w = orientation[2]
        goal_pose.pose.orientation.z = orientation[3]
        succ = self.rviz2_navigator.goToPose(goal_pose)
    
    def cancel_goal(self):
        self.cancel_cli.call_async(self.req)
    
    def run(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self.node)
        # executor.add_node(self.rviz2_navigator)

        while rclpy.ok() and self.running:
            try:
                executor.spin_once()
            except Exception as e:
                self.node.get_logger().info(f"Can't spin {e}")
        
    
    def stop(self):
        self.running = False
        self.node.destroy_node()
        rclpy.shutdown()