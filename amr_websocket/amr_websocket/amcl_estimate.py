import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
import math
from action_msgs.srv import CancelGoal
from nav2_simple_commander.robot_navigator import BasicNavigator
# from tf_transformations import quaternion_from_euler, euler_from_quaternion

class PoseJumpDetector(Node):
    def __init__(self):
        super().__init__('pose_jump_detector')
        self.navigator = BasicNavigator('planner_navigator')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # Topic name (adjust as needed)
            self.pose_callback,
            10)
        
        self.subscription_initial = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',  # Topic name (adjust as needed)
            self.initialpose_callback,
            10)

        self.goal_sub = self.create_subscription(
            Path,
            '/received_global_plan',
            self.goal_callback,
            10
        )

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.brake_pub = self.create_publisher(Bool, "motor_brake", 10)
        self.last_pose = None  # To store the previous pose
        self.last_initialpose = None
        self.position_threshold = 0.7  # meters (you can adjust this)
        self.angular_threshold = math.radians(45)  # 45 degrees (adjustable)
        self.jump_detected = False

        self.cancel_cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        while not self.cancel_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('cancel goal service not available. waiting again...')
        self.req = CancelGoal.Request()
    
    def initialpose_callback(self, pose):
        print("initial pose received")
        self.last_initialpose = pose

    def pose_callback(self, msg):
        # print("amcl pose received")
        # Get the current pose from the message
        # self.initialpose_pub.publish(self.last_initialpose)
        current_pose = msg.pose.pose
        current_position = current_pose.position
        current_orientation = current_pose.orientation
        # print(current_position.x)
        iniitialpose = self.last_initialpose.pose.pose.position 
        amcl_initial_change = math.sqrt(
            (current_position.x - iniitialpose.x) ** 2 +
            (current_position.y - iniitialpose.y) ** 2
        )
        print(f"amcl change: {amcl_initial_change}")

        if amcl_initial_change > 0.8:
            if self.last_pose is not None:
                # Calculate the position change (Euclidean distance)
                last_position = self.last_pose.pose.pose.position
                position_change = math.sqrt(
                    (current_position.x - last_position.x) ** 2 +
                    (current_position.y - last_position.y) ** 2
                )

                print((f"position change: {position_change}"))

                # Calculate the orientation change (using quaternions)
                last_orientation = self.last_pose.pose.pose.orientation
                # Convert quaternions to Euler angles for easier comparison
                # _, _, last_yaw = euler_from_quaternion([last_orientation.x, last_orientation.y, last_orientation.z, last_orientation.w])
                # _, _, current_yaw = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

                # Calculate the angular change (in radians)
                # angular_change = abs(current_yaw - last_yaw)

                # Check if there is a large jump in position or orientation
                if position_change > self.position_threshold:
                    self.get_logger().info(f"Large position jump detected: {position_change} meters")
                    print("+++++++++++goal canceled")
                    # self.cancel_cli.call_async(self.req)
                    msg = Bool()
                    msg.data = True
                    self.brake_pub.publish(msg)
                    time.sleep(2)
                    print("*********publishing initialpose")
                    # self.publisher.publish(self.last_pose)
                    self.initialpose_pub.publish(self.last_pose)
                    time.sleep(1)
                    print("*****Brake canceled")
                    msg.data = False
                    self.brake_pub.publish(msg)
                    # self.current_goal.header.stamp = self.navigator.get_clock().now().to_msg()
                    # self.navigator.goToPose(self.current_goal)
                    self.jump_detected = True
                    self.last_initialpose = self.last_pose

                # if angular_change > self.angular_threshold:
                #     pass
                    # self.get_logger().info(f"Large orientation jump detected: {math.degrees(angular_change)} degrees")

            # Store the current pose for the next callback
            if not self.jump_detected:
                self.last_pose = msg
            self.jump_detected = False
        else:
            self.last_pose = None
    
    def goal_callback(self, goal_path):
        self.goal_sent = True
        self.current_goal = goal_path.poses[len(goal_path.poses) - 1]

def main(args=None):
    rclpy.init(args=args)
    pose_jump_detector = PoseJumpDetector()

    try:
        rclpy.spin(pose_jump_detector)
    except KeyboardInterrupt:
        pass
    finally:
        pose_jump_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()