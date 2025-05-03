import json
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry

from amr_websocket.controls import ControlPublisher

class CAutoDock(Node):
    def __init__(self):
        super().__init__('control_panel_auto_dock')
        self.get_logger().info('control_panel_auto_dock node initialized')

        self.control_publisher = ControlPublisher('delivery')

        self.dock_sub = self.create_subscription(Bool, '/docking_topic', self.docking_callback, 10)
        self.station_sub = self.create_subscription(Int32, 'goal_to_pose_feedback', self.feedback_callback, 10)

        self.charging_pub = self.create_publisher(Bool, '/auto_dock', 10)
        self.timer = self.create_timer(0.1, self.dock_timer)
        self.initial_variables()
    
    def initial_variables(self):
        self.timer_started = False
        self.goal_sent = False
        self.isdocking = False
        self.t = 0.0
        self.rotation_time = (2* math.pi) / 0.4
    
    def docking_callback(self, dock_msg):
        if dock_msg.data:
            self.control_publisher.publish_color('m\n')
            self.control_publisher.send_goal('docker')
            self.goal_sent = True
            self.isdocking = True
            # if not self.timer_started:
            #     self.timer_started = True
    
    def dock_timer(self):
        # if self.timer_started:
        #     self.control_publisher.send_vel(0.0, 0.4)
        #     # self.t += 0.1 
        #     # if self.t >= self.rotation_time:
        #     #     self.t = 0.0
        #     #     self.control_publisher.send_vel(0.0, 0.0)
        #     self.control_publisher.send_goal('docker')
        #     self.goal_sent = True
        #     self.timer_started = False
        
        # else:
        if self.goal_sent:
            self.control_publisher.results()
    
    def feedback_callback(self, msg):
        self.get_logger().info(f'{msg.data}')
        self.get_logger().info(f'{self.isdocking}')
        if msg.data == 1 and self.isdocking:
            self.docking_pub(True)
            self.isdocking = False
            self.goal_sent = False
    
    def docking_pub(self, msg):
        dock = Bool()
        dock.data = msg
        self.charging_pub.publish(dock)

def main(args=None):
    rclpy.init(args=args)
    c_auto_dock = CAutoDock()
    executor = MultiThreadedExecutor()
    executor.add_node(c_auto_dock)

    try:
        executor.spin()
    except KeyboardInterrupt:
        c_auto_dock.destroy_node()
        rclpy.shutdown

if __name__ == '__main__':
    main()