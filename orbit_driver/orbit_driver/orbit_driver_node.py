import numpy as np
import rclpy
import time
from datetime import datetime
from rclpy.node import Node
from arduino_msgs.msg import Buttons
from std_msgs.msg import String
import subprocess
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from orbit_driver.zlac8015d_driver.zlac8015d import Controller

# _RADIUS = 0.105
_RADIUS = 125
# _REV_PULSE = 16385
_REV_PULSE = 4096
_REV_WAIT = 6000
_WHEEL_DIF = 397.5

Wheel_distance = 0.3975
R_Wheel = 0.125

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('orbit_driver_node')
        self.get_logger().info("Initializing Orbit Motor Driver")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_imu', True),
                ('timeout_times', 5),
                ('use_emmergency_btn', True)
            ]
        )
        self.use_imu = self.get_parameter('use_imu').get_parameter_value().bool_value
        self.timeout_tries = self.get_parameter('timeout_times').get_parameter_value().integer_value
        self.use_emmergency_btn = self.get_parameter('use_emmergency_btn').get_parameter_value().bool_value

        self.cntr = Controller(port= '/dev/hub_motor')
        self.cntr.disable_motor()
        self.cntr.set_acc(0)
        self.cntr.set_decc(0)
        self.cntr.enable_motor()

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            1)
        

        self.subscription
        self.create_odom_publisher()
        self.create_emm_sub()
        self.gui_emm_sub()

        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        ###variables###
        self.current_time = 0
        self.previous_time = datetime.now()
        self.time_interval = 2
        self.start_timer = False
        self.pos_l = 0
        self.pos_r = 0
        self.pos_old_l = 0
        self.pos_old_r = 0
        self.pos_m_l = 0.0
        self.pos_m_r = 0.0
        self.phi = 0
        self.x = 0
        self.y = 0
        self.velo_x = 0.0
        self.ang_z = 0.0
        self.countout = 0
        self.motor_enabled = True
    
    def create_odom_publisher(self):
        if self.use_imu:
            topic = 'wheel/odom'
        else:
            topic = '/odom'
        
        self.odom_publisher = self.create_publisher(
            Odometry, topic, 10
        )
    
    def create_emm_sub(self):
        if self.use_emmergency_btn:
            self.button_sub = self.create_subscription(Buttons, 'arduino_topic', self.buttons_callback, 10)
    def gui_emm_sub(self):
        self.rbt_status = self.create_subscription(String, 'rbt_status', self.rbt_callback, 10)


    def listener_callback(self, msg):
        self.velo_x = msg.linear.x
        self.ang_z = msg.angular.z
        vel_x_l = int(self.linear_to_rpm(msg.linear.x * 1000)) * 10
        ang_v = int(self.angular_to_rpm(msg.angular.z)) * 10
        vel_x_r = vel_x_l * -1
        vel_to_left=self.left_wheel_action(msg.linear.x,msg.angular.z)
        vel_to_right=self.right_wheel_action(msg.linear.x,msg.angular.z)
        self.cntr.set_rpm(vel_to_left, vel_to_right)
        try:
            self.cntr.set_rpm(vel_x_l - ang_v, vel_x_r - ang_v)
            self.countout = 0
        except:
            self.countout += 1
        if self.countout >= self.timeout_tries:
            self.get_logger().error('CANNOT WRITE TO MOTOR DRIVER')
            self.destroy_node()
            rclpy.shutdown()
    
    def rbt_callback(self,msg):
        self.text = msg.data
        if self.text == 'motor_stop':
            if self.motor_enabled:
                    self.cntr.disable_motor()
                    self.motor_enabled = False
        if self.text == 'motor_start':
            if not self.motor_enabled:
                self.cntr.enable_motor()
                self.motor_enabled = True

        if self.text == 'system_off':
            subprocess.call(f'shutdown -h now', shell=True)
        
        # if self.text == 'system_reboot':
        #     subprocess.call(f'reboot', shell=True)

    def buttons_callback(self, msg):
        if msg.emmergency_btn == 1:
            if not self.start_timer:
                self.previous_time = datetime.now()
                self.start_timer = True
                if self.motor_enabled:
                    self.cntr.disable_motor()
                    self.motor_enabled = False
                    print('disabled once')
                elif not self.motor_enabled:
                    self.cntr.enable_motor()
                    self.motor_enabled = True
                    print('******enabled once')
        timeout = (datetime.now() - self.previous_time).total_seconds()
        if timeout >= self.time_interval:
            self.start_timer = False
            self.previous_time = datetime.now()
    
    def linear_to_rpm(self, x):
        return 30 * x / (np.pi * _RADIUS)

    def angular_to_rpm(self, w):
        return 15 * w * _WHEEL_DIF / (_RADIUS * np.pi)
    
    def left_wheel_action(self,linear,angular):
        left_wheel_linear=linear/R_Wheel
        left_wheel_angular=float((((Wheel_distance/2)*angular))/R_Wheel)
        left_vel=((left_wheel_linear)+(-left_wheel_angular))
        left_wheel=round(((60*left_vel)/2*np.pi))
        return left_wheel
    
    def right_wheel_action(self,linear,angular):
        right_wheel_linear=linear/R_Wheel
        right_wheel_angular=float((((Wheel_distance/2)*angular))/R_Wheel)
        right_vel=((-right_wheel_linear)-(right_wheel_angular))
        right_wheel=round(((60*right_vel)/2*np.pi))
        return right_wheel

    def close_motor(self):
        self.cntr.disable_motor()
    
    def timer_callback(self):
        try:
            self.pos_l = self.cntr.read_encoder()[0]
            self.pos_r = self.cntr.read_encoder()[1] * -1
            self.countout = 0
        except Exception as e:
            self.countout +=1 
            print(e)
        if self.countout >= self.timeout_tries:
            self.get_logger().error('CANNOT READ FROM MOTOR DRIVER')
            self.destroy_node()
            rclpy.shutdown()
            
        pos_diff_l = self.pos_l - self.pos_old_l
        pos_diff_r = self.pos_r - self.pos_old_r
        
        if abs(pos_diff_l) < _REV_WAIT and abs(pos_diff_r) < _REV_WAIT:
            self.pos_m_l = (pos_diff_l / _REV_PULSE) * (2 * np.pi * _RADIUS)
            self.pos_m_r = (pos_diff_r / _REV_PULSE) * (2 * np.pi * _RADIUS)
        
            pos_diff = self.pos_m_r - self.pos_m_l
            # print(self.pos_m_r, self.pos_m_l)
            avr_pos = (self.pos_m_l + self.pos_m_r) / 2
            self.phi += pos_diff / _WHEEL_DIF
            if self.phi > (2 * np.pi):
                self.phi -= (2 * np.pi)
            elif self.phi < (2 * np.pi):
                self.phi += (2 * np.pi)
            self.x += avr_pos * np.cos(self.phi)
            self.y += avr_pos * np.sin(self.phi)
        self.pos_old_l = self.pos_l
        self.pos_old_r = self.pos_r
        self.odom_pub()
    
    
    
    def odom_pub(self):
        quaternion = quaternion_from_euler(0, 0, self.phi)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x / 1000
        odom.pose.pose.position.y = self.y / 1000
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        v = ((self.pos_m_l + self.pos_m_r) / 2) / 47.3
        odom.twist.twist.linear.x = ((self.pos_m_l + self.pos_m_r) / 2) / 47.3
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = ((self.pos_m_r - self.pos_m_l) / _WHEEL_DIF) * 35

        if not self.use_imu:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id = 'base_link'

            tf_msg.transform.translation.x = self.x / 1000
            tf_msg.transform.translation.y = self.y / 1000
            tf_msg.transform.translation.z = 0.0

            tf_msg.transform.rotation.x = quaternion[0]
            tf_msg.transform.rotation.y = quaternion[1]
            tf_msg.transform.rotation.z = quaternion[2]
            tf_msg.transform.rotation.w = quaternion[3]
            self.tf_broadcaster_.sendTransform(tf_msg)
        self.odom_publisher.publish(odom)
            
 

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    minimal_subscriber.cntr.enable_motor()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.close_motor()
        time.sleep(1)
        minimal_subscriber.close_motor()

        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
