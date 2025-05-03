import numpy as np
import math
import rclpy
import time
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Bool,String
from arduino_msgs.msg import MotorDriver
from arduino_msgs.msg import Arduino
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import subprocess
from tf2_ros import TransformBroadcaster
from orbit_driver.zlac8015d_driver.zlac8015_v1 import Controller

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_imu', True),
                ('timeout_times', 5),
                ('use_emmergency_btn', True), 
                ('driver_port', '/dev/hub_motor'),
                ('wheel_radius', 0.125),
                ('wheel_diff', 0.3975),
                ('publish_frequency', 30)
            ]
        )

        self.use_imu = self.get_parameter('use_imu').get_parameter_value().bool_value
        self.timeout_tries = self.get_parameter('timeout_times').get_parameter_value().integer_value
        self.use_emmergency_btn = self.get_parameter('use_emmergency_btn').get_parameter_value().bool_value
        self.driver_port = self.get_parameter('driver_port').get_parameter_value().string_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_diff = self.get_parameter('wheel_diff').get_parameter_value().double_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().integer_value

        self.brake = False
        self.initial_turn = False
        self.motor_enabled = True
        self.timeout_count = 0
        self.x = 0.0
        self.y = 0.0
        self.previous_phi = 0.0
        self.phi = 0.0
        self.previous_time = time.time()

        while not self.connect_motor():
            self.connect_motor()
        print("connection successful")

        self.create_odom_publisher()
        self.create_emm_sub()
        self.create_imu_sub()
        self.gui_emm_sub()

        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.driver_publisher = self.create_publisher(MotorDriver, "motor_driver_topic", 1)
        self.publish_timer = self.create_timer(1 / self.publish_frequency, self.timer_callback)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            1
        )

        self.brake_sub = self.create_subscription(
            Bool, "motor_brake", self.brake_callback, 10
        )
    
    def connect_motor(self):
        # Connect to the motor driver
        try:
            self.cntr = Controller(port= self.driver_port)
            self.cntr.disable_motor()
            self.cntr.set_acc(0)
            self.cntr.set_decc(0)
            self.cntr.enable_motor()
            self.cntr.set_rpm(0, 0)
            self.get_logger().info("Connected to motor driver")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to connect to motor driver: {e}")
            return False
    
    def create_odom_publisher(self):
        if self.use_imu:
            topic = 'wheel/odom'
        else:
            topic = '/odom'
        
        self.odom_publisher = self.create_publisher(
            Odometry, topic, 1
        )
    
    def create_emm_sub(self):
        if self.use_emmergency_btn:
       
         self.button_sub = self.create_subscription(Arduino, 'arduino_topic', self.buttons_callback, 10)

    def create_imu_sub(self):
        if self.use_imu:
            self.imu_sub = self.create_subscription(Imu, 'bno055/imu', self.imu_callback, 1)
            self.get_logger().info("IMU is enabled")
    def gui_emm_sub(self):
        self.rbt_status = self.create_subscription(String, 'rbt_status', self.rbt_callback, 10)
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
    def imu_callback(self, msg:Imu):
        euler_orientation = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        # self.get_logger().info(f"IMU data: {euler_orientation}")
        if not self.initial_turn:
            self.previous_phi = euler_orientation[2]
            self.initial_turn = True
        self.phi = euler_orientation[2] - self.previous_phi
        self.phi = math.atan2(math.sin(self.phi), math.cos(self.phi))  # Normalize theta
        # self.previous_phi = euler_orientation[2]

    def buttons_callback(self, msg:Arduino):
        pass
        #use this for emmergency btn
        # if msg.emmergency_btn == 0:
        #     if self.motor_enabled:
        #         self.cntr.disable_motor()
        #         self.motor_enabled = False
        # else:
        #     if not self.motor_enabled:
        #         self.cntr.enable_motor()
        #         self.motor_enabled = True
    
    def cmd_callback(self, cmd:Twist):
        if not self.brake:
            linear_vel_rpm = self.linear_to_rpm(cmd.linear.x) * 10 # *10 for driver decimal drive
            angular_vel_rpm = self.angular_to_rpm(cmd.angular.z) * 10 # *10 for driver decimal drive
            left_motor_rpm = linear_vel_rpm - angular_vel_rpm
            right_motor_rpm = (linear_vel_rpm + angular_vel_rpm) * -1

            try:
                self.cntr.set_rpm(left_motor_rpm, right_motor_rpm)
                self.timeout_count = 0
            except Exception as e:
                self.get_logger().error(f"Failed to set motor speed: {e}")
                self.timeout_count += 1
            if self.timeout_count >= self.timeout_tries:
                self.get_logger().error(f"Timeout reached, motor speed not set, trying to reconnect")
                while not self.connect_motor():
                    self.connect_motor()
    
    def brake_callback(self, brake:Bool):
        self.brake = brake.data
        if self.brake:
            self.cntr.set_rpm(0,0)
            self.get_logger().info("Motor stopped")
    
    def linear_to_rpm(self, v):
        return int(30 * v / (np.pi * self.wheel_radius))
    
    def angular_to_rpm(self, w):
        return int(15 * w * self.wheel_diff / (self.wheel_radius * np.pi))
    
    def rpm_to_linear(self, rpm):
        return self.wheel_radius * rpm * np.pi / 30
    
    # def update_pos(self, v, omega, delta_t):
    #     if abs(omega) < 1e-6:  # Small angular velocity approximation
    #         self.x += v * math.cos(self.phi) * delta_t
    #         self.y += v * math.sin(self.phi) * delta_t
    #     else:
    #         # Arc-based motion update
    #         self.x += (v / omega) * (math.sin(self.phi + omega * delta_t) - math.sin(self.phi))
    #         self.y += (v / omega) * (-math.cos(self.phi + omega * delta_t) + math.cos(self.phi))

    #     self.phi += omega * delta_t  # Update heading
    #     self.phi = math.atan2(math.sin(self.phi), math.cos(self.phi))  # Normalize theta
    
    def update_pos(self, v, omega, delta_t):

        self.x += v * math.cos(self.phi) * delta_t
        self.y += v * math.sin(self.phi) * delta_t

        if not self.use_imu:
            self.phi += omega * delta_t  # Update heading
            self.phi = math.atan2(math.sin(self.phi), math.cos(self.phi))  # Normalize theta
        # self.get_logger().info(f"phi: {self.phi}, x: {self.x}, y: {self.y}")
    
    def timer_callback(self):
        try:
            registers = self.cntr.read_all()
            l_rpm, r_rpm = self.cntr.read_motor_speed(registers)
            l_speed = self.rpm_to_linear(l_rpm / 10)
            r_speed = self.rpm_to_linear(r_rpm / 10) * -1
            linear_speed = (l_speed + r_speed) / 2
            angular_speed = (r_speed - l_speed) / self.wheel_diff
            delta_t = time.time() - self.previous_time
            if delta_t > (1/self.publish_frequency) * 2:
                # print(f"jump detected {delta_t}")
                delta_t = 1 / self.publish_frequency # avoid position jumps
            # print(delta_t, 1/self.publish_frequency)
            self.update_pos(linear_speed, angular_speed, delta_t)
            self.previous_time = time.time()

            quaternion = quaternion_from_euler(0, 0, self.phi)
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.x 
            odom.pose.pose.position.y = self.y 
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = quaternion[0]
            odom.pose.pose.orientation.y = quaternion[1]
            odom.pose.pose.orientation.z = quaternion[2]
            odom.pose.pose.orientation.w = quaternion[3]
            odom.twist.twist.linear.x = linear_speed
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = angular_speed

            if not self.use_imu:
                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'odom'
                tf_msg.child_frame_id = 'base_link'

                tf_msg.transform.translation.x = self.x
                tf_msg.transform.translation.y = self.y
                tf_msg.transform.translation.z = 0.0

                tf_msg.transform.rotation.x = quaternion[0]
                tf_msg.transform.rotation.y = quaternion[1]
                tf_msg.transform.rotation.z = quaternion[2]
                tf_msg.transform.rotation.w = quaternion[3]
                self.tf_broadcaster_.sendTransform(tf_msg)
            self.odom_publisher.publish(odom)
            self.motor_driver_publisher(registers)
            self.timeout_count = 0
        except Exception as e:
            self.get_logger().error(f"Failed to read motor speed: {e}")
            self.timeout_count += 1
        if self.timeout_count >= self.timeout_tries:
                self.get_logger().error(f"Timeout reached, motor speed not read, trying to reconnect")
                while not self.connect_motor():
                    self.connect_motor()
                self.timeout_count = 0
    
    def motor_driver_publisher(self, registers:list):
        msg = MotorDriver()
        msg.left_motor_temp, msg.right_motor_temp = self.cntr.read_temperature(registers)
        msg.driver_temp = self.cntr.read_driver_temp(registers)
        msg.left_motor_current, msg.right_motor_current = self.cntr.read_current(registers)
        msg.motor_voltage = self.cntr.read_voltage(registers)
        msg.motor_status = self.motor_enabled
        self.driver_publisher.publish(msg)
    
    def close_motor(self):
        self.cntr.disable_motor()

def main(args=None):
    rclpy.init(args=args)

    motor_driver = MotorDriverNode()

    try:
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        motor_driver.close_motor()
        time.sleep(1)
        motor_driver.close_motor()

        motor_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()