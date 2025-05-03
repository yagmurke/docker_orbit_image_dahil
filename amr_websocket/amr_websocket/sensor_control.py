import json
import rclpy
from rclpy.node import Node
from arduino_msgs.msg import Buttons
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry

class SensorControl(Node):
    def __init__(self):
        super().__init__('sensor_control_node')
        self.get_logger().info('Sensor controler node initialized')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('timeout', 10)
            ]
        )
        self.timeout = self.get_parameter('timeout').get_parameter_value().bool_value
        self.reset_sensors()
        self.update_sensors()

        self.cam1_sub = self.create_subscription(LaserScan, 'camera1/scan', self.cam1_callback, 10)
        self.cam2_sub = self.create_subscription(LaserScan, 'camera2/scan', self.cam2_callback, 10)
        self.front_lidar = self.create_subscription(LaserScan, '/scan', self.front_lidar_callback, 10)
        self.back_lidar = self.create_subscription(LaserScan, '/scan2', self.back_lidar_callback, 10)
        self.motor_sub = self.create_subscription(Odometry, '/wheel/odom', self.motor_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/bno055/imu', self.imu_callback, 10)
        self.arduino_sub = self.create_subscription(Buttons, 'arduino_topic', self.arduino_callback, 10)

        self.sensor_publisher = self.create_publisher(String, 'sensors_status', 10)

        # self.timer = self.create_timer(self.timeout, self.timer_callback)
        self.pub_timer = self.create_timer(0.5, self.publish_timer)

    
    def update_sensors(self):
        self.sensors_dict = {
            'cam1': self.cam1,
            'cam2': self.cam2,
            'front_lidar': self.lidar1,
            'back_lidar': self.lidar2,
            'motor': self.motor,
            'imu': self.imu,
            'arduino': self.arduino
        }
    
    def reset_sensors(self):
        self.cam1, self.cam2, self.lidar1, self.lidar2, self.motor, self.imu, self.arduino = 'disconnected', 'disconnected', 'disconnected', 'disconnected', 'disconnected', 'disconnected', 'disconnected'

    def cam1_callback(self, scan):
        self.cam1 = 'connected'

    def cam2_callback(self,scan):
        self.cam2 = 'connected'

    def front_lidar_callback(self, scan):
        self.lidar1 = 'connected'

    def back_lidar_callback(self, scan):
        self.lidar2 = 'connected'

    def motor_callback(self, odom):
        self.motor = 'connected'

    def imu_callback(self, imu):
        self.imu = 'connected'
    
    def arduino_callback(self, ard):
        self.arduino = 'connected'
    
    def timer_callback(self):
        self.update_sensors()
    
    def publish_timer(self):
        self.update_sensors()
        pub_msg = String()
        pub_msg.data = json.dumps(self.sensors_dict)
        self.sensor_publisher.publish(pub_msg)
        self.reset_sensors()

def main(args=None):
    rclpy.init(args=args)
    sensor_control = SensorControl()

    try:
        rclpy.spin(sensor_control)
    except KeyboardInterrupt:
        sensor_control.destroy_node()
        rclpy.shutdown

if __name__ == '__main__':
    main()