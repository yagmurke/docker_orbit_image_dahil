import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math
from tf_transformations import euler_from_quaternion

class ImuOrientationNode(Node):
    def __init__(self):
        super().__init__('imu_orientation_node')

        # Son yaw değerini saklamak için değişken
        self.last_yaw_deg = 0.0

        # IMU verisini abone ol
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10)

        # Dereceyi publish edeceğimiz topic
        self.publisher_ = self.create_publisher(Float32, '/imu_topic', 10)
        
        # 0.5 saniyelik timer oluştur
        self.timer = self.create_timer(0.5, self.timer_callback)

    def imu_callback(self, msg):
        # Quaternion değerlerini al
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Quaternion'dan Euler (roll, pitch, yaw) açılar
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])

        # Yaw açısını dereceye çevir
        yaw_deg = math.degrees(yaw)

        # 0 - 360 dereceye normalize et
        self.last_yaw_deg = (yaw_deg + 360) % 360

    def timer_callback(self):
        # 0.5 saniyede bir yayınla
        msg_out = Float32()
        msg_out.data = self.last_yaw_deg
        self.publisher_.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOrientationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
