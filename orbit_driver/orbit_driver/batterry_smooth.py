import rclpy
from rclpy.node import Node
from arduino_msgs.msg import MotorDriver, BatteryStatus
from std_msgs.msg import Float32

class BatterySmoother(Node):
    def __init__(self):
        super().__init__('battery_smoother')
        
        # Parameters for battery
        self.smooth_size = 10
        self.smooth_data = []
        self.smooth_count = 0
        
        # Parameters for current
        self.current_smooth_size = 10
        self.current_smooth_data = []
        self.current_smooth_count = 0
        
        # Store last values
        self.last_voltage = 0.0
        self.last_current = 0.0
        
        # Subscribers
        self.motor_sub = self.create_subscription(
            MotorDriver,
            '/motor_driver_topic',
            self.motor_callback,
            10
        )
        
        self.battery_status_publisher = self.create_publisher(
            BatteryStatus,
            '/battery_status',
            10
        )

        # Create timer for publishing battery status every 3 seconds
        self.create_timer(3.0, self.publish_battery_status)

    def motor_callback(self, msg):
        self.set_smooth_battery_level(msg.motor_voltage)
        avg_current = (msg.left_motor_current + msg.right_motor_current) / 2.0
        self.set_smooth_current_level(avg_current)

    def set_smooth_battery_level(self, level):
        self.smooth_data.append(level)
        self.smooth_count += 1
        
        if self.smooth_count >= self.smooth_size:
            smoothened_voltage = sum(self.smooth_data) / self.smooth_size
            self.last_voltage = smoothened_voltage
            self.smooth_data = []
            self.smooth_count = 0

    def set_smooth_current_level(self, level):
        self.current_smooth_data.append(level)
        self.current_smooth_count += 1
        
        if self.current_smooth_count >= self.current_smooth_size:
            smoothened_current = sum(self.current_smooth_data) / self.current_smooth_size
            self.last_current = abs(smoothened_current)
            self.current_smooth_data = []
            self.current_smooth_count = 0

    def publish_battery_status(self):
        msg = BatteryStatus()
        msg.voltage_smth = round(float(self.last_voltage), 2)
        msg.current_smth = round(float(self.last_current), 2)
        self.battery_status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    battery_smoother = BatterySmoother()
    rclpy.spin(battery_smoother)
    battery_smoother.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
