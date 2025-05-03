import serial
import json
import time
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from arduino_msgs.msg import Arduino
from std_msgs.msg import String

class ArduinoControl(Node):
    def __init__(self):
        super().__init__('arduino_publisher')
        self.get_logger().info('******arduino_publisher started******')

        
        self.ser = serial.Serial('/dev/arduino', timeout=1.0)
        if self.ser:
            time.sleep(3)  # Seri bağlantının oturmasını bekle
            self.get_logger().info("Arduino hazır mı kontrol ediliyor...")

            while True:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if "READY" in line:
                        self.get_logger().info("Arduino hazır. Kalibrasyon komutu gönderiliyor.")
                        self.ser.write(b'k\n')
                        self.get_logger().error("Arduino'ya kalibrasyon komutu (k) gönderildi.")
                        break
                time.sleep(0.1)
        self.last_received = ''
        self.buffer_string = ''
        self.previous_msg = None

        self.ard_color = self.create_subscription(String, 'arduino_color', self.color_callback, 10)
        self.publisher = self.create_publisher(Arduino, 'arduino_topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.subscription = self.create_subscription(
            String,
            '/motions',
            self.motions_callback,
            10
        )
        self.subscription
        self.duygu_komutlari = {
            'greeting': 'q',
            'reject': 'w',
            'confirm': 'e',
            'curious': 'r',
            'thinking': 't',
            'happy': 'y',
            'sad': 'u',
            'confused': 'i',
            'excited': 'o',
            'sleepy': 'p'
        }
    
    def motions_callback(self, msg):
        gelen_duygu = msg.data.strip().lower()
        self.get_logger().info(f'/motions mesajı alındı: {gelen_duygu}')

        if self.ser and gelen_duygu in self.duygu_komutlari:
            komut = self.duygu_komutlari[gelen_duygu]
            self.ser.write(komut.encode())
            self.get_logger().info(f"Arduino'ya gönderilen komut: {komut}")
            ## istedigin topige burda publish edip istedigin texti ve duyguyu oynatabilirsin.
        else:
            self.get_logger().warn(f"Bilinmeyen duygu veya Arduino bağlı değil: {gelen_duygu}")

    def timer_callback(self):
        try:
            obj = self.arduino_read()
            self.arduino_publish(obj)
        except Exception as e:
            self.get_logger().info(f'{e}')

    def color_callback(self, color):
        clr = color.data
        if self.previous_msg != clr:
            self.get_logger().info(f'********* message is: {clr}')
            self.ser.write(clr.encode())
            self.previous_msg = clr

    def arduino_read(self):
        self.buffer_string += self.ser.read(self.ser.in_waiting).decode()
        if '\n' in self.buffer_string:
            lines = self.buffer_string.split('\n')
            self.last_received = lines[-2]
            self.buffer_string = lines[-1]
            return json.loads(self.last_received)
        return None

    def arduino_publish(self, obj):
        if obj:
            msg = Arduino()
            msg.emmergency_btn = obj['emmergency_btn']
            msg.motions_status = obj['motions_status']
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    arduino_control = ArduinoControl()
    rclpy.spin(arduino_control)
    arduino_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
