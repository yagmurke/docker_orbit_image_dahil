import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class MotionSender(Node):
    def __init__(self):
        super().__init__('motion_sender_node')
        self.subscription = self.create_subscription(
            String,
            '/motions',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        try:
            self.arduino = serial.Serial('/dev/arduino', timeout=1.0)
            time.sleep(2)
            self.get_logger().info('Arduino bağlantısı kuruldu.')
        except Exception as e:
            self.get_logger().error(f'Arduinoya bağlanılamadı: {e}')
            self.arduino = None

        # Duygular ve karşılık gelen karakterler
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

    def listener_callback(self, msg):
        gelen_duygu = msg.data.strip().lower()
        self.get_logger().info(f'/motions mesajı alındı: {gelen_duygu}')

        if self.arduino and gelen_duygu in self.duygu_komutlari:
            komut = self.duygu_komutlari[gelen_duygu]
            self.arduino.write(komut.encode())
            self.get_logger().info(f"Arduino'ya gönderilen komut: {komut}")
            ## istedigin topige burda publish edip istedigin texti ve duyguyu oynatabilirsin.
        else:
            self.get_logger().warn(f"Bilinmeyen duygu veya Arduino bağlı değil: {gelen_duygu}")

def main(args=None):
    rclpy.init(args=args)
    node = MotionSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node kapatılıyor...')
    finally:
        if node.arduino:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
