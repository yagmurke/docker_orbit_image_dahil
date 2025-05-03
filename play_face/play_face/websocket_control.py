import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from arduino_msgs.msg import WebSocket
import socket
import time
import subprocess

WEBSOCKET_PORT = 9090

def get_local_ip():
    """Yerel IP adresini döndürür."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "10.42.0.1"
    finally:
        s.close()
    return ip

def get_wifi_ssid():
    """Ubuntu'da bağlı olunan Wi-Fi ağının SSID'sini döndürür."""
    try:
        result = subprocess.run(['nmcli', '-t', '-f', 'active,ssid', 'device', 'wifi'], stdout=subprocess.PIPE)
        wifi_info = result.stdout.decode('utf-8').strip().split('\n')
        
        # Aktif bağlantıyı bul ve SSID'yi döndür
        for info in wifi_info:
            if info.startswith('yes'):
                return info.split(':')[1]
        return "unknown"
    except Exception as e:
        return "unknown"

class WebsocketStatusNode(Node):
    def __init__(self):
        super().__init__('websocket_status_node')
        
        self.subscription = self.create_subscription(
            Int32,
            'client_count',
            self.listener_callback,
            10
        )
        self.subscription  # gereksiz uyarıyı önler

        self.publisher_ = self.create_publisher(WebSocket, 'websocket_status', 10)

        self.conn_count = 0
        self.ssid = get_wifi_ssid()
        self.local_ip = get_local_ip()
        
        # self.get_logger().info(f"🌐 Websocket'e bağlanmak için IP: ws://{self.local_ip}:{WEBSOCKET_PORT}")
        # self.publish_status(True,self.ssid)

    def publish_status(self, status_bool,ssid):
        msg = WebSocket()
        msg.status = status_bool
        msg.ip = self.local_ip
        msg.name = ssid
        self.publisher_.publish(msg)
        self.get_logger().info(f"📡 WebSocket mesajı gönderildi: status={msg.status}, ip={msg.ip}, name={msg.name}")

    def listener_callback(self, msg):
        self.conn_count = msg.data
        self.ssid = get_wifi_ssid()
        self.local_ip = get_local_ip()

        if self.conn_count > 0:
            self.get_logger().info(f"🔌 Bağlı istemci sayısı: {self.conn_count}")
            self.publish_status(False,self.ssid)
        else:
            self.get_logger().info("❌ Hiçbir istemci bağlı değil.")
            self.get_logger().info(f"🌐 Websocket'e bağlanmak için IP: ws://{self.local_ip}:{WEBSOCKET_PORT}")
            
            # IP bilgisi yazdırıldıktan sonra status=True ve ardından status=False yayınla
            self.publish_status(True,self.ssid)
            

def main(args=None):
    rclpy.init(args=args)
    node = WebsocketStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
