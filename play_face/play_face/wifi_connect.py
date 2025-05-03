import socket
import os
import json
import subprocess
import time
import threading
import signal

import rclpy
from rclpy.node import Node
from arduino_msgs.msg import WebSocket  # Mesaj tipi

SAVE_PATH = "/home/ovali/begin_cmd/wifi_status.json"
HOST = "0.0.0.0"
PORT = 5000

# === IP ve SSID Bilgileri ===
def get_local_ip():
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
    try:
        result = subprocess.run(['nmcli', '-t', '-f', 'active,ssid', 'device', 'wifi'], stdout=subprocess.PIPE)
        wifi_info = result.stdout.decode('utf-8').strip().split('\n')
        for info in wifi_info:
            if info.startswith('yes'):
                return info.split(':')[1]
        return "unknown"
    except Exception:
        return "unknown"

# === ROS 2 WebSocket Publisher Node ===
class WebSocketPublisher(Node):
    def __init__(self):
        super().__init__('websocket_status_publisher')
        self.publisher_ = self.create_publisher(WebSocket, 'websocket_status', 10)

    def publish_status(self, status_bool):
        msg = WebSocket()
        msg.status = status_bool
        msg.ip = get_local_ip()
        msg.name = get_wifi_ssid()
        self.publisher_.publish(msg)
        self.get_logger().info(f"[ROS] 📡 Gönderildi: status={msg.status}, ip={msg.ip}, name={msg.name}")

# === WebSocket Manager ===
class WebSocketManager:
    def __init__(self):
        self.process = None

    def start_websocket(self):
        if self.process is None:
            print("[SYSTEM] WebSocket başlatılıyor...")
            self.process = subprocess.Popen(
                ['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )

    def stop_websocket(self):
        if self.process:
            print("[SYSTEM] WebSocket durduruluyor (SIGINT gönderiliyor)...")
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("[WARNING] WebSocket kapanmadı, SIGKILL gönderiliyor...")
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except Exception as e:
                print(f"[ERROR] WebSocket durdurulurken hata: {e}")
            self.process = None

    def restart_websocket(self):
        print("[ACTION] WebSocket yeniden başlatılıyor...")
        self.stop_websocket()
        time.sleep(1)
        self.start_websocket()

# === WiFi İşlemleri ===
def connect_to_wifi(ssid, password, ip, gateway, dns):
    print(f"📡 '{ssid}' ağına (gizli) bağlanılmaya çalışılıyor...")
    result = subprocess.run([
        "nmcli", "device", "wifi", "connect", ssid,
        "password", password,
        "hidden", "yes"
    ])
    if result.returncode == 0:
        print(f"✅ '{ssid}' ağına başarıyla bağlanıldı.")
        subprocess.run(["nmcli", "connection", "down", ssid])
        subprocess.run(["nmcli", "connection", "up", ssid])
        print("🔄 Ağ bağlantısı yeniden başlatıldı.")
        return True
    else:
        print(f"❌ Bağlantı hatası:")
        return False

def get_active_wifi_connection():
    try:
        result = subprocess.check_output(
            ["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device"],
            text=True
        )
        for line in result.strip().split("\n"):
            parts = line.split(":")
            if len(parts) >= 4:
                device, dev_type, state, connection = parts
                if dev_type == "wifi" and state == "connected":
                    return connection
    except subprocess.CalledProcessError as e:
        print("Wi-Fi cihazı üzerinden bağlantı alınamadı:", e)
    return None

def disconnect_wifi(connection_name):
    try:
        subprocess.run(["nmcli", "connection", "down", connection_name], check=True)
        print(f"{connection_name} bağlantısı kapatıldı.")
    except subprocess.CalledProcessError as e:
        print(f"{connection_name} bağlantısı kapatılamadı:", e)

# === TCP Sunucusu ===
def receive_forever(ws_manager, ros_node):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(5)
        print(f"🚀 Hotspot üzerinden dinleniyor (PORT {PORT})...")

        while True:
            conn, addr = s.accept()
            with conn:
                print(f"📥 Bağlantı alındı: {addr}")

                size_data = conn.recv(4)
                if not size_data:
                    continue
                total_size = int.from_bytes(size_data, 'big')

                data = b''
                while len(data) < total_size:
                    packet = conn.recv(4096)
                    if not packet:
                        break
                    data += packet

                try:
                    json_data = json.loads(data.decode('utf-8'))
                    os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
                    with open(SAVE_PATH, "w") as f:
                        json.dump(json_data, f, indent=4)
                    print(f"💾 JSON kaydedildi: {SAVE_PATH}")

                    wifi_mode = json_data.get("wifi_mode")
                    ssid = json_data.get("name")
                    password = json_data.get("password")
                    ip = json_data.get("ip")
                    gateway = json_data.get("gateway")
                    dns = json_data.get("dns")

                    current_wifi = get_active_wifi_connection()
                    if current_wifi:
                        print(f"Mevcut bağlantı: {current_wifi}")
                        disconnect_wifi(current_wifi)
                    else:
                        print("Aktif Wi-Fi bağlantısı bulunamadı, doğrudan hedefe bağlanılıyor...")

                    time.sleep(2)
                    if ssid and password and wifi_mode == "wifi":
                        success = connect_to_wifi(ssid, password, ip, gateway, dns)
                        if success:
                            print("🌐 Yeni Wi-Fi ağına geçildi. WebSocket yeniden başlatılıyor.")
                            ws_manager.restart_websocket()
                            ros_node.publish_status(True)
                            continue
                    else:
                        print("⚠️ Hotspot moduna geçiliyor...")
                        subprocess.run([
                            "nmcli", "device", "wifi", "hotspot",
                            "ifname", "wlo1",
                            "ssid", "Orbit",
                            "password", "Nct.2525"
                        ])
                        print("🔁 Hotspot kuruldu. WebSocket yeniden başlatılıyor.")
                        ws_manager.restart_websocket()
                        ros_node.publish_status(True)

                except json.JSONDecodeError:
                    print("⚠️ JSON geçersiz.")

# === Ana Çalışma ===
def ros_spin(node):
    rclpy.spin(node)

if __name__ == "__main__":
    rclpy.init()
    ros_node = WebSocketPublisher()

    ros_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    ws_manager = WebSocketManager()
    ws_manager.start_websocket()

    try:
        receive_forever(ws_manager, ros_node)
    except KeyboardInterrupt:
        print("\n[EXIT] Program sonlandırılıyor...")
        ws_manager.stop_websocket()
        rclpy.shutdown()
