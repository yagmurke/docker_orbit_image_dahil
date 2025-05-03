import subprocess
import time
import socket
import json
import rclpy
from rclpy.node import Node
from arduino_msgs.msg import WebSocket

HOTSPOT_SSID = "Orbit"
HOTSPOT_PASSWORD = "Nct.2525"
INTERFACE_NAME = "wlo1"
CHECK_INTERVAL = 10  # kaç saniyede bir kontrol edilecek
RETRY_THRESHOLD = 10  # 10 saniye boyunca bağlı değilse hotspot yeniden denenecek

def update_wifi_status_to_hotspot():
    json_path = "/home/ovali/begin_cmd/wifi_status.json"
    try:
        with open(json_path, "r") as file:
            data = json.load(file)
        
        data["wifi_mode"] = "hotspot"
        data["name"] = "Orbit"

        with open(json_path, "w") as file:
            json.dump(data, file, indent=4)
        
        print("[INFO] wifi_status.json dosyası güncellendi (wifi_mode = hotspot).")

    except Exception as e:
        print(f"[ERROR] wifi_status.json dosyasını güncellerken hata oluştu: {e}")

def is_wifi_connected():
    try:
        result = subprocess.check_output(
            ["nmcli", "-t", "-f", "DEVICE,TYPE,STATE", "device"], text=True
        )
        for line in result.strip().split("\n"):
            parts = line.split(":")
            if len(parts) >= 3:
                device, dev_type, state = parts
                if dev_type == "wifi" and state == "connected":
                    return True
    except subprocess.CalledProcessError:
        pass
    return False

def has_internet_access():
    try:
        subprocess.check_call(
            ["ping", "-c", "1", "-W", "2", "8.8.8.8"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return True
    except subprocess.CalledProcessError:
        return False

def connect_to_hotspot():
    print("[ACTION] Hotspot bağlantısı deneniyor...")
    result = subprocess.run([
        "nmcli", "device", "wifi", "hotspot",
        "ifname", INTERFACE_NAME,
        "ssid", HOTSPOT_SSID,
        "password", HOTSPOT_PASSWORD
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if result.returncode == 0:
        print("[SUCCESS] Hotspot bağlantısı kuruldu.")
        update_wifi_status_to_hotspot()
        return True
    else:
        print("[FAIL] Hotspot bağlantısı başarısız.")
        return False

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

class WebSocketPublisher(Node):
    def __init__(self):
        super().__init__('wifi_watchdog_node')
        self.publisher_ = self.create_publisher(WebSocket, 'websocket_status', 10)

    def publish_status(self, status_bool):
        msg = WebSocket()
        msg.status = status_bool
        msg.ip = get_local_ip()
        msg.name = get_wifi_ssid()
        self.publisher_.publish(msg)
        self.get_logger().info(f"[ROS] 📡 Gönderildi: status={msg.status}, ip={msg.ip}, name={msg.name}")

def main():
    rclpy.init()
    node = WebSocketPublisher()

    previously_connected = True
    hotspot_tried = False
    disconnected_since = None

    TRY_BOOL = 5  # Maksimum tekrar deneme sayısı
    retry_count = 0  # Deneme sayacı

    while rclpy.ok():
        internet = has_internet_access()
        wifi = is_wifi_connected()

        if internet:
            if not previously_connected:
                print("[INFO] İnternet geri geldi.")
                # node.publish_status(True)
            previously_connected = True
            hotspot_tried = False
            disconnected_since = None
            retry_count = 0  # İnternet geldiğinde sayaç sıfırlanır

        else:
            if previously_connected:
                print("[WARNING] İnternet bağlantısı KAYBEDİLDİ.")
                # node.publish_status(False)

            previously_connected = False

            if not wifi:
                if disconnected_since is None:
                    disconnected_since = time.time()
                    print("[INFO] Wi-Fi bağlantısı da yok, süre sayacı başlatıldı...")
                elif time.time() - disconnected_since >= RETRY_THRESHOLD:
                    print("[ACTION] Wi-Fi ve internet 10 saniyedir yok, hotspot yeniden kuruluyor...")
                    connect_to_hotspot()
                    node.publish_status(True)
                    disconnected_since = time.time()
                else:
                    print(f"[INFO] Hotspot daha önce denendi, {int(time.time() - disconnected_since)} saniyedir bekleniyor...")
            else:
                if retry_count < TRY_BOOL:
                    print(f"[INFO] Wi-Fi bağlı ama internet yok, sadece bekleniyor...{retry_count}")
                    retry_count += 1
                elif retry_count == TRY_BOOL:
                    print("[INFO] Maksimum deneme sayısına ulaşıldı. Daha fazla uyarı verilmeyecek.")
                    retry_count += 1  # Bu sayede bir daha bu bloğa girmez

                disconnected_since = None

        time.sleep(CHECK_INTERVAL)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
