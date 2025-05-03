import socket
import os
import json
import subprocess
import time

SAVE_PATH = "/home/ovali/begin_cmd/wifi_status.json"
HOST = "0.0.0.0"
PORT = 10000

STATIC_IP = "192.168.31.10/24"
GATEWAY_IP = "192.168.31.1"

def connect_to_wifi(ssid, password,ip,gateway,dns):
    print(f"📡 '{ssid}' ağına (gizli) bağlanılmaya çalışılıyor...")

    result = subprocess.run([
        "nmcli", "device", "wifi", "connect", ssid,
        "password", password,
        "hidden", "yes"
    ])

    if result.returncode == 0:
        print(f"✅ '{ssid}' ağına başarıyla bağlanıldı.")

        # Statik IP ayarlarını yap
        set_ip = subprocess.run([
            "nmcli", "connection", "modify", ssid,
            "ipv4.addresses", f'{ip}/24',
            "ipv4.gateway", gateway,
            "ipv4.dns", dns,
            "ipv4.method", "manual"
        ])

        if set_ip.returncode == 0:
            print(f"⚙️ Statik IP ayarlandı: {STATIC_IP}")

            # Bağlantıyı yeniden başlat
            subprocess.run(["nmcli", "connection", "down", ssid])
            subprocess.run(["nmcli", "connection", "up", ssid])

            time.sleep(3)  # Bağlantının yeniden başlatılması için bekle

            subprocess.run(["nmcli", "connection", "up", ssid])

            print("🔄 Ağ bağlantısı yeniden başlatıldı.")
            return True
        else:
            print(f"⚠️ IP ayarlanamadı:\n{set_ip.stderr.strip()}")
            return False
    else:
        print(f"❌ Bağlantı hatası:\n{result.stderr.strip()}")
        return False

def get_active_wifi_connection():
    try:
        # Aktif Wi-Fi bağlantısını cihaz üzerinden bul (örneğin wlan0 gibi)
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

def receive_forever():
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
                        success = connect_to_wifi(ssid, password,ip,gateway,dns)

                        if success:
                            print("🌐 Yeni Wi-Fi ağına geçildi. Hotspot bağlantısı kopabilir.")
                            continue  # IP değiştiyse bağlantı kesilir, döngüden çık
                    else:
                        print("⚠️ Hotspot moduna geçiliyor...")
                        # Hotspot moduna geç
                        subprocess.run(["nmcli", "device", "wifi", "hotspot", "ifname", "wlo1", "ssid", "Orbit", "password", "Nct.2525"])
                        
                except json.JSONDecodeError:
                    print("⚠️ JSON geçersiz.")

if __name__ == "__main__":
    receive_forever()
