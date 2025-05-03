import json
import subprocess
import os
from ament_index_python import get_package_share_directory

CONFIG = os.path.join(get_package_share_directory('control_panel'),'config')
# JSON dosyasından veriyi okuma
def load_config(file_path):
    with open(file_path, 'r') as f:
        config = json.load(f)
    return config

# Wi-Fi'ye bağlanmak ve statik IP ayarlarını yapmak için nmcli komutlarını çalıştırma
def set_static_ip(config):
    ssid = config["name"]
    password = config["password"]
    ip_address = config["ip"]
    dns = config["dns"]
    gateway = config["gateway"]

    # Wi-Fi'ye bağlanma
    print(f"Bağlanılıyor: {ssid}...")
    subprocess.run(f"nmcli dev wifi connect \"{ssid}\" password \"{password}\"", shell=True)

    # Bağlantı profilini yapılandırma
    print(f"Statik IP ayarlanıyor: {ip_address}...")
    subprocess.run(f"nmcli con mod \"{ssid}\" ipv4.addresses {ip_address}/24", shell=True)
    subprocess.run(f"nmcli con mod \"{ssid}\" ipv4.gateway {gateway}", shell=True)
    subprocess.run(f"nmcli con mod \"{ssid}\" ipv4.dns \"{dns}\"", shell=True)
    subprocess.run(f"nmcli con mod \"{ssid}\" ipv4.method manual", shell=True)

    # Yapılandırmayı uygula
    print("Yapılandırma uygulanıyor...")
    subprocess.run(f"nmcli con up \"{ssid}\"", shell=True)

    print("Bağlantı başarıyla kuruldu ve yapılandırmalar uygulandı.")

if __name__ == "__main__":
    mode_json = os.path.join(CONFIG, 'wifi_status.json')
    config = load_config(mode_json)
    
    # Statik IP yapılandırmasını yap
    set_static_ip(config)
