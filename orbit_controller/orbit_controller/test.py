import rclpy
from rclpy.node import Node
from orbit_command_msgs.srv import Converter
import base64
from gtts import gTTS
import os

class ConverterClient(Node):
    def __init__(self):
        super().__init__('converter_client')
        self.cli = self.create_client(Converter, 'converter_topic')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servis bekleniyor...')

        self.req = Converter.Request()
        self.send_request()

    def send_request(self):
        # Çalışma dizinini bul
        current_dir = "/home/ovali/orbit_ws/src/orbit_controller/orbit_controller"
        mp3_path = os.path.join(current_dir, "merhaba.mp3")

        # "Merhaba" diyen bir mp3 dosyası oluştur
        tts = gTTS(text="Bugün size nasıl yardımcı olabilirim?", lang="tr")
        tts.save(mp3_path)
        self.get_logger().info(f'MP3 dosyası oluşturuldu: {mp3_path}')

        # Dosyayı base64'e çevir
        with open(mp3_path, "rb") as audio_file:
            audio_bytes = audio_file.read()
            audio_b64 = base64.b64encode(audio_bytes).decode('utf-8')

        # Servis isteğine ata
        self.get_logger().info(f'Base64 formatında ses verisi gönderiliyor...{audio_b64}')
        self.req.converter = audio_b64

        # Servise isteği gönder
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.response:
                self.get_logger().info('Servis başarılı şekilde ses verisini çözdü.')
            else:
                self.get_logger().error('Servis isteği başarısız oldu.')
        except Exception as e:
            self.get_logger().error(f'Servis çağrısı sırasında hata: {e}')


def main(args=None):
    rclpy.init(args=args)
    client = ConverterClient()
    rclpy.spin_once(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
