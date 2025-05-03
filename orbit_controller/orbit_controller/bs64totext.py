import rclpy
from rclpy.node import Node
from orbit_command_msgs.srv import Converter
import base64
import io
from pydub import AudioSegment
import speech_recognition as sr

class ConverterService(Node):
    def __init__(self):
        super().__init__('converter_service')
        self.srv = self.create_service(Converter, 'converter_topic', self.converter_callback)
        self.get_logger().info("Converter servisi başlatıldı.")

    def converter_callback(self, request, response):
        try:
            self.get_logger().info("Gelen ses verisi çözümleniyor...")

            audio_bytes = base64.b64decode(request.converter)

            audio = AudioSegment.from_file(io.BytesIO(audio_bytes), format="mp3")
            wav_io = io.BytesIO()
            audio.export(wav_io, format="wav")
            wav_io.seek(0)

            recognizer = sr.Recognizer()
            with sr.AudioFile(wav_io) as source:
                audio_data = recognizer.record(source)
                text = recognizer.recognize_google(audio_data, language="tr-TR")
                self.get_logger().info(f"Çözümlenen Metin: {text}")

            response.response = text
        except Exception as e:
            self.get_logger().error(f"Hata oluştu: {str(e)}")
            response.response = ""

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ConverterService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
