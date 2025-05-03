import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from langdetect import detect
import os
import librosa
import numpy as np
import pygame
import time
from orbit_command_msgs.srv import Records
import threading
import subprocess
from queue import Queue

class ControlPublisher(Node):
    def __init__(self, parent_node, name='control_publisher'):
        super().__init__(name)
        self.get_logger().info('**** Control_Publisher initialized ****')
        self.amp_pub = parent_node.create_publisher(Float32, 'decibel', 1)

    def publish_amp(self, amp):
        decibel = Float32()
        decibel.data = amp
        self.amp_pub.publish(decibel)

class LipSyncNode(Node):
    def __init__(self):
        super().__init__('lip_sync_node')
        self.get_logger().info('**** LipSyncNode initialized ****')
        self.publisher = ControlPublisher(self)

        self.srv = self.create_service(Records, 'record', self.record_callback)
        self.task_srv = self.create_service(Records, 'record_task', self.recordtask_callback_)

        self.audio_thread = None
        self.stop_flag = threading.Event()

        self.task_queue = Queue()
        self.task_thread = threading.Thread(target=self.task_worker)
        self.task_thread.daemon = True
        self.task_thread.start()

    def metni_sese_cevir(self, metin, dosya_adi="ses.mp3"):
        try:
            if "merhaba" in metin.lower():
                voice_lang = "tr-TR-EmelNeural"
            elif self.check_language(metin) == "tr":
                voice_lang = "tr-TR-EmelNeural"
            else:
                voice_lang = "en-US-AriaNeural"

            # cmd = f'edge-tts --text "{metin}" --voice "{voice_lang}" --write-media "{dosya_adi}"'
            cmd = f'/home/ovali/.local/bin/edge-tts --text "{metin}" --voice "{voice_lang}" --write-media "{dosya_adi}"'
            subprocess.run(cmd, shell=True, check=True)
            self.get_logger().info(f"{dosya_adi} kaydedildi!")
            return dosya_adi
        except Exception as e:
            self.get_logger().error(f"Ses oluşturulurken hata: {e}")
            return None

    def check_language(self, metin):
        try:
            return detect(metin)
        except:
            return "en"

    def record_callback(self, request, response):
        metin = request.records.strip().lower()
        self.get_logger().info(f"Gelen mesaj: {metin}")

        if metin == "dur":
            # Eğer ses çalıyorsa, hemen durdur
            if self.audio_thread and self.audio_thread.is_alive():
                self.get_logger().warn("Ses durduruluyor (dur komutu geldi).")
                self.stop_flag.set()
                self.audio_thread.join()
                self.get_logger().info("Ses durduruldu.")
            else:
                self.get_logger().info("Durdurulacak ses yok.")
            self.publisher.publish_amp(0.0)
            response.response = True
            return response

        if metin == "test":
            self.get_logger().info("Test mesajı alındı, ses oynatılmayacak.")
            response.response = True
            return response

        # Yeni bir ses başlatmadan önce önceki varsa durdur
        if self.audio_thread and self.audio_thread.is_alive():
            self.get_logger().warn("Önceki ses kesiliyor...")
            self.stop_flag.set()
            self.audio_thread.join()
            self.get_logger().info("Önceki ses durduruldu.")

        self.stop_flag.clear()

        self.audio_thread = threading.Thread(target=self.process_audio, args=(metin,))
        self.audio_thread.start()

        response.response = True
        return response

    def recordtask_callback_(self, request, response):
        metin = request.records
        done_event = threading.Event()

        self.task_queue.put((metin, done_event))
        self.get_logger().info("record_task kuyruğa eklendi")

        # İş tamamlanana kadar bekle
        done_event.wait()
        response.response = True
        return response

    def task_worker(self):
        while True:
            metin, done_event = self.task_queue.get()
            self.get_logger().info("record_task kuyruğundan alındı")
            self.stop_flag.clear()
            self.process_audio(metin)
            done_event.set()

    def process_audio(self, metin):
        mp3_file = None
        try:
            mp3_file = self.metni_sese_cevir(metin)
            if not mp3_file or not os.path.exists(mp3_file):
                self.get_logger().error("Ses dosyası oluşturulamadı.")
                return

            y, sr = librosa.load(mp3_file, sr=22050)
            frame_rate = 30
            frame_samples = sr // frame_rate

            amplitudes = [
                np.max(np.abs(y[i * frame_samples: (i + 1) * frame_samples]))
                for i in range(len(y) // frame_samples)
            ]

            min_amp, max_amp = min(amplitudes), max(amplitudes)

            pygame.mixer.init()
            pygame.mixer.music.load(mp3_file)
            pygame.mixer.music.play()

            start_time = time.time()

            for i, amp in enumerate(amplitudes):
                if self.stop_flag.is_set():
                    self.get_logger().warn("Ses işlemi durduruldu.")
                    break

                lip_height = float(10 + (amp - min_amp) / (max_amp - min_amp) * 110) if max_amp != min_amp else 10.0
                self.publisher.publish_amp(lip_height)
                self.get_logger().info(f"Lip Height: {lip_height}")

                elapsed_time = time.time() - start_time
                target_time = (i + 1) / frame_rate
                wait_time = max(0, target_time - elapsed_time)
                time.sleep(wait_time)

            pygame.mixer.music.stop()

        except Exception as e:
            self.get_logger().error(f"process_audio hatası: {e}")

        finally:
            pygame.mixer.quit()
            if mp3_file and os.path.exists(mp3_file):
                os.remove(mp3_file)

def main(args=None):
    rclpy.init(args=args)
    node = LipSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#[text_amp_publisher-12] free(): invalid pointer
#[ERROR] [text_amp_publisher-12]: process has died [pid 39033, exit code -6, cmd '/home/ovali/orbit_ws/install/play_face/lib/play_face/text_amp_publisher --ros-args'].
