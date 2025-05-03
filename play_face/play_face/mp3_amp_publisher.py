import os
import cv2
import librosa
import numpy as np
import pygame
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32, Bool
from orbit_command_msgs.srv import Mpname
import threading
from ament_index_python.packages import get_package_share_directory

AUDIO_FOLDER = os.path.join(get_package_share_directory('play_face'), 'resource','audios')

class Decibel_Control(Node):
    def __init__(self, name='decibel_control'):
        super().__init__(name)
        self.get_logger().info('**** Decibel_Control initialized ****')
        self.amp_pub = self.create_publisher(Float32, 'decibel', 10)
        self.audio_status_publisher = self.create_publisher(Bool, 'audio_mode', 10)

    def publish_amp(self, amp):
        decibel = Float32()
        decibel.data = amp
        self.amp_pub.publish(decibel)
    
    def publish_audio_status(self, status:bool):
        status_msg = Bool()
        status_msg.data = status
        self.audio_status_publisher.publish(status_msg)

class MP3_AMP(Node):
    def __init__(self):
        super().__init__('lip_sync_node')
        self.get_logger().info('**** MP3_AMP initialized ****')
        self.publisher = Decibel_Control()
        
        self.srv = self.create_service(
            Mpname,
            'mpname',
            self.mp3_start)

    
    def mp3_start(self, request, response):
        mp3_name = request.mpname

        # İşlemi arka planda başlat
        thread = threading.Thread(target=self.mp3_started_sync, args=(mp3_name,))
        thread.start()

        response.response = True
        return response
    
    def mp3_started_sync(self, mp3_name):
        """mp3_started fonksiyonunu senkron olarak çalıştırmak için"""
        self.mp3_started(mp3_name)

    def mp3_started(self, mp3_name):
        if mp3_name == 'greeting':
            self.publisher.publish_audio_status(True)
        mp3_filename = mp3_name
        mp3_file = os.path.join(AUDIO_FOLDER, f'{mp3_filename}.mp3')

        if not os.path.exists(mp3_file):
            self.get_logger().info(f"{mp3_file} bulunamadı. Lütfen geçerli bir dosya adı girin.")
            return

        # 1️⃣ MP3 Dosyasını Yükle
        y, sr = librosa.load(mp3_file, sr=22050)

        frame_rate = 30 
        frame_samples = sr // frame_rate 

        amplitudes = [
            np.max(np.abs(y[i * frame_samples: (i + 1) * frame_samples]))
            for i in range(len(y) // frame_samples)
        ]
        print(len(amplitudes))  
        # 3️⃣ Dudak Açma-Kapama Değerlerini Belirle
        min_amp, max_amp = min(amplitudes), max(amplitudes)

        pygame.mixer.init()
        pygame.mixer.music.load(mp3_file)
        pygame.mixer.music.play()

        start_time = time.time()  # Başlangıç zamanını kaydet
        preveus_time = time.time()

        for i, amp in enumerate(amplitudes):

            # Dudak Açıklığını Hesapla
            lip_height = float(10 + (amp - min_amp) / (max_amp - min_amp) * 110)
            if mp3_filename != 'camera_voice':
                self.publisher.publish_amp(lip_height)
            
            
            # Senkronizasyon İçin Bekleme
            elapsed_time = time.time() - start_time
            target_time = (i + 1) / frame_rate
            wait_time = max(0, target_time - elapsed_time)
            time.sleep(wait_time)
        if mp3_filename == 'greeting':
            self.publisher.publish_audio_status(False)


def main(args=None):
    rclpy.init(args=args)
    lip_sync_node = MP3_AMP()
    rclpy.spin(lip_sync_node)
    lip_sync_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
