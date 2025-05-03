import importlib.util
import subprocess
import sys
import os
import pygame
import io
import threading
import time
import queue
import numpy as np
from pydub import AudioSegment
from google.cloud import texttospeech

# ROS2 modülleri
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ROS2 başlat
rclpy.init()

class DecibelPublisher(Node):
    def __init__(self):
        super().__init__('decibel_publisher')
        self.publisher_ = self.create_publisher(Float32, '/decibel', 10)

    def publish_decibel(self, value):
        msg = Float32()
        msg.data = value
        self.publisher_.publish(msg)

ros_node = DecibelPublisher()

# Gerekli paketleri yükle
def install_and_import(package):
    try:
        importlib.import_module(package)
    except ImportError:
        print(f"{package} not found, installing...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", package])

install_and_import("numpy")
install_and_import("pydub")

# Google Cloud TTS istemcisi
client = texttospeech.TextToSpeechClient()

# pygame başlat
pygame.init()
pygame.mixer.init()

# TTS durum değişkenleri
speech_queue = queue.Queue()
is_speaking = False
tts_active = False
COOLDOWN_AFTER_TTS = 0.5
EARLY_LISTEN_START = 3.0
last_speech_end_time = 0
speech_duration = 0
speech_start_time = 0
speech_end_event = threading.Event()
thread_running = True

# Amplitüd serisini çıkart
def extract_amplitude_series(audio_bytes, frame_duration_ms=30):
    try:
        sound = AudioSegment.from_file(audio_bytes, format="mp3")
        amplitudes = []
        for i in range(0, len(sound), frame_duration_ms):
            frame = sound[i:i + frame_duration_ms]
            samples = np.array(frame.get_array_of_samples())

            if frame.channels == 2:
                samples = samples.reshape((-1, 2))
                samples = samples.mean(axis=1)

            samples = samples / np.iinfo(samples.dtype).max
            amp = np.mean(np.abs(samples))
            amplitudes.append(amp)

        return amplitudes, len(sound) / 1000.0  # saniye
    except Exception as e:
        print(f"Amplitüd çıkarma hatası: {str(e)}")
        return [], 0

# Amplitüd verisini senkronize olarak hem görselle hem ROS2'ye gönder
def sync_amplitude_visuals(amplitudes, interval=0.03):
    def run():
        max_amp = max(amplitudes) if amplitudes else 1.0

        for amp in amplitudes:
            normalized = amp / max_amp
            scaled_amp = float(normalized * 120)
            print(f"Anlık amplitüd: {scaled_amp:.2f}")
            ros_node.publish_decibel(scaled_amp)
            time.sleep(interval)
    threading.Thread(target=run, daemon=True).start()

def speak_worker():
    """Kuyrukta bekleyen konuşma parçalarını sürekli çalar ve amplitüd verilerini işler"""
    global is_speaking, tts_active, last_speech_end_time, speech_duration, speech_start_time
    
    while thread_running:
        try:
            try:
                audio_data, amplitudes = speech_queue.get(timeout=0.5)
            except queue.Empty:
                if tts_active and not is_speaking and time.time() - last_speech_end_time > COOLDOWN_AFTER_TTS:
                    tts_active = False
                    print("\r✅ Dinleme aktif edildi        ", end="", flush=True)
                continue
                
            if audio_data is None:
                speech_queue.task_done()
                break
                
            is_speaking = True
            tts_active = True
            speech_start_time = time.time()
            print("\r🔊 Asistan konuşuyor - dinleme devre dışı", end="", flush=True)
            
            try:
                pygame.mixer.music.load(audio_data)
                audio_data.seek(0)
                
                try:
                    temp_audio_file = "temp_audio.mp3"
                    with open(temp_audio_file, 'wb') as f:
                        f.write(audio_data.getvalue())
                    
                    sound = pygame.mixer.Sound(temp_audio_file)
                    speech_duration = sound.get_length()
                    
                    try:
                        os.remove(temp_audio_file)
                    except:
                        pass
                except:
                    speech_duration = 2.0
                
                pygame.mixer.music.play()
                
                # Amplitüd verilerini senkronize et
                sync_amplitude_visuals(amplitudes)
                
                start_time = time.time()
                while pygame.mixer.music.get_busy() and thread_running:
                    current_time = time.time()
                    elapsed_time = current_time - start_time
                    
                    remaining_time = speech_duration - elapsed_time
                    if remaining_time <= EARLY_LISTEN_START:
                        if tts_active:
                            tts_active = False
                            print(f"\r🔊→🎤 Dinlemeye erken başlanıyor ({EARLY_LISTEN_START}s)", end="", flush=True)
                    
                    time.sleep(0.05)
                    
            except Exception as e:
                print(f"\nSes çalma sırasında hata: {str(e)}")
            
            is_speaking = False
            last_speech_end_time = time.time()
            print("\r✓ Konuşma tamamlandı                 ", end="", flush=True)
            speech_queue.task_done()
            
            if tts_active and COOLDOWN_AFTER_TTS > 0:
                time.sleep(COOLDOWN_AFTER_TTS)
                tts_active = False
                print("\r✅ Dinleme aktif edildi               ", end="", flush=True)
            
        except Exception as e:
            print(f"\nSes çalma hatası (speak_worker): {str(e)}")
            is_speaking = False
            tts_active = False
            last_speech_end_time = time.time()
            if not speech_queue.empty():
                speech_queue.task_done()

# Konuşma thread'i başlat
speech_thread = threading.Thread(target=speak_worker, daemon=True)
speech_thread.start()

def is_tts_active():
    """TTS'in aktif olup olmadığını kontrol eder"""
    return tts_active

def speak(text, lang="tr-TR", voice_name="tr-TR-Standard-C"):
    """Metni TTS ile sese çevirir ve kuyruğa ekler"""
    global tts_active
    
    try:
        if len(text) < 5 and not text.endswith(('.', '!', '?')):
            if not speech_queue.empty() or is_speaking:
                return
        
        tts_active = True
        
        synthesis_input = texttospeech.SynthesisInput(text=text)

        voice = texttospeech.VoiceSelectionParams(
            language_code=lang,
            name=voice_name,
            ssml_gender=texttospeech.SsmlVoiceGender.FEMALE
        )

        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.MP3
        )

        response = client.synthesize_speech(
            input=synthesis_input, voice=voice, audio_config=audio_config
        )

        audio_bytes = io.BytesIO(response.audio_content)
        
        # Amplitüd verilerini çıkar
        amplitudes, duration = extract_amplitude_series(io.BytesIO(response.audio_content))
        print(f"{len(amplitudes)} adet amplitüd değeri çıkarıldı. Süre: {duration:.2f}s")
        
        # Kuyruğa ses ve amplitüd verisini birlikte koy
        speech_queue.put((audio_bytes, amplitudes))

    except Exception as e:
        print(f"TTS hatası: {str(e)}")

def wait_for_speech_to_complete():
    """Tüm konuşma parçaları ve cooldown süresi bitene kadar bekler"""
    try:
        speech_queue.join()
        while is_speaking or pygame.mixer.music.get_busy():
            time.sleep(0.1)
            
        cooldown_start = time.time()
        while time.time() - cooldown_start < COOLDOWN_AFTER_TTS:
            time.sleep(0.1)
            
    except Exception as e:
        print(f"Konuşma tamamlanma kontrolü sırasında hata: {str(e)}")

def cleanup_speech_system():
    """Ses sistemini ve ROS2 nodunu temiz bir şekilde kapatır"""
    global thread_running, tts_active, is_speaking
    
    try:
        print("TTS sistemi kapatılıyor...")
        thread_running = False
        tts_active = False
        is_speaking = False
        
        while not speech_queue.empty():
            try:
                speech_queue.get_nowait()
                speech_queue.task_done()
            except:
                pass
        
        speech_queue.put((None, None))
        
        if pygame.mixer.music.get_busy():
            pygame.mixer.music.stop()
        
        if speech_thread.is_alive():
            speech_thread.join(timeout=1.0)
            
        pygame.mixer.quit()
        
        # ROS2 kapat
        ros_node.destroy_node()
        rclpy.shutdown()
        
        print("TTS sistemi başarıyla kapatıldı.")
    except Exception as e:
        print(f"TTS sistemi kapatılırken hata: {str(e)}")

# Örnek kullanım
if __name__ == "__main__":
    speak("AMR, bulunduğu ortamda kendi başına gezinme, engellerden kaçınma ve belirli görevleri yerine getirme yeteneğine sahip mobil robotlardır. En önemli özellikleri, harici bir kontrol sistemine ihtiyaç duymadan çevrelerini algılayarak karar verebilmeleridir.")
    wait_for_speech_to_complete()
    cleanup_speech_system()
