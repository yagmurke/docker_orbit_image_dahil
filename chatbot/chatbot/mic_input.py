import os
import pyaudio
import queue
import threading
from google.cloud import speech
import time
import numpy as np
from collections import deque
import webrtcvad
import wave
import audioop
import array
import math
import json
import datetime

class AudioProcessor:
    def __init__(self, rate=16000):
        self.rate = rate
        self.vad = webrtcvad.Vad(2)  # Daha az agresif mod
        # 10, 20 veya 30ms frame süreleri için örneklem sayısı
        self.frame_duration_ms = 30
        self.frame_size = int(self.rate * self.frame_duration_ms / 1000)
        self.history_size = 30
        self.speech_buffer = deque(maxlen=self.history_size)
        self.energy_threshold = 0.3
        self.speech_start_threshold = 0.8
        self.speech_end_threshold = 0.2
        self.min_speech_duration = 0.3
        self.max_speech_duration = 15.0
        self.silence_duration = 0.8
        self.last_activity = time.time()

    def _pad_frame(self, frame):
        """Frame'i doğru boyuta getir"""
        if len(frame) < self.frame_size:
            # Küçük frame'i sıfırlarla doldur
            return frame + b'\x00' * (self.frame_size - len(frame))
        elif len(frame) > self.frame_size:
            # Büyük frame'i kes
            return frame[:self.frame_size]
        return frame

    def is_speech(self, audio_chunk):
        """Gelişmiş ve hata toleranslı konuşma algılama"""
        try:
            # Giriş verisi kontrolü
            if not audio_chunk:
                return 0.0

            # Ses verisini hazırla
            if isinstance(audio_chunk, (bytes, bytearray)):
                audio_data = audio_chunk
            else:
                try:
                    audio_data = bytes(audio_chunk)
                except:
                    return 0.0

            # Frame boyutunu kontrol et ve ayarla
            audio_data = self._pad_frame(audio_data)

            # VAD kontrolü - daha sağlam hata yönetimi
            try:
                frames = []
                offset = 0
                while offset + self.frame_size <= len(audio_data):
                    frames.append(audio_data[offset:offset + self.frame_size])
                    offset += self.frame_size

                vad_scores = []
                for frame in frames:
                    try:
                        if len(frame) == self.frame_size:  # Sadece doğru boyuttaki frame'leri işle
                            is_speech = self.vad.is_speech(frame, self.rate)
                            vad_scores.append(float(is_speech))
                    except Exception as e:
                        continue

                # En az bir geçerli VAD skoru varsa ortalamasını al
                vad_score = sum(vad_scores) / len(vad_scores) if vad_scores else 0.0

            except Exception as e:
                vad_score = 0.0

            # Enerji tabanlı kontrol
            try:
                rms = audioop.rms(audio_data, 2) / 32768.0
            except:
                rms = 0.0

            # Spektral analiz
            try:
                audio_array = np.frombuffer(audio_data, dtype=np.int16)
                fft = np.fft.fft(audio_array)
                freq = np.fft.fftfreq(len(audio_array))
                spectrum = np.abs(fft)

                # Frekans özellikleri - insan sesi aralığına odaklan
                speech_range = (85, 255)  # Yaklaşık 300-3400 Hz
                speech_freqs = np.logical_and(
                    freq >= speech_range[0] / len(audio_array) * self.rate,
                    freq <= speech_range[1] / len(audio_array) * self.rate
                )
                speech_energy = np.sum(spectrum[speech_freqs]) if any(speech_freqs) else 0
                total_energy = np.sum(spectrum)
                spectral_ratio = speech_energy / total_energy if total_energy > 0 else 0
            except:
                spectral_ratio = 0.0

            # Konuşma olasılığını hesapla - ağırlıkları ayarla
            vad_weight = 0.3  # VAD güvenilirliğini azalt
            energy_weight = 0.4  # Enerji ağırlığını artır
            spectral_weight = 0.3

            speech_probability = (
                vad_weight * vad_score +
                energy_weight * (rms > self.energy_threshold) +
                spectral_weight * (spectral_ratio > 0.4)
            )

            # Sonucu yumuşat
            self.speech_buffer.append(speech_probability)
            smoothed_probability = sum(self.speech_buffer) / len(self.speech_buffer)

            return smoothed_probability

        except Exception as e:
            # Ana hata durumunda sessizce devam et
            return 0.0

class MicrophoneStream:
    def __init__(self, rate=16000, chunk=1024):
        self._rate = rate
        self._chunk = chunk
        self._buff = queue.Queue()
        self._audio = None
        self._stream = None
        self.closed = True
        self.audio_processor = AudioProcessor(rate)
        
        # Ses seviyesi izleme - daha hassas eşik değerleri
        self.volume_threshold = 100  # Daha düşük eşik değeri (önceki: 500)
        self.is_speaking = False
        self.silence_frames = 0
        self.max_silence_frames = 45  # Sessizlik toleransını artır (önceki: 30)
        
    def _process_multichannel(self, in_data):
        """ReSpeaker çok kanallı veriyi işle"""
        try:
            # Veriyi 16-bit integer dizisine dönüştür
            audio_array = np.frombuffer(in_data, dtype=np.int16)
            
            # Çok kanallı veriyi yeniden şekillendir (6 kanal için)
            if len(audio_array) % 6 == 0:
                audio_array = audio_array.reshape((-1, 6))
                # İlk kanalı kullan (ana mikrofon)
                mono_channel = audio_array[:, 0]
                return mono_channel.tobytes()
            return in_data
        except Exception as e:
            print(f"Kanal işleme hatası (göz ardı ediliyor): {str(e)}")
            return in_data

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Gelişmiş ses tamponu doldurma"""
        try:
            # Çok kanallı veriyi işle
            processed_data = self._process_multichannel(in_data)
            
            # Ses seviyesini kontrol et
            audio_array = np.frombuffer(processed_data, dtype=np.int16)
            volume_norm = np.abs(audio_array).mean()
            
            # Konuşma durumunu güncelle
            if volume_norm > self.volume_threshold:
                if not self.is_speaking:
                    print("\r🎤 Ses algılandı...", end="", flush=True)
                self.is_speaking = True
                self.silence_frames = 0
            else:
                if self.is_speaking:
                    self.silence_frames += 1
                    if self.silence_frames >= self.max_silence_frames:
                        self.is_speaking = False
                        print("\r✓ Ses kaydı tamamlandı", end="", flush=True)
            
            # Veriyi tampona ekle
            self._buff.put(processed_data)
            return None, pyaudio.paContinue
            
        except Exception as e:
            print(f"\nTampon doldurma hatası: {str(e)}")
            return None, pyaudio.paAbort

    def __enter__(self):
        """Gelişmiş mikrofon başlatma"""
        try:
            self._audio = pyaudio.PyAudio()
            
            # ReSpeaker'ı ara
            respeaker_index = None
            for i in range(self._audio.get_device_count()):
                try:
                    dev_info = self._audio.get_device_info_by_index(i)
                    if 'ReSpeaker' in dev_info['name']:
                        respeaker_index = i
                        channels = int(dev_info['maxInputChannels'])
                        print(f"✓ ReSpeaker bulundu: {dev_info['name']} ({channels} kanal)")
                        break
                except:
                    continue
            
            # Stream yapılandırması
            stream_config = {
                'format': pyaudio.paInt16,
                'channels': 6 if respeaker_index is not None else 1,  # ReSpeaker için 6 kanal
                'rate': self._rate,
                'input': True,
                'frames_per_buffer': self._chunk,
                'stream_callback': self._fill_buffer
            }
            
            # ReSpeaker varsa kullan
            if respeaker_index is not None:
                stream_config['input_device_index'] = respeaker_index
            
            # Stream'i başlat
            self._stream = self._audio.open(**stream_config)
            
            if not self._stream.is_active():
                raise Exception("Stream başlatılamadı")
            
            self.closed = False
            print("✓ Ses yakalama başlatıldı")
            return self
            
        except Exception as e:
            print(f"❌ Mikrofon başlatma hatası: {str(e)}")
            if hasattr(self, '_audio') and self._audio:
                self._audio.terminate()
            raise e

    def __exit__(self, type, value, traceback):
        """Mikrofon kapatma"""
        if self._stream:
            self._stream.stop_stream()
            self._stream.close()
        if self._audio:
            self._audio.terminate()
        self.closed = True
        self._buff.put(None)

    def generator(self):
        """Ses verisi üretici"""
        while not self.closed:
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]
            
            # Kuyruktaki tüm verileri al
            while True:
                try:
                    chunk = self._buff.get_nowait()
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)

def listen_print_loop(responses, callback=None):
    """Gelişmiş konuşma tanıma döngüsü"""
    num_chars_printed = 0
    confidence_threshold = 0.65  # Güvenilirlik eşiğini düşürdüm
    max_silence_duration = 1.0
    last_transcript_time = time.time()
    current_transcript = ""
    low_confidence_count = 0
    max_low_confidence = 3  # Maksimum düşük güvenilirlik sayısı
    
    try:
        for response in responses:
            if not response.results:
                continue
                
            result = response.results[0]
            if not result.alternatives:
                continue
                
            transcript = result.alternatives[0].transcript
            confidence = result.alternatives[0].confidence if hasattr(result.alternatives[0], 'confidence') else 0.0
            
            if result.is_final:
                # Güven kontrolü
                if confidence >= confidence_threshold:
                    print(f"\r✓ {transcript.strip()} ({confidence:.2f})")
                    low_confidence_count = 0  # Başarılı tanıma ile sayacı sıfırla
                    if callback:
                        return callback(transcript)
                else:
                    print(f"\r⚠️ Düşük güvenilirlik ({confidence:.2f}): {transcript.strip()}")
                    low_confidence_count += 1
                    
                    # Düşük güvenilirlik limiti kontrolü - callback'in moduna göre davran
                    if low_confidence_count >= max_low_confidence:
                        if callback and hasattr(callback, '__name__') and callback.__name__ == 'process_passive_speech':
                            # Pasif modda ise sadece sayacı sıfırla ve devam et
                            low_confidence_count = 0
                        else:
                            # Aktif modda ise ve limit aşıldıysa uyar
                            print("\n⚠️ Çok fazla düşük güvenilirlik algılandı.")
                            if callback:
                                return None
                    
                num_chars_printed = 0
                current_transcript = ""
                
            else:
                # Ara transkript gösterimi
                current_transcript = transcript
                overwrite_chars = " " * (num_chars_printed - len(transcript))
                quality_indicator = "🟢" if confidence > 0.8 else "🟡" if confidence > 0.6 else "🔴"
                print(f"\r{quality_indicator} {transcript}{overwrite_chars}", end="")
                num_chars_printed = len(transcript) + 1
                
                # Sessizlik kontrolü
                if transcript != "":
                    last_transcript_time = time.time()
                elif time.time() - last_transcript_time > max_silence_duration:
                    if len(current_transcript.strip()) > 0:
                        print(f"\n⌛ Sessizlik algılandı, mevcut transkript işleniyor: {current_transcript.strip()}")
                        if callback:
                            return callback(current_transcript)
                    
    except Exception as e:
        print(f"\nKonuşma tanıma döngüsü hatası: {str(e)}")
        return None

def record_voice_stream(callback=None, language="tr-TR"):
    """Gelişmiş konuşma tanıma stream'i"""
    client = speech.SpeechClient()
    
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=16000,
        language_code=language,
        enable_automatic_punctuation=True,
        model="latest_long",
        use_enhanced=True,
        enable_word_confidence=True,
        enable_word_time_offsets=True,
        max_alternatives=1,
        profanity_filter=False,
        speech_contexts=[{
            "phrases": [
                "merhaba", "selam", "güle güle", "hoşçakal",
                "evet", "hayır", "tamam", "anladım",
                "nasılsın", "teşekkürler", "rica ederim",
                "saat kaç", "hava nasıl", "ne görüyorsun"
            ],
            "boost": 15.0
        }],
        metadata=speech.RecognitionMetadata(
            interaction_type=speech.RecognitionMetadata.InteractionType.VOICE_COMMAND,
            microphone_distance=speech.RecognitionMetadata.MicrophoneDistance.NEARFIELD,
            recording_device_type=speech.RecognitionMetadata.RecordingDeviceType.OTHER_INDOOR_DEVICE
        )
    )

    streaming_config = speech.StreamingRecognitionConfig(
        config=config,
        interim_results=True,
        single_utterance=False
    )

    with MicrophoneStream(16000, 1024) as stream:
        audio_generator = stream.generator()
        requests = (speech.StreamingRecognizeRequest(audio_content=content)
                   for content in audio_generator)

        try:
            responses = client.streaming_recognize(streaming_config, requests)
            return listen_print_loop(responses, callback)
        except Exception as e:
            print(f"\n❌ Stream hatası: {str(e)}")
            return None

def record_voice(callback=None):
    """Geriye dönük uyumluluk için wrapper fonksiyon"""
    return record_voice_stream(callback)

if __name__ == "__main__":
    print("🎯 Ses algılama testi başlatılıyor...")
    
    def process_text(text):
        print(f"✅ İşlenen metin: {text}")
        if "bitir" in text.lower():
            return "stop"
        return None
    
    result = record_voice_stream(process_text)
    print(f"Son sonuç: {result}")