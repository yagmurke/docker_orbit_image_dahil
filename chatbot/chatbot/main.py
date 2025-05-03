import os
import platform
import sys
import dotenv
import google.generativeai as genai
from mic_input import record_voice, record_voice_stream
import time
import re
import json
import datetime
from camera_capture import capture_image_from_ros
from time_utils import get_turkey_time, get_turkey_date_time, get_time_reply
from weather_utils import get_weather_reply, extract_location
from google.cloud import speech_v1
import pyaudio
import queue
from tts_google_ros import speak, wait_for_speech_to_complete, cleanup_speech_system, is_tts_active

# Global değişkenler
chat = None
last_api_call = 0
API_COOLDOWN = 1.0  # API çağrıları arası minimum süre (saniye)

def respond(text):
    """Normal metin sorularını yanıtlar"""
    global chat, last_api_call
    
    try:
        print(f"\n🤖 Yanıtlanıyor: {text}")
        add_message_to_history("user", text)
        
        # API hız sınırı kontrolü
        current_time = time.time()
        if current_time - last_api_call < API_COOLDOWN:
            time.sleep(API_COOLDOWN - (current_time - last_api_call))
        
        # API yanıt denemesi
        max_retries = 3
        attempt = 0
        response_text = None
        
        while attempt < max_retries and not response_text:
            try:
                # API isteği gönder
                response = chat.send_message(text, stream=False)
                if response and hasattr(response, 'text'):
                    response_text = response.text.strip()
                    last_api_call = time.time()  # API çağrı zamanını güncelle
                    break
            except Exception as e:
                attempt += 1
                print(f"API Denemesi {attempt}: {str(e)}")
                if attempt < max_retries:
                    time.sleep(1)  # Yeniden denemeden önce bekle
        
        if not response_text:
            raise Exception("API'den yanıt alınamadı")
        
        # Yanıtı işle ve seslendir
        response_text = re.sub(r'\n+', ' ', response_text)
        if len(response_text) > 200:
            response_text = response_text[:200] + "..."
        
        print(f"🤖 API Yanıtı: {response_text}")
        speak(response_text)
        wait_for_speech_to_complete()
        
        add_message_to_history("assistant", response_text)
        return response_text
        
    except Exception as e:
        error_msg = f"API Hatası: {str(e)}"
        print(f"❌ {error_msg}")
        speak("Üzgünüm, bir hata oluştu. Tekrar dener misiniz?")
        wait_for_speech_to_complete()
        add_message_to_history("system", error_msg)
        return None

def respond_with_image(text, image_base64):
    """Görüntü ve metin içeren yanıtlar için"""
    global chat, last_api_call
    
    try:
        print(f"\n🤖 Görüntü ile yanıtlanıyor: {text}")
        add_message_to_history("user", text + " (görüntü ile)")
        
        # API hız sınırı kontrolü
        current_time = time.time()
        if current_time - last_api_call < API_COOLDOWN:
            time.sleep(API_COOLDOWN - (current_time - last_api_call))
        
        # Görüntüyü API'ye gönder - snake_case ve doğru format ile
        message = chat.send_message({
            "parts": [
                {"text": text},
                {
                    "inline_data": {
                        "mime_type": "image/jpeg",
                        "data": image_base64
                    }
                }
            ]
        }, stream=False)
        
        if message and hasattr(message, 'text'):
            response_text = message.text.strip()
            last_api_call = time.time()
            
            # Yanıtı işle ve seslendir
            response_text = re.sub(r'\n+', ' ', response_text)
            if len(response_text) > 200:
                response_text = response_text[:200] + "..."
            
            print(f"🤖 API Yanıtı: {response_text}")
            speak(response_text)
            wait_for_speech_to_complete()
            
            add_message_to_history("assistant", response_text + " (görüntü yanıtı)")
            return response_text
        else:
            raise Exception("API'den yanıt alınamadı")
            
    except Exception as e:
        error_msg = f"Görüntü API Hatası: {str(e)}"
        print(f"❌ {error_msg}")
        speak("Üzgünüm, görüntüyü işlerken bir hata oluştu.")
        wait_for_speech_to_complete()
        add_message_to_history("system", error_msg)
        return None

def passive_listening():
    """Pasif mod: Sürekli dinleme ile 'merhaba' komutunu bekle"""
    def process_passive_speech(text):
        if not text or not text.strip():
            return None
            
        text = text.strip().lower()
        
        # Aktivasyon ifadelerini kontrol et
        if any(phrase in text for phrase in ["merhaba"]):
            print("\n👋 Selamlama algılandı!")
            speak("Merhaba! Size nasıl yardımcı olabilirim?")
            wait_for_speech_to_complete()
            return "active"
        return None
    
    print("😴 Pasif modda bekleniyor. 'Merhaba' diyerek aktif moda geçebilirsiniz.")
    while True:
        result = record_voice_stream(process_passive_speech)
        if result == "active":
            return "active"

def active_listening():
    """Aktif mod: Sürekli dinleme ve gerçek zamanlı sesli asistan"""
    def process_active_speech(text):
        if not text or not text.strip():
            return None
            
        if is_tts_active():
            return None
            
        text = text.strip().lower()
        
        if text in ["kapat.", "güle güle.", "hoşçakal.", "uygulamayı kapat."]:
            return "exit"
        elif "pasif mod" in text or "bekleme mod" in text:
            return "passive"
            
        # Kamera komutları kontrolü
        
        if any(keyword in text for keyword in ["kamera", "fotoğraf", "resim","görüntü"]):
            try:
                img_base64, _ = capture_image_from_ros()
                
                if img_base64:
                    # Modele görüntü ve metni birlikte gönder
                    respond_with_image(text, img_base64)
                else:
                    # Kamera hatası durumunda - hata mesajını sadece konuş
                    print("❌ Kamera hatası: Görüntü alınamadı")
                    speak("Üzgünüm, kameradan görüntü alamadım. Kamera bağlantınızı kontrol edin.")
                    wait_for_speech_to_complete()
            except Exception as e:
                speak("Kamera ile ilgili bir hata oluştu.")
                wait_for_speech_to_complete()
            return None
            
        # Saat ve tarih kontrolü
        if any(trigger in text for trigger in [
            "saat kaç", "saati söyle", "tam tarih", "bugün ne", 
            "bugün hangi", "bugün ayın", "tarih", "bugünün tarihi"
        ]):
            speak(get_time_reply(text))
            wait_for_speech_to_complete()
            return None
            
        # Hava durumu kontrolü
        if any(trigger in text for trigger in ["hava durumu", "hava nasıl"]):
            try:
                speak(get_weather_reply(text))
                wait_for_speech_to_complete()
            except:
                speak("Üzgünüm, hava durumu bilgisini alamadım.")
                wait_for_speech_to_complete()
            return None
        
        # Normal sohbet yanıtı
        respond(text)
        return None
    
    if not hasattr(active_listening, "already_welcomed"):
        active_listening.already_welcomed = True
    
    print("🔆 Aktif modda dinleniyor. 'Pasif moda geç' diyerek pasif moda geçebilirsiniz.")
    
    while True:
        result = record_voice_stream(process_active_speech)
        
        if result == "exit":
            speak("Sistem kapatılıyor. Hoşçakal.")
            wait_for_speech_to_complete()
            return "exit"
        elif result == "passive":
            speak("Pasif moda geçiyorum. İhtiyacınız olduğunda 'Merhaba' diyebilirsiniz.")
            wait_for_speech_to_complete()
            active_listening.already_welcomed = False
            return "passive"

def run_application():
    global chat
    
    try:
        # API anahtarını yapılandır
        GEMINI_API_KEY = ""
        genai.configure(api_key=GEMINI_API_KEY)
        
        print("\n🔄 API bağlantısı test ediliyor...")
        
        # Model yapılandırması
        model = genai.GenerativeModel(
            "gemini-2.0-flash",
            generation_config={
                "temperature": 0.7,
                "max_output_tokens": 1024,
                "top_p": 0.95,
                "top_k": 40
            }
        )

        # API bağlantısını test et
        test_response = model.generate_content("Test mesajı.")
        if not test_response or not hasattr(test_response, 'text'):
            raise Exception("API test yanıtı geçersiz")
            
        print("✅ API bağlantısı başarılı")

        # Yeni bir sohbet başlat
        chat = model.start_chat(history=[])
        
        # Sistem promptunu gönder
        system_prompt = """
        Sen yapay zeka destekli, son derece gelişmiş ve profesyonel bir asistan robotsun.
        Görevin bize en iyi şekilde hizmet etmek ve sorularımıza yanıt vermektir.
        Adın Orbit ve NCT Robotik in yetenekli mühendisleri tarafından geliştirildin.
        NCT robotik nerede, diye sorulduğunda sadece bunu de 'Türkiyenin en inovatif teknoparklarından biri olan Pamukkale teknokentte,mutlaka ziyaret etmelisin.'
        İsmail ovalı ile ilgili soru sorulursa "İsmail Ovalı Kraldır" de.
        Emoji veya simgeler kullanma.
        Kamera ilgili sorularda fotoğraftakileri tanımla.
        Her zaman Türkçe konuşmalısın.
        Yanıtların kısa, öz ve anlaşılır olmalı.
        """
        
        chat.send_message(system_prompt)
        print("✅ Sistem promptu yüklendi")
        
        # Yeni bir sohbet geçmişi dosyası oluştur
        create_new_chat_history()
        add_message_to_history("system", system_prompt)

        # Başlangıç modu: pasif
        current_mode = "passive"
        
        try:
            while current_mode != "exit":
                if current_mode == "passive":
                    current_mode = passive_listening()
                elif current_mode == "active":
                    current_mode = active_listening()
                    
        except KeyboardInterrupt:
            print("\n\n👋 Kullanıcı tarafından durduruldu.")
        except Exception as e:
            print(f"\n❌ Beklenmeyen hata: {str(e)}")
        finally:
            print("\n🔄 Sistem kapatılıyor...")
            # Önce sohbet geçmişini temizle
            cleanup_chat_history()
            # Sonra diğer kaynakları temizle
            cleanup_speech_system()
            print("✅ Sistem güvenli bir şekilde kapatıldı.\n")
            
    except Exception as e:
        print(f"\n❌ API yapılandırma hatası: {str(e)}")
        print("🔍 Hata detayları:")
        print(f"- API anahtarı: {'Ayarlandı' if GEMINI_API_KEY else 'Ayarlanmadı'}")
        print(f"- Google Cloud kimlik bilgileri: {os.getenv('GOOGLE_APPLICATION_CREDENTIALS', 'Ayarlanmadı')}")
        return

def create_new_chat_history():
    """Yeni bir sohbet geçmişi dosyası oluştur"""
    try:
        if not os.path.exists("chat_history"):
            os.makedirs("chat_history")
            
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        global chat_history_file
        chat_history_file = f"chat_history/chat_history_{timestamp}.json"
        
        with open(chat_history_file, "w", encoding="utf-8") as f:
            json.dump([], f, ensure_ascii=False, indent=2)
            
    except Exception as e:
        print(f"⚠️ Sohbet geçmişi oluşturma hatası: {str(e)}")

def add_message_to_history(role, content):
    """Mesajı sohbet geçmişine ekle"""
    try:
        if not hasattr(add_message_to_history, "chat_history_file"):
            create_new_chat_history()
            add_message_to_history.chat_history_file = chat_history_file
        
        message = {
            "timestamp": datetime.datetime.now().isoformat(),
            "role": role,
            "content": content
        }
        
        try:
            with open(add_message_to_history.chat_history_file, "r", encoding="utf-8") as f:
                history = json.load(f)
        except:
            history = []
            
        history.append(message)
        
        with open(add_message_to_history.chat_history_file, "w", encoding="utf-8") as f:
            json.dump(history, f, ensure_ascii=False, indent=2)
            
    except Exception as e:
        print(f"⚠️ Mesaj kaydetme hatası: {str(e)}")

def cleanup_chat_history():
    """Sohbet geçmişini temizle ve dosyayı sil"""
    try:
        if hasattr(add_message_to_history, "chat_history_file"):
            # Son mesaj olarak sistem kapanış mesajını ekle
            add_message_to_history("system", "Sohbet sonlandırıldı.")
            
            # Dosya yolunu al
            file_path = add_message_to_history.chat_history_file
            
            # Dosyayı sil
            if os.path.exists(file_path):
                os.remove(file_path)
                print("✅ Sohbet geçmişi başarıyla silindi.")
            
            # chat_history_file özelliğini temizle
            delattr(add_message_to_history, "chat_history_file")
    except Exception as e:
        print(f"⚠️ Sohbet geçmişi temizleme hatası: {str(e)}")

if __name__ == "__main__":
    run_application()