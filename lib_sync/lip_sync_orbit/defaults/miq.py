import numpy as np
import cv2
import os
import sounddevice as sd
import time

script_dir = os.path.dirname(os.path.abspath(__file__))
mouth_folder = os.path.join(script_dir, "mouth")
eyes_video_path = os.path.join(mouth_folder, "blinking.mp4")

# Mikrofon ayarları
samplerate = 22050  # Ses örnekleme hızı
frame_rate = 30     # FPS (Saniyede kaç kare)
frame_samples = samplerate // frame_rate  # Her frame için kaç örnek
preveus_time = time.time()

# Dudak görselleri
lip_images = {
    "1": cv2.imread("mouth/1_face_.png", cv2.IMREAD_UNCHANGED),  # Kapalı ağız
    "1_2": cv2.imread("mouth/1_face.png", cv2.IMREAD_UNCHANGED),
    "2": cv2.imread("mouth/2_face.png", cv2.IMREAD_UNCHANGED),
    "4": cv2.imread("mouth/3_face.png", cv2.IMREAD_UNCHANGED),  # Yarı açık ağız
    "3": cv2.imread("mouth/4_face.png", cv2.IMREAD_UNCHANGED),
    "5": cv2.imread("mouth/5_face.png", cv2.IMREAD_UNCHANGED)  # Tam açık ağız
}

# Göz videosunu aç
eyes_video = cv2.VideoCapture(eyes_video_path)

# Minimum ve maksimum ses şiddeti değerleri (kalibrasyon için başlangıç)
min_amp, max_amp = 0.001, 0.1

# Mikrofon verisini işleyen fonksiyon
def audio_callback(indata, frames, time_info, status):
    global min_amp, max_amp, eyes_video,preveus_time  

    if status:
        print(status)
    
    # Ses verisini mono kanala indir
    audio_data = np.abs(indata[:, 0])
    
    # Ses şiddetini hesapla (amplitude)
    amp = np.max(audio_data)

    # Minimum ve maksimum değerleri güncelle (otomatik kalibrasyon)
    min_amp = min(min_amp, amp)
    max_amp = max(max_amp, amp)

    # Dudak açıklığını hesapla
    lip_height = int(10 + (amp - min_amp) / (max_amp - min_amp) * 110)
    # print(lip_height)
    
    # Dudak görselini seç
    if 10 <= lip_height < 15:
        lip_time =  time.time() - preveus_time
        # print(lip_time)
        if lip_time < 0.25:
            lip_img = lip_images["2"]
        else:
            lip_img = lip_images["1"]
    elif 15 <= lip_height < 60:
        lip_img = lip_images["2"]
        preveus_time = time.time()
    elif 60 <= lip_height < 80:
        lip_img = lip_images["3"]
        preveus_time = time.time()
    elif 80 <= lip_height < 100:
        lip_img = lip_images["4"]
        preveus_time = time.time()
    elif 100 <= lip_height < 120:
        lip_img = lip_images["5"]
        preveus_time = time.time()
    else:
        lip_img = lip_images["1_2"]
        preveus_time = time.time()

    # Göz videosundan kare oku
    ret, eye_frame = eyes_video.read()
    if not ret:
        eyes_video.set(cv2.CAP_PROP_POS_FRAMES, 0)
        ret, eye_frame = eyes_video.read()

    # Dudak ve göz görüntülerini birleştir
    mouth_crop = 730
    height_rate = int(((1080 - mouth_crop) / 1920) * 1280)
    
    if lip_img is not None and lip_img.shape[0] > mouth_crop:
        eye_frame = eye_frame[:mouth_crop - 1080, :]
        # lip_img = lip_img[mouth_crop:, :]
        if 10 <= lip_height < 15 and lip_time > 0.25:
            mouth_crop = mouth_crop + 100
            lip_img = lip_img[mouth_crop:, :]
        else:
            lip_img = lip_img[mouth_crop:, :]
            
        eye_frame = cv2.resize(eye_frame, (1280, 720 - height_rate))
        lip_img = cv2.resize(lip_img, (1280, height_rate))

        # Transparan PNG desteği için BGR formatına çevir
        if lip_img.shape[2] == 4:
            lip_img = cv2.cvtColor(lip_img, cv2.COLOR_BGRA2BGR)

        final_frame = np.vstack((eye_frame, lip_img))
        cv2.namedWindow("Lip Sync", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Lip Sync", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow("Real-Time Lip Sync", final_frame)

    # Çıkış için 'q' tuşuna basılmasını kontrol et
    if cv2.waitKey(1) & 0xFF == ord("q"):
        exit(0)

# Mikrofonu başlat ve ses verisini işle
with sd.InputStream(callback=audio_callback, samplerate=samplerate, channels=1):
    print("Gerçek zamanlı lip-sync başlatıldı. Kapatmak için 'q' tuşuna basın.")
    while True:
        time.sleep(0.05)  # Her frame için küçük bir bekleme süresi

cv2.destroyAllWindows()
