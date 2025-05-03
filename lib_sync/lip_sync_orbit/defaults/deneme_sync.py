import librosa
import numpy as np
import cv2, os
import pygame
import time
import matplotlib.pyplot as plt

script_dir = os.path.dirname(os.path.abspath(__file__))  # Correct the reference to __file__
audio_folder = os.path.join(script_dir, "Audio")
mouth_folder = os.path.join(script_dir, "mouth")
feelings_folder = os.path.join(script_dir, "feelings")
mp3_file = os.path.join(audio_folder, f"output.mp3")
eyes_video_path = os.path.join(mouth_folder, "blinking.mp4")
feelin_video_path = os.path.join(feelings_folder, "breathe.mp4")
# 1️⃣ MP3 Dosyasını Yükle

y, sr = librosa.load(mp3_file, sr=22050)

frame_rate = 30 # FPS (Saniyede kaç kare olacak)
frame_samples = sr // frame_rate  # Her frame için kaç örnek alacağız

lip_images = {
    "1": cv2.imread("mouth/1_face_.png", cv2.IMREAD_UNCHANGED),  # Kapalı ağız
    "1_2": cv2.imread("mouth/1_face.png", cv2.IMREAD_UNCHANGED),
    "2": cv2.imread("mouth/2_face.png", cv2.IMREAD_UNCHANGED),
    "3": cv2.imread("mouth/3_face.png", cv2.IMREAD_UNCHANGED),  # Yarı açık ağız
    "4": cv2.imread("mouth/4_face.png", cv2.IMREAD_UNCHANGED), 
    "5": cv2.imread("mouth/5_face.png", cv2.IMREAD_UNCHANGED)  # Tam açık ağız
}

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
eyes_video = cv2.VideoCapture(eyes_video_path)
expression_video = cv2.VideoCapture(feelin_video_path)

for i, amp in enumerate(amplitudes):
    ret, eye_frame = eyes_video.read()  
    if not ret:
        eyes_video.set(cv2.CAP_PROP_POS_FRAMES, 0)  
        ret, eye_frame = eyes_video.read()

    # Dudak Açıklığını Hesapla
    lip_height = int(10 + (amp - min_amp) / (max_amp - min_amp) * 110)

    print(lip_height)
    
    if 10<=lip_height < 15:
        lip_time =  time.time() - preveus_time
        print(lip_time)
        if lip_time < 0.1:
            lip_img = lip_images["2"]
        else:
            lip_img = lip_images["1"]
    elif 15<=lip_height < 60:
        lip_img = lip_images["2"]
        preveus_time = time.time()
    elif 60<=lip_height < 80:
        lip_img = lip_images["3"]
        preveus_time = time.time()
    elif 80<=lip_height < 100:
        lip_img = lip_images["4"]
        preveus_time = time.time()
    elif 100<=lip_height < 120:
        lip_img = lip_images["5"]
        preveus_time = time.time()

#Göz ve Dudak Görüntülerini Kirp ve Birleştir
    mouth_crop = 730 # Dudak görüntüsünün alt kısmını kırp
    height_rate = int(((1080 - mouth_crop) / 1920) * 1280) # Dudak görüntüsünü yeniden boyutlandır

    if lip_img is not None and lip_img.shape[0] > mouth_crop:
        eye_frame = eye_frame[:mouth_crop - 1080, :]
        # lip_img = lip_img[mouth_crop:, :]
        if 10<=lip_height < 15 and lip_time > 0.1:
            mouth_crop = mouth_crop + 100
            lip_img = lip_img[mouth_crop:, :]
        else:
            lip_img = lip_img[mouth_crop:, :]
            
        eye_frame = cv2.resize(eye_frame, (1280, 720 - height_rate))
        lip_img = cv2.resize(lip_img, (1280, height_rate))
    else:
        continue
    if lip_img.shape[2] == 4:
        lip_img = cv2.cvtColor(lip_img, cv2.COLOR_BGRA2BGR)

    final_frame = np.vstack((eye_frame, lip_img))

    cv2.imshow("Lip Sync", final_frame)

    # Senkronizasyon İçin Bekleme
    elapsed_time = time.time() - start_time
    target_time = (i + 1) / frame_rate
    wait_time = max(0, target_time - elapsed_time)
    time.sleep(wait_time)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
while expression_video.isOpened():
    ret, exp_frame = expression_video.read()
    if not ret:
        break
    exp_frame = cv2.resize(exp_frame, (1280, 720))
    cv2.imshow("Lip Sync", exp_frame)
    if cv2.waitKey(33) & 0xFF == ord("q"):
        break

# İlk yüz ifadesine geri dön
eyes_video.set(cv2.CAP_PROP_POS_FRAMES, 0)
expression_video.release()

cv2.destroyAllWindows()
pygame.mixer.music.stop() 
