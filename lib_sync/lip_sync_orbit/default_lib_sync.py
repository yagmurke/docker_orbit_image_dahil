import librosa
import numpy as np
import cv2, os
import pygame
import time
import matplotlib.pyplot as plt

script_dir = os.path.dirname(os.path.abspath(__file__))  # Correct the reference to __file__
audio_folder = os.path.join(script_dir, "Audio")
mouth_folder = os.path.join(script_dir, "mouth")
mp3_file = os.path.join(audio_folder, f"output.mp3")
eyes_video_path = os.path.join(mouth_folder, "eyes_2.mp4")
# 1️⃣ MP3 Dosyasını Yükle

y, sr = librosa.load(mp3_file, sr=22050)

# 2️⃣ Ses Dalga Şiddetini (Amplitude) Ölç
frame_rate = 30 # FPS (Saniyede kaç kare olacak)
frame_samples = sr // frame_rate  # Her frame için kaç örnek alacağız

lip_images = {
    "1": cv2.imread("orbit/B_M_P.png", cv2.IMREAD_UNCHANGED),  # Kapalı ağız
    "2": cv2.imread("orbit/C_D_G_K_N_S_T_X_Y_Z.png", cv2.IMREAD_UNCHANGED),
    "3": cv2.imread("orbit/R.png", cv2.IMREAD_UNCHANGED),  # Yarı açık ağız
    "4": cv2.imread("orbit/EE.png", cv2.IMREAD_UNCHANGED), 
    "5": cv2.imread("orbit/CH_J_SH.png", cv2.IMREAD_UNCHANGED)  # Tam açık ağız
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
eyes_video = cv2.VideoCapture(eyes_video_path)

for i, amp in enumerate(amplitudes):
    ret, eye_frame = eyes_video.read()  
    if not ret:
        eyes_video.set(cv2.CAP_PROP_POS_FRAMES, 0)  
        ret, eye_frame = eyes_video.read()

    # Dudak Açıklığını Hesapla
    lip_height = int(10 + (amp - min_amp) / (max_amp - min_amp) * 120)

    print(lip_height)
    
    if 10<=lip_height < 40:
        lip_img = lip_images["1"]
    elif 40<=lip_height < 60:
        lip_img = lip_images["2"]
    elif 60<=lip_height < 80:
        lip_img = lip_images["3"]
    elif 80<=lip_height < 100:
        lip_img = lip_images["4"]
    elif 100<=lip_height < 120:
        lip_img = lip_images["5"]

    mouth_crop = 820 # Dudak görüntüsünün alt kısmını kırp
    height_rate = int(((1080 - mouth_crop) / 1920) * 1280) # Dudak görüntüsünü yeniden boyutlandır

    if lip_img is not None and lip_img.shape[0] > mouth_crop:
        eye_frame = eye_frame[:mouth_crop - 1080, :]
        lip_img = lip_img[mouth_crop:, :]

        eye_frame = cv2.resize(eye_frame, (1280, 720 - height_rate))
        lip_img = cv2.resize(lip_img, (1280, height_rate))
    else:
        continue
    if lip_img.shape[2] == 4:
        lip_img = cv2.cvtColor(lip_img, cv2.COLOR_BGRA2BGR)

    final_frame = np.vstack((eye_frame, lip_img))

    cv2.namedWindow("Lip Sync", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Lip Sync", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("Lip Sync", final_frame)

    # Senkronizasyon İçin Bekleme
    elapsed_time = time.time() - start_time
    target_time = (i + 1) / frame_rate
    wait_time = max(0, target_time - elapsed_time)
    time.sleep(wait_time)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
pygame.mixer.music.stop() 
