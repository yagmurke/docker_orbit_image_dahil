import librosa
import numpy as np
import cv2, os
import pygame
import time

script_dir = os.path.dirname(os.path.abspath(__file__))  # Correct the reference to __file__
audio_folder = os.path.join(script_dir, "Audio")
mp3_file = os.path.join(audio_folder, f"output.mp3")
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
L_w = 80
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

for i, amp in enumerate(amplitudes):

    # Dudak Açıklığını Hesapla
    lip_height = 10 ** ((L_w - L_p - 8) / 20)

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
    # crop = 260
    # lip_img = lip_img[1080 - crop:, :]  # Dudak resminin sadece ağız kısmını al 
    # height_show = 
    lip_img = cv2.resize(lip_img, (1280, 720))
    
    cv2.imshow("Lip Sync", lip_img)

    # Senkronizasyon İçin Bekleme
    elapsed_time = time.time() - start_time
    target_time = (i + 1) / frame_rate
    wait_time = max(0, target_time - elapsed_time)
    time.sleep(wait_time)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
pygame.mixer.music.stop() 
