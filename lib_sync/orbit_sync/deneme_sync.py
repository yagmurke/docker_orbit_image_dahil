import librosa
import numpy as np
import cv2, os
import pygame
import time
from pydub import AudioSegment

# 📌 Dosya Yolları


script_dir = os.path.dirname(os.path.abspath(__file__))  
audio_folder = os.path.join(script_dir, "Audio")
mouth_folder = os.path.join(script_dir, "mouth")
mp3_file = os.path.join(audio_folder, f"output.mp3")
eyes_video_path = os.path.join(mouth_folder, "eyes.mp4")

audio = AudioSegment.from_file(mp3_file, format="mp3")
samples = np.array(audio.get_array_of_samples(), dtype=np.int16)

y, sr = librosa.load(mp3_file, sr=24000)

frame_rate = 30  
frame_samples = sr // frame_rate  

lip_images = {
    "1": cv2.imread("mouth/_1_.png", cv2.IMREAD_UNCHANGED),  
    "2": cv2.imread("mouth/_2_.png", cv2.IMREAD_UNCHANGED),
    "3": cv2.imread("mouth/_3_.png", cv2.IMREAD_UNCHANGED),  
    "4": cv2.imread("mouth/_4_.png", cv2.IMREAD_UNCHANGED), 
    "5": cv2.imread("mouth/_5_.png", cv2.IMREAD_UNCHANGED)  
}

amplitudes = [
    np.max(np.abs(y[i * frame_samples: (i + 1) * frame_samples]))
    for i in range(len(y) // frame_samples)
]

min_amp, max_amp = min(amplitudes), max(amplitudes)

# 🎵 MP3 Çal
pygame.mixer.init()
pygame.mixer.music.load(mp3_file)
pygame.mixer.music.play()

start_time = time.time()
prev_lip_img = None  

eyes_video = cv2.VideoCapture(eyes_video_path)

for i, amp in enumerate(amplitudes):
    ret, eye_frame = eyes_video.read()  
    if not ret:
        eyes_video.set(cv2.CAP_PROP_POS_FRAMES, 0)  
        ret, eye_frame = eyes_video.read()

    lip_height = int(10 + (amp - min_amp) / (max_amp - min_amp) * 120)
    
    if 10 <= lip_height < 40:
        lip_img = lip_images["1"]
    elif 40 <= lip_height < 60: 
        lip_img = lip_images["2"]
    elif 60 <= lip_height < 80:
        lip_img = lip_images["3"]
    elif 80 <= lip_height < 100:
        lip_img = lip_images["4"]
    elif 100 <= lip_height < 120:
        lip_img = lip_images["5"]

    eye_frame = cv2.resize(eye_frame, (1280, 548))
    lip_img = cv2.resize(lip_img, (1280, 172))  

    if lip_img.shape[2] == 4:
        lip_img = cv2.cvtColor(lip_img, cv2.COLOR_BGRA2BGR)

    final_frame = np.vstack((eye_frame, lip_img))
    cv2.imshow("Face Sync", final_frame)

    # if prev_lip_img is not None:
    #     # Hızlı geçiş için sadece 2 blend adımı kullanıyoruz
    #     for alpha in np.linspace(0, 1, 2):  # Sadece 2 geçiş adımı
    #         blended_img = cv2.addWeighted(prev_lip_img, 1 - alpha, final_frame, alpha, 0)
    #         cv2.imshow("Lip Sync", blended_img)
    #         cv2.waitKey(5)  # Daha kısa bekleme süresi
    
    # else:
    #     cv2.imshow("Lip Sync", final_frame)
    # prev_lip_img = final_frame

    elapsed_time = time.time() - start_time
    target_time = (i + 1) / frame_rate
    wait_time = max(0, target_time - elapsed_time)
    time.sleep(wait_time)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
pygame.mixer.music.stop()
eyes_video.release()
