from __future__ import annotations
import os
import io
import base64
import asyncio
import threading
from typing import Callable, Awaitable
import cv2
import numpy as np
import pyaudio
import sounddevice as sd
from pydub import AudioSegment
import time
from openai.resources.beta.realtime.realtime import AsyncRealtimeConnection

CHUNK_LENGTH_S = 0.05  # 100ms
SAMPLE_RATE = 24000
FORMAT = pyaudio.paInt16
CHANNELS = 1
import logging
# pyright: reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false



def audio_to_pcm16_base64(audio_bytes: bytes) -> bytes:
    # load the audio file from the byte stream
    audio = AudioSegment.from_file(io.BytesIO(audio_bytes))
    print(f"Loaded audio: {audio.frame_rate=} {audio.channels=} {audio.sample_width=} {audio.frame_width=}")
    # resample to 24kHz mono pcm16
    pcm_audio = audio.set_frame_rate(SAMPLE_RATE).set_channels(CHANNELS).set_sample_width(2).raw_data
    return pcm_audio

class AudioPlayerAsync:
    def __init__(self):
        self.lhc = LipHeightCalibrator()
        self.queue = []
        self.lock = threading.Lock()
        self.stream = sd.OutputStream(
            device=5,
            callback=self.callback,
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype=np.int16,
            blocksize=int(CHUNK_LENGTH_S * SAMPLE_RATE),
            
        )
        
        self.playing = False
        self._frame_count = 0

        

    def callback(self, outdata, frames, time, status):  # noqa
        with self.lock:
            data = np.empty(0, dtype=np.int16)
            # get next item from queue if there is still space in the buffer
            while len(data) < frames and len(self.queue) > 0:
                item = self.queue.pop(0)


                frames_needed = frames - len(data)
                data = np.concatenate((data, item[:frames_needed]))
                if len(item) > frames_needed:
                    self.queue.insert(0, item[frames_needed:])

            self._frame_count += len(data)

            # fill the rest of the frames with zeros if there is no more data
            if len(data) < frames:
                data = np.concatenate((data, np.zeros(frames - len(data), dtype=np.int16)))

            # self.expression(data)
            self.expression(data)

            outdata[:] = data.reshape(-1, 1)


        # logging.info(outdata)
        # logging.info(f"Output data shape: {outdata.shape}")

    def reset_frame_count(self):
        self._frame_count = 0
    def get_frame_count(self):
        return self._frame_count

    def add_data(self, data: bytes):
        with self.lock:
            # bytes is pcm16 single channel audio data, convert to numpy array
            np_data = np.frombuffer(data, dtype=np.int16)
            self.queue.append(np_data)
        self.stream.start()
    def get_queue_duration(self):
        with self.lock:
            total_frames = sum(len(item) for item in self.queue)
        return total_frames / SAMPLE_RATE
    def start(self):
        self.playing = True
        # self.stream.start()
        self.wait()
        self.stop()
        cv2.destroyAllWindows()
        
    def wait(self):
        time.sleep(self.get_queue_duration() + 0.675)
    def stop(self):
        self.stream.stop()
        self.playing = False

        with self.lock:
            self.queue = []
    def terminate(self):
        self.stream.close()
        
    def expression(self, data):
        # logging.info(f"Max value of data: {np.max(data)}, Min value of data: {np.min(data)}")
        # logging.info(data.shape)
        # Convert to decibels (using 1 as reference)


        
        # Normalize amplitudes if they're in integer format (e.g., int16)
        # Normalize to the range [0, 1] for float32 data

        rms = np.sqrt(np.mean(data**2))
        decibels = 20 * np.log10(rms) if rms > 0 else 0.0
        decibels = decibels if decibels > 0 else 0.0


        try:

            final_frame = self.lhc.calibrate_(decibels)
        except UnboundLocalError as e :
            logging.error(f"UnboundLocalError: {e}")
            pass

        else : 
            cv2.imshow("Image" , final_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            pass

async def send_audio_worker_sounddevice(
        
    connection: AsyncRealtimeConnection,
    should_send: Callable[[], bool] | None = None,
    start_send: Callable[[], Awaitable[None]] | None = None,
):
    sent_audio = False

    device_info = sd.query_devices()
    print(device_info)

    read_size = int(SAMPLE_RATE * 0.02)

    stream = sd.InputStream(
        channels=CHANNELS,
        samplerate=SAMPLE_RATE,
        dtype="int16",
    )
    stream.start()

    try:
        while True:
            if stream.read_available < read_size:
                await asyncio.sleep(0)
                continue

            data, _ = stream.read(read_size)

            if should_send() if should_send else True:
                if not sent_audio and start_send:
                    await start_send()
                await connection.send(
                    {"type": "input_audio_buffer.append", "audio": base64.b64encode(data).decode("utf-8")}
                )
                sent_audio = True

            elif sent_audio:
                print("Done, triggering inference")
                await connection.send({"type": "input_audio_buffer.commit"})
                await connection.send({"type": "response.create", "response": {}})
                sent_audio = False

            await asyncio.sleep(0)

    except KeyboardInterrupt:
        pass
    finally:
        stream.stop()
        stream.close()
class LipHeightCalibrator:
    def __init__(self):
        self.previous_time = time.time()
        self.min_lip_height = float("inf")  # Start with a very high min
        self.max_lip_height = float("-inf")  # Start with a very low max
        self.L_w = 80.0

        ## PATHS
        self.script_dir = os.path.dirname(os.path.abspath(__file__))  
        self.mouth_folder = os.path.join(self.script_dir, "mouth")
        self.eyes_video_path = os.path.join(self.mouth_folder, "eyes.mp4")


        ###Eye Videos and COntent
        self.eyes_video = cv2.VideoCapture(self.eyes_video_path) ## Gox 
        self.lip_images = {
            "1": cv2.imread("mouth/1_face_.png", cv2.IMREAD_UNCHANGED),  # Kapalı ağız
            "1_2": cv2.imread("mouth/1_face.png", cv2.IMREAD_UNCHANGED),
            "2": cv2.imread("mouth/2_face.png", cv2.IMREAD_UNCHANGED),
            "3": cv2.imread("mouth/3_face.png", cv2.IMREAD_UNCHANGED),  # Yarı açık ağız
            "4": cv2.imread("mouth/4_face.png", cv2.IMREAD_UNCHANGED), 
            "5": cv2.imread("mouth/5_face.png", cv2.IMREAD_UNCHANGED)  # Tam açık ağız
        }
        

    def calculate_height(self , rms_dB): ## Bos ver
        lip_height = 10 ** ((self.L_w - rms_dB - 8) / 20)
        lip_height = lip_height if lip_height > 0 else 0
        return lip_height
    def update_range(self, lip_height):
        """Update the min and max lip height dynamically."""
        if lip_height < self.min_lip_height:
            self.min_lip_height = lip_height
        if lip_height > self.max_lip_height:
            self.max_lip_height = lip_height

    def calibrate_(self, rms_db):
        """Determine which lip image to use based on dynamic range scaling."""
        
        ret, eye_frame = self.eyes_video.read()

        if not ret:
            self.eyes_video.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, eye_frame = self.eyes_video.read()
        
        self.update_range(rms_db)
        
        thresholds = {
            "1" : 0.0 ,
            "2" : 20.0,
            "3" : 30.0,
            "4" : 40.0,
            "5" : 50.0,
        }
        
        
        try:
            lip_time = time.time() - self.previous_time
            mouth_crop = 730 
            logging.info(f"Current min lip height: {self.min_lip_height}, max lip height: {self.max_lip_height}")
            lip_img = None  # Initialize lip_img
            if thresholds["1"] <= rms_db < thresholds["2"]:
                lip_img = self.lip_images["2"] if lip_time < 0.1 else self.lip_images["1"]
                lip_img = lip_img[mouth_crop:, :]  # Apply cropping
            elif thresholds["2"] <= rms_db < thresholds["3"]:
                lip_img = self.lip_images["2"]
                lip_img = lip_img[mouth_crop:, :]
            elif thresholds["3"] <= rms_db < thresholds["4"]:
                lip_img = self.lip_images["3"]
                lip_img = lip_img[mouth_crop:, :]
            elif thresholds["4"] <= rms_db < thresholds["5"]:
                lip_img = self.lip_images["4"]
                lip_img = lip_img[mouth_crop:, :]
            elif rms_db >= thresholds["5"]:
                lip_img = self.lip_images["5"]
                lip_img = lip_img[mouth_crop:, :]
            
            eye_frame = cv2.resize(eye_frame, (1280, 548))
            lip_img = cv2.resize(lip_img, (1280, 172))

        except Exception as e:
            logging.error(f"Error in calibrate_lip_height: {type(e).__name__} - {e}")
            final_frame = self.lip_images["3"]
        finally:
            if lip_img.shape[2] == 4:
                lip_img = cv2.cvtColor(lip_img, cv2.COLOR_BGRA2BGR)
            final_frame = np.vstack((eye_frame, lip_img))
            return final_frame
    def calibrate_lip_height(self, rms_db):
        ret, eye_frame = self.eyes_video.read()

        if not ret:
            self.eyes_video.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, eye_frame = self.eyes_video.read()
            
        
        lip_height = self.calculate_height(rms_db)
        self.update_range(lip_height)

        # Prevent division by zero when min == max initially
        if self.max_lip_height == self.min_lip_height:
            return self.lip_images["1"]
        range_span = self.max_lip_height - self.min_lip_height
        thresholds = {
            "1": self.min_lip_height + 0.01 * range_span,  # 1% of range
            "2": self.min_lip_height + 0.05 * range_span,  # 5% of range
            "3": self.min_lip_height + 0.15 * range_span,  # 15% of range
            "4": self.min_lip_height + 0.30 * range_span,  # 30% of range
            "5": self.min_lip_height + 0.50 * range_span,  # 50% of range
        }
        try:
            lip_time = time.time() - self.previous_time
            mouth_crop = 730 
            lip_img = None  # Initialize lip_img

            if thresholds["1"] <= lip_height < thresholds["2"]:
                lip_img = self.lip_images["2"] if lip_time < 0.1 else self.lip_images["1"]
                lip_img = lip_img[mouth_crop:, :]  # Apply cropping
            elif thresholds["2"] <= lip_height < thresholds["3"]:
                lip_img = self.lip_images["2"]
                lip_img = lip_img[mouth_crop:, :]
            elif thresholds["3"] <= lip_height < thresholds["4"]:
                lip_img = self.lip_images["3"]
                lip_img = lip_img[mouth_crop:, :]
            elif thresholds["4"] <= lip_height < thresholds["5"]:
                lip_img = self.lip_images["4"]
                lip_img = lip_img[mouth_crop:, :]
            else:
                lip_img = self.lip_images["5"]
                lip_img = lip_img[mouth_crop:, :]
            
            eye_frame = cv2.resize(eye_frame, (1280, 548))
            lip_img = cv2.resize(lip_img, (1280, 172))

        except Exception as e:
            logging.error(f"Error in calibrate_lip_height: {type(e).__name__} - {e}")
            final_frame = self.lip_images["3"]
        finally:
            if lip_img.shape[2] == 4:
                lip_img = cv2.cvtColor(lip_img, cv2.COLOR_BGRA2BGR)
            final_frame = np.vstack((eye_frame, lip_img))
            return final_frame
    
