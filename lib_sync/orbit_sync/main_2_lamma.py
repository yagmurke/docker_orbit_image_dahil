#!/usr/bin/env uv run
####################################################################
# Sample TUI app with a push to talk interface to the Realtime API #
# If you have `uv` installed and the `OPENAI_API_KEY`              #
# environment variable set, you can run this example with just     #
#                                                                  #
# `./examples/realtime/push_to_talk_app.py`                        #
####################################################################
#
# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "textual",
#     "numpy",
#     "pyaudio",
#     "pydub",
#     "sounddevice",
#     "openai[realtime]",
# ]
#
# [tool.uv.sources]
# openai = { path = "../../", editable = true }
# ///
import speech_recognition as sr
import threading
import time

import base64
import asyncio
from typing import Any, cast
from typing_extensions import override
# import sounddevice as sd  # type: ignore
import numpy as np
from audio_utils import CHANNELS, SAMPLE_RATE, AudioPlayerAsync
from openai import AsyncOpenAI
from openai.types.beta.realtime.session import Session
from openai.resources.beta.realtime.realtime import AsyncRealtimeConnection
from dotenv import load_dotenv
import os
from websockets.exceptions import ConnectionClosedError
from openwakeword.model import Model
import sounddevice as sd
import numpy as np
import logging

import threading
# Load Silero VAD Model
load_dotenv()

async def main():
    obj = RealtimeApp()
    await obj.run_worker()

class RealTimeRecognizer:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
    def recognize_speech(self):
        """Handles speech recognition in real-time."""
        with self.microphone as source:
            print("Adjusting for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source)  # Adjust to ambient noise
            while True:
                print("Listening...")
                print(self.microphone.device_index)
                try:
                    audio = self.recognizer.listen(source, timeout=9)  # Listen for audio
                    print("Recognizing...")
                    text = self.recognizer.recognize_google(audio)  # Use Google Web Speech API
                    print(f"Recognized: {text}")
                    if any(word in text.lower() for word in ["hey", "merhaba", "hi" , "hello"]):
                        try:
                            asyncio.run(main())
                        except KeyboardInterrupt:
                            pass
                        except ConnectionClosedError:
                            asyncio.run(main())
                        else: 
                            print("Next")

                except sr.WaitTimeoutError:
                    print("Timed out listening, continuing...")
                except sr.UnknownValueError:
                    print("Sorry, could not understand audio.")
                except sr.RequestError as e:
                    print(f"Error with the speech recognition service: {e}")

    def start_listening(self):
        """Start a thread to handle speech recognition."""
        listen_thread = threading.Thread(target=self.recognize_speech )
        listen_thread.daemon = True  # Allow the program to exit even if the thread is still running
        listen_thread.start()

# Create an instance and start recognizing speech in real-time
recognizer = RealTimeRecognizer()
recognizer.start_listening()




class RealtimeApp():


    client: AsyncOpenAI
    should_send_audio: asyncio.Event
    audio_player: AudioPlayerAsync
    last_audio_item_id: str | None
    connection: AsyncRealtimeConnection | None
    session: Session | None
    connected: asyncio.Event

    def __init__(self) -> None:
        super().__init__()
        self.session_id = None
        self.connection = None
        self.session = None
        self.client = AsyncOpenAI(timeout=60 )
        self.audio_player = AudioPlayerAsync()
        self.last_audio_item_id = None
        self.should_send_audio = asyncio.Event()
        self.connected = asyncio.Event()
        self.audio_finished = asyncio.Event()
        self.is_recording = True
        # Set up logging
        log_file = "log_file.log"
        if os.path.exists(log_file):
            os.remove(log_file)
        self.logs = logging.basicConfig(
            filename=log_file,
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s"
        )

        logging.info("Logging to another terminal in Windows!")
        os.system(f'start cmd /k "type {log_file}"')
    async def handle_realtime_connection(self) -> None:
        while True:
            try: 
                async with self.client.beta.realtime.connect(model="gpt-4o-mini-realtime-preview-2024-12-17" , extra_headers={"ping_interval" : 10 , "ping_timeout" : 900}) as conn:
                    self.connection = conn
                    self.connected.set()

                    # note: this is the default and can be omitted
                    # if you want to manually handle VAD yourself, then set `'turn_detection': None`
                    await conn.session.update(session={
                        "turn_detection": {
                            "type": "server_vad",
                            "threshold": 0.75,
                            "prefix_padding_ms": 500,
                            "silence_duration_ms": 500
                        },
                        "instructions": "Adınız Orbit Robot ve çeşitli konuları öğretmek için tasarlanmış bir Eğitim Robotusunuz. Her zaman en fazla bilgiyi en az kelimeyle aktarmaya öncelik verin",
                        "voice": "echo",
                        "input_audio_transcription": {
                            "model": "whisper-1",
                            "language": "tr",
                        },
                        "max_response_output_tokens": 1024
                    })
                    acc_items: dict[str, Any] = {}

                    async for event in conn:
                        # logging.info(f"Event: {event.type}")
                        
                        if event.type == "session.created":
                            self.session = event.session
                            assert event.session.id is not None
                            self.session_id = event.session.id
                            introduction_message = """Say exactly the following:
                                Merhaba, ben Orbit. NCT Robotics'in bir robotuyum ve burada sizlerle bilgiyi paylaşmak ve öğrenmenize yardımcı olmak için varım.

                                """
                            await conn.response.create(response={
                                    # An empty input array removes all prior context
                                "input": [],
                                "instructions": introduction_message,})
                            continue


                        if event.type == "session.updated":

                            self.session = event.session
                            continue

                        if event.type == "response.audio.delta":
                            self.audio_player.playing = True
                            if event.item_id != self.last_audio_item_id:
                                self.audio_player.reset_frame_count()
                                self.last_audio_item_id = event.item_id
                            bytes_data = base64.b64decode(event.delta)
                            self.audio_player.add_data(bytes_data)
                            # await conn.response.create(response={
                            #         # An empty input array removes all prior context
                            #     "input": [], ## Last Resort
                            #     "modalities": ["text"]
                            #     })
                            continue
                        if event.type == "response.done":
                            self.audio_player.start()
                            
                            connection = await self._get_connection()
                            connection.input_audio_buffer.clear()
                            self.audio_player.playing = False
                            self.should_send_audio.set()
                            continue


                        if event.type == "response.audio_transcript.delta":
                            try:
                                text = acc_items.get(event.item_id, "")
                                if "hoşça kalın" in text.lower():
                                    import time
                                    time.sleep(2)  # Wait for logs to be read
                                    self.audio_player.terminate()
                                    self.client.close()
                                    logging.shutdown()
                            except KeyError:
                                acc_items[event.item_id] = event.delta
                            else:
                                acc_items[event.item_id] = text + event.delta

                            # Clear and update the entire content because RichLog otherwise treats each delta as a new line
                            continue
                        if event.type == "conversation.item.input_audio_transcription.completed":
                            logging.info(f"Transcription: {event.transcript}")
                            if "bye" in event.transcript.lower():
                                import time
                                time.sleep(2)  # Wait for logs to be read
                                self.audio_player.terminate()
                                self.client.close()
                                return
            except ConnectionClosedError as e:
                print(f"Failed to connect: {e}. Retrying in 5 seconds...")
                await asyncio.sleep(5)  # Wait before retrying
    async def run_worker(self):
        # This will run both tasks concurrently
        await asyncio.gather(
            self.handle_realtime_connection(),
            self.send_mic_audio()
        )
    async def _get_connection(self) -> AsyncRealtimeConnection:
            await self.connected.wait()
            assert self.connection is not None
            return self.connection
    async def send_mic_audio(self) -> None:
        import sounddevice as sd  # type: ignore
        sent_audio = False
        device_info = sd.query_devices()
        print(device_info)

        read_size = int(SAMPLE_RATE * 0.02)
        stream = sd.InputStream(
            channels=CHANNELS,
            samplerate=SAMPLE_RATE,
            dtype="int16",
            device=1
        )
        stream.start()
        self.is_recording = True
        try:
            while True:

                if stream.read_available < read_size:
                    await asyncio.sleep(0)
                    continue
                
                if self.audio_player.playing:
                    await asyncio.sleep(0.5)  # Wait and check again

                    continue
                await self.should_send_audio.wait()

                self.is_recording = True
                
                 
                data, _ = stream.read(read_size)
                
                connection = await self._get_connection()
                if not sent_audio:
                    asyncio.create_task(connection.send({"type": "response.cancel"}))
                    sent_audio = True
                await connection.input_audio_buffer.append(audio=base64.b64encode(cast(Any, data)).decode("utf-8"))
                await asyncio.sleep(0)
        except KeyboardInterrupt:
            pass
        finally:
            stream.stop()
            stream.close()  


while True:
    time.sleep(1)
