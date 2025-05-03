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

import pvporcupine
import base64
import asyncio
from typing import Any, cast
from typing_extensions import override
import sounddevice as sd  # type: ignore
import numpy as np
import groq
from textual import events
from audio_utils import CHANNELS, SAMPLE_RATE, AudioPlayerAsync
from textual.app import App, ComposeResult
from textual.widgets import Button, Static, RichLog
from textual.reactive import reactive
from textual.containers import Container
from textual.app import App, ComposeResult
from textual.widgets import Input
from textual.events import Key
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


# def expression(self , amplitudes):
#         amps = amplitudes / 32768.0  

#         logging.info(amps[10:20])
#         min_amp, self.max_amp = min(amplitudes), max(amplitudes)

#         for i, amp in enumerate(amps): 
#             ret, eye_frame = self.eyes_video.read()  
#             if not ret:
#                 self.eyes_video.set(cv2.CAP_PROP_POS_FRAMES, 0)  
#             ret, eye_frame = self.eyes_video.read()

#             lip_height = int(10 + (amp - min_amp) / (self.max_amp - min_amp) * 120)
            
#             if 10 <= lip_height < 40:
#                 lip_img = self.lip_images["1"]
#             elif 40 <= lip_height < 60: 
#                 lip_img = self.lip_images["2"]
#             elif 60 <= lip_height < 80:
#                 lip_img = self.lip_images["3"]
#             elif 80 <= lip_height < 100:
#                 lip_img = self.lip_images["4"]
#             elif 100 <= lip_height < 120:
#                 lip_img = self.lip_images["5"]

#             eye_frame = cv2.resize(eye_frame, (1280, 548))
#             lip_img = cv2.resize(lip_img, (1280, 172))  

#             if lip_img.shape[2] == 4:
#                 lip_img = cv2.cvtColor(lip_img, cv2.COLOR_BGRA2BGR)

#             final_frame = np.vstack((eye_frame, lip_img))
#             cv2.imshow("Face Sync", final_frame)

#             if cv2.waitKey(1) & 0xFF == ord("q"):
#                 break
#         cv2.destroyAllWindows()
#         self.eyes_video.release()
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
                try:
                    audio = self.recognizer.listen(source, timeout=9)  # Listen for audio
                    print("Recognizing...")
                    text = self.recognizer.recognize_google(audio)  # Use Google Web Speech API
                    print(f"Recognized: {text}")
                    if any(word in text.lower() for word in ["hey", "merhaba", "hi" , "hello"]):
                        app = RealtimeApp()
                        try:
                            app.run()
                        except KeyboardInterrupt:
                            pass
                        except ConnectionClosedError:
                            sd.play("error.wav" , samplerate= SAMPLE_RATE)
                            sd.wait()
                            app = RealtimeApp()
                            app.run()
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

class SessionDisplay(Static):
    """A widget that shows the current session ID."""

    session_id = reactive("")

    @override
    def render(self) -> str:
        return f"Session ID: {self.session_id}" if self.session_id else "Connecting..."


class AudioStatusIndicator(Static):
    """A widget that shows the current audio recording status."""

    is_recording = reactive(False)

    @override
    def render(self) -> str:
        status = (
            "🔴 Recording... (Press K to stop)" if self.is_recording else "⚪ Press K to start recording (Q to quit)"
        )
        return status


class RealtimeApp(App[None]):
    CSS = """
        Screen {
            background: #1a1b26;  /* Dark blue-grey background */
        }

        Container {
            border: double rgb(91, 164, 91);
        }

        Horizontal {
            width: 100%;
        }

        #input-container {
            height: 5;  /* Explicit height for input container */
            margin: 1 1;
            padding: 1 2;
        }

        Input {
            width: 80%;
            height: 3;  /* Explicit height for input */
        }

        Button {
            width: 20%;
            height: 3;  /* Explicit height for button */
        }

        #bottom-pane {
            width: 100%;
            height: 82%;  /* Reduced to make room for session display */
            border: round rgb(205, 133, 63);
            content-align: center middle;
        }

        #status-indicator {
            height: 3;
            content-align: center middle;
            background: #2a2b36;
            border: solid rgb(91, 164, 91);
            margin: 1 1;
        }

        #session-display {
            height: 3;
            content-align: center middle;
            background: #2a2b36;
            border: solid rgb(91, 164, 91);
            margin: 1 1;
        }

        Static {
            color: white;
        }
    """

    client: AsyncOpenAI
    should_send_audio: asyncio.Event
    audio_player: AudioPlayerAsync
    last_audio_item_id: str | None
    connection: AsyncRealtimeConnection | None
    session: Session | None
    connected: asyncio.Event

    def __init__(self) -> None:
        super().__init__()
        self.connection = None
        self.session = None
        self.client = AsyncOpenAI(timeout=60 )
        self.audio_player = AudioPlayerAsync()
        self.last_audio_item_id = None
        self.should_send_audio = asyncio.Event()
        self.connected = asyncio.Event()
        self.audio_finished = asyncio.Event()
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

    @override
    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        with Container():
            yield SessionDisplay(id="session-display")
            yield AudioStatusIndicator(id="status-indicator")
            yield RichLog(id="bottom-pane", wrap=True, highlight=True, markup=True)

    async def on_mount(self) -> None:
        try:
            # Attempt to run worker tasks
            self.run_worker(self.handle_realtime_connection())
            self.run_worker(self.send_mic_audio())
            
            # Query and set status indicator
            status_indicator = self.query_one(AudioStatusIndicator)
            status_indicator.is_recording = True
            
        except Exception as e:
            # Handle the error gracefully
            bottom_pane = self.query_one("#bottom-pane", RichLog)
            bottom_pane.clear()
            bottom_pane.write(f"Error: {e}. Attempting to reconnect...")
            logging.error(f"Error: {e}. Attempting to reconnect...")
            await self.handle_connection_loss(e)

            # You can decide whether to attempt a reconnection here or handle differently
    async def handle_connection_loss(self, error):
        """Handle the connection loss or any specific error."""
        # Add your logic for reconnecting or retrying the connection
        # For instance, you might want to restart the connection or notify the user
        await asyncio.sleep(2)  # Wait for a short period before retrying
        await self.on_mount()  # Retry mounting the components or connection handling

        
    async def handle_realtime_connection(self) -> None:
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
                        session_display = self.query_one(SessionDisplay)
                        assert event.session.id is not None
                        session_display.session_id = event.session.id
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
                                self.exit()
                        except KeyError:
                            acc_items[event.item_id] = event.delta
                        else:
                            acc_items[event.item_id] = text + event.delta

                        # Clear and update the entire content because RichLog otherwise treats each delta as a new line
                        bottom_pane = self.query_one("#bottom-pane", RichLog)
                        bottom_pane.clear()
                        bottom_pane.write(acc_items[event.item_id])
                        continue
                    if event.type == "conversation.item.input_audio_transcription.completed":
                        logging.info(f"Transcription: {event.transcript}")
                        if "bye" in event.transcript.lower():
                            import time
                            time.sleep(2)  # Wait for logs to be read
                            self.audio_player.terminate()
                            self.client.close()
                            self.exit()
                            return
        except ConnectionClosedError as e:
            print(f"Failed to connect: {e}. Retrying in 5 seconds...")
            await asyncio.sleep(5)  # Wait before retrying


    async def _get_connection(self) -> AsyncRealtimeConnection:
        try:
            await self.connected.wait()
        except ConnectionClosedError:
            self.handle_connection_loss()
        finally:
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
        
        status_indicator = self.query_one(AudioStatusIndicator)
        try:
            while True:

                if stream.read_available < read_size:
                    await asyncio.sleep(0)
                    continue
                
                if self.audio_player.playing:
                    await asyncio.sleep(0.5)  # Wait and check again

                    continue
                await self.should_send_audio.wait()

                bottom_pane = self.query_one("#bottom-pane", RichLog)
                bottom_pane.clear()
                bottom_pane.write("Recording.")
                bottom_pane.clear()
                bottom_pane.write("Recording..")
                bottom_pane.clear()
                bottom_pane.write("Recording...")

                
                status_indicator.is_recording = True
                
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

    async def on_key(self, event: events.Key) -> None:
        """Handle key press events."""
        if event.key == "enter":
            self.query_one(Button).press()
            return

        if event.key == "q":
            self.exit()
            return

        if event.key == "k":
            status_indicator = self.query_one(AudioStatusIndicator)
            logging.info("Key pressed: %s", event.key)
            if status_indicator.is_recording:
                logging.info("Stopping recording")
                self.should_send_audio.clear()
                status_indicator.is_recording = False

                if self.session and self.session.turn_detection is None:
                    # The default in the API is that the model will automatically detect when the user has
                    # stopped talking and then start responding itself.
                    #
                    # However if we're in manual `turn_detection` mode then we need to
                    # manually tell the model to commit the audio buffer and start responding.
                    conn = await self._get_connection()
                    logging.info("Committing audio buffer")
                    await conn.input_audio_buffer.commit()
                    await conn.response.create()
            else:
                self.should_send_audio.set()
                status_indicator.is_recording = True


while True:
    time.sleep(1)
