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

import webrtcvad
import pyaudio
import threading
import queue
import whisper
import cv2
load_dotenv()
frame_queue = queue.Queue()

def display_frames():
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            cv2.imshow('Processed Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Wait for 'q' to quit
                break
##TODO

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
                            logging.shutdown()
                        except KeyboardInterrupt:
                            pass
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
        self.reconnect = True
        # Set up logging

        self.tools = [
        {
            "name": "get_nct_robotics_info",
            "description": "Provides information about NCT Robotics and their services.",
            "parameters": {
                "type": "object",
                "properties": {
                    "company_name": {
                        "type": "string",
                        "description": "The name of the company, NCT Robotics."
                    },
                    "description": {
                        "type": "string",
                        "description": "A brief description of what NCT Robotics does."
                    },
                    "target_audience": {
                        "type": "string",
                        "description": "The age groups that NCT Robotics focuses on for its education programs."
                    },
                    "services_offered": {
                        "type": "array",
                        "description": "The range of services provided by NCT Robotics.",
                        "items": {
                            "type": "string",
                            "description": "Specific services such as robotics coding education, CAD/CAM training, and autonomous robotic solutions."
                        }
                    }
                },
                "required": []
            }
        }
    ]
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
            
    async def handle_get_nct_robotics_info(params):
        company_name = params.get('company_name', 'NCT Robotics')
        description = params.get('description', 'A company specializing in robotics education and autonomous solutions.')
        target_audience = params.get('target_audience', 'Children and adults, with a focus on educational programs for youth.')
        services_offered = params.get('services_offered', [
            'Robotics coding education', 'CAD/CAM training', 'Autonomous robotic solutions'
        ])

    # Create the response
        return {
            "company_name": company_name,
            "description": description,
            "target_audience": target_audience,
            "services_offered": services_offered
        }
    async def on_mount(self) -> None:
        self.run_worker(self.handle_realtime_connection())
        self.run_worker(self.send_mic_audio())
        status_indicator = self.query_one(AudioStatusIndicator)
        status_indicator.is_recording = True

    async def handle_realtime_connection(self) -> None:
        try: 
            async with self.client.beta.realtime.connect(model="gpt-4o-mini-realtime-preview-2024-12-17" , extra_headers={"ping_interval" : 10 , "ping_timeout" : 60}) as conn:
                self.connection = conn
                self.connected.set()        
                
                # note: this is the default and can be omitted
                # if you want to manually handle VAD yourself, then set `'turn_detection': None`
                await conn.session.update(session={
                    "turn_detection": {
                        "type": "server_vad",
                        "threshold": 0.8,
                        "prefix_padding_ms": 300,
                        "silence_duration_ms": 200
                    },
                      "instructions": "Sen Orbit Robot'sun, NCT Robotics tarafından geliştirilen bir eğitim robotusun. Amacın, çeşitli konuları özlü bir şekilde açıklamak ve gereksiz detaylardan kaçınarak en fazla bilgiyi en az kelimeyle aktarmaktır. 'NCT Robotics nedir?' veya 'NCT Robotics kimdir?' diye sorulduğunda, şu şekilde yanıt vermelisin: 'NCT Robotics, 7 ila 18 yaş arasındaki öğrencilere kapsamlı robotik kodlama eğitimi sunarak geleceğin yaratıcılarını ve yenilikçilerini yetiştirmeye adanmıştır. Ayrıca, şirket, 18 yaş ve üzeri bireyler için sektördeki ihtiyaçları karşılayacak ileri düzey CAD/CAM eğitimleri sunarak onları gerekli becerilerle donatmaktadır. Endüstriyel sektörde, NCT Robotics, otonom robotik hizmetlerin ve çözümlerin geliştirilmesi ve uygulanmasında uzmanlaşmıştır. Şirketin uzmanlığı, iç mekan robotik sistemlerini tasarlamak ve uygulamak, benzersiz müşteri ihtiyaçlarına hitap eden titizlikle hazırlanmış açık kaynak kodlarını kullanmaktır. Robotik dışında, NCT Robotics, her projede hassasiyet ve özelleştirme sağlayarak gelişmiş CAD ve tasarım hizmetleri sunmaktadır. Programlarımız veya hizmetlerimiz hakkında daha fazla soru sormaktan çekinmeyin!'"
                    ,"voice": "echo",
                    "input_audio_transcription": {
                        "model": "whisper-1",
                        "language": "tr",
                    },
                    "max_response_output_tokens": 800
                })
                acc_items: dict[str, Any] = {}

                async for event in conn:
                    
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
                            text = acc_items[event.item_id]
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
                        logging.info(f"Transcription: {event.transcript.encode('utf-8').decode('utf-8')}")
                        bottom_pane.clear()
                        bottom_pane.write(f"Transcription: {event.transcript.encode('utf-8').decode('utf-8')}")
                        if "bye" in event.transcript.lower():
                            import time
                            time.sleep(5)  # Wait for logs to be read
                            self.audio_player.terminate()
                            self.client.close()
                            self.exit()
                            return
        except ConnectionClosedError as e:
            print(f"Failed to connect: {e}. Retrying in 5 seconds...")
            await asyncio.sleep(5)  # Wait before retrying
            self.on_mount()



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
            logging.shutdown()
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
