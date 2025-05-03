import torch
import torchaudio
import asyncio
import websockets
import json
from silero_vad import get_speech_timestamps, load_model

# Load Silero VAD Model
model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=True)
get_speech_timestamps = utils['get_speech_timestamps']

# OpenAI API Key (Replace with your key)
OPENAI_API_KEY = "your_openai_api_key"

# OpenAI Realtime API WebSocket URL
REALTIME_API_URL = "wss://api.openai.com/v1/realtime"
