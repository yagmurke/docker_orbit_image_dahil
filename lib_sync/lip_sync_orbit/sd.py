import sounddevice as sd
import numpy as np

# Set parameters
duration = 5  # Recording duration in seconds
samplerate = 44100  # Standard sample rate
channels = 1  # Mono recording

# Find the 'sysdefault' microphone device ID
device_info = sd.query_devices()
sysdefault_id = None

for i, device in enumerate(device_info):
    if "sysdefault" in device['name'].lower() and device['max_input_channels'] > 0:
        sysdefault_id = i
        break

if sysdefault_id is None:
    print("No 'sysdefault' microphone found.")
else:
    print(f"Using 'sysdefault' microphone: Device {sysdefault_id} ({device_info[sysdefault_id]['name']})")

    # Record audio from sysdefault microphone
    print("Recording...")
    recorded_audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=channels, dtype=np.float32, device=sysdefault_id)
    sd.wait()  # Wait until recording is finished
    print("Recording complete!")

    # Play back the recorded audio
    print("Playing back recorded audio...")
    sd.play(recorded_audio, samplerate)
    sd.wait()  # Wait until playback is finished
    print("Playback complete.")
