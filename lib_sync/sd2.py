import numpy as np
import scipy.signal as signal
import sounddevice as sd

SAMPLE_RATE = 24000
CHANNELS = 1
FRAME_SIZE = 1024  # Window size

def spectral_subtraction(noisy_signal, noise_estimate):
    """Perform basic spectral subtraction for noise suppression."""
    noisy_fft = np.fft.rfft(noisy_signal)
    noise_fft = np.fft.rfft(noise_estimate)
    
    clean_fft = np.maximum(noisy_fft - noise_fft, 0)  # Subtract noise spectrum
    clean_signal = np.fft.irfft(clean_fft)  # Convert back to time domain
    return clean_signal

def callback(indata, outdata, frames, time, status):
    """Real-time noise suppression callback."""
    if status:
        print(status)
    
    noise_estimate = np.mean(indata, axis=0)  # Estimate noise
    clean_audio = spectral_subtraction(indata[:, 0], noise_estimate)
    outdata[:, 0] = clean_audio  # Output processed signal

with sd.Stream(samplerate=SAMPLE_RATE, channels=CHANNELS, callback=callback):
    print("🎤 Running Noise Suppression... Press Ctrl+C to Stop.")
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\n🔇 Stopped.")
