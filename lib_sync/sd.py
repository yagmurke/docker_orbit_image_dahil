import sounddevice as sd
import numpy as np

# Audio settings
SAMPLE_RATE = 24000  # 44.1 kHz standard sample rate
CHANNELS = 2  # Stereo audio
BUFFER_SIZE = 1024  # Number of frames per buffer

# Create an output stream globally (to be accessed in the callback)
output_stream = sd.OutputStream(samplerate=SAMPLE_RATE, channels=CHANNELS)
output_stream.start()  # Start the output stream
def spectral_subtraction(noisy_signal, noise_estimate):
    """Perform basic spectral subtraction for noise suppression."""
    noisy_fft = np.fft.rfft(noisy_signal)
    noise_fft = np.fft.rfft(noise_estimate)
    
    clean_fft = np.maximum(noisy_fft - noise_fft, 0)  # Subtract noise spectrum
    clean_signal = np.fft.irfft(clean_fft)  # Convert back to time domain
    return clean_signal
def input_callback(indata, frames, time, status):
    """Capture audio from the input and send it to the output stream."""
    if status:
        print(status)  # Print errors if any
    output_stream.write(indata)  # Write captured audio to output

# Open an InputStream and use the callback
with sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, callback=input_callback, blocksize=BUFFER_SIZE):
    print("🎙️ Real-time Audio Streaming Started... Press Ctrl+C to Stop.")
    try:
        while True:
            pass  # Keep running until manually stopped
    except KeyboardInterrupt:
        print("\n🔇 Streaming Stopped.")

# Close the output stream properly when done
output_stream.stop()
output_stream.close()
