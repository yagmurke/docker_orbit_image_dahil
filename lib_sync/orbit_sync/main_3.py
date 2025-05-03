import pyaudio

def test_microphone():
    p = pyaudio.PyAudio()

    # Open stream
    stream = p.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=44100,
                    input=True,
                    frames_per_buffer=1024)

    print("Recording...")

    try:
        while True:
            data = stream.read(1024)
            if data:
                print("Microphone is receiving audio.")
                
    except KeyboardInterrupt:
        pass

    print("Finished recording.")

    # Stop and close the stream
    stream.stop_stream()
    stream.close()
    p.terminate()

if __name__ == "__main__":
    test_microphone()