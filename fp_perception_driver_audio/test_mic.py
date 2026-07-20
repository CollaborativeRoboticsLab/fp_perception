import pyaudio
import wave

def record_microphone(filename="test_mic_python.wav", duration=5, rate=44100, channels=1, chunk=1024, device_index=None):
    p = pyaudio.PyAudio()

    if device_index is not None:
        try:
            dev_info = p.get_device_info_by_index(device_index)
            print(f"🎙️ Using input device [{device_index}]: {dev_info['name']}")
        except Exception as e:
            print(f"❌ Failed to get device info: {e}")
            p.terminate()
            return

    print("🎤 Recording started. Speak now...")

    stream = p.open(format=pyaudio.paInt16,
                    channels=channels,
                    rate=rate,
                    input=True,
                    input_device_index=device_index,
                    frames_per_buffer=chunk)

    frames = []

    for _ in range(0, int(rate / chunk * duration)):
        data = stream.read(chunk)
        frames.append(data)

    print("✅ Recording finished. Saving to file...")

    stream.stop_stream()
    stream.close()
    p.terminate()

    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(rate)
        wf.writeframes(b''.join(frames))

    print(f"🎧 File saved as {filename}")

if __name__ == "__main__":
    # Replace with your desired device ID after listing devices
    record_microphone(device_index=5,
                      duration=5, 
                      rate=48000,
                      channels=2,
                      chunk=1024)
