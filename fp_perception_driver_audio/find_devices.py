import pyaudio

def list_audio_devices():
    pa = pyaudio.PyAudio()

    default_input_index = pa.get_default_input_device_info()["index"]
    default_output_index = pa.get_default_output_device_info()["index"]

    print("\n🎤 Input Devices:\n")
    for i in range(pa.get_device_count()):
        info = pa.get_device_info_by_index(i)
        if info['maxInputChannels'] > 0:
            print(f"Device ID: {i}")
            print(f"  Name       : {info['name']}")
            print(f"  Channels   : {info['maxInputChannels']}")
            print(f"  Sample Rate: {info['defaultSampleRate']}")
            print(f"  Host API   : {pa.get_host_api_info_by_index(info['hostApi'])['name']}")
            if i == default_input_index:
                print("  ⚠️  Default Input Device")
            print()

    print("🔈 Output Devices:\n")
    for i in range(pa.get_device_count()):
        info = pa.get_device_info_by_index(i)
        if info['maxOutputChannels'] > 0:
            print(f"Device ID: {i}")
            print(f"  Name       : {info['name']}")
            print(f"  Channels   : {info['maxOutputChannels']}")
            print(f"  Sample Rate: {info['defaultSampleRate']}")
            print(f"  Host API   : {pa.get_host_api_info_by_index(info['hostApi'])['name']}")
            if i == default_output_index:
                print("  ⚠️  Default Output Device")
            print()

    pa.terminate()

if __name__ == "__main__":
    list_audio_devices()