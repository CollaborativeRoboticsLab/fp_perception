import pyaudio

def list_output_devices():
    p = pyaudio.PyAudio()
    print("Available Output Devices:\n")

    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info['maxOutputChannels'] > 0:
            print(f"Device ID: {i}")
            print(f"  Name       : {info['name']}")
            print(f"  Channels   : {info['maxOutputChannels']}")
            print(f"  Sample Rate: {info['defaultSampleRate']}")
            print(f"  Host API   : {p.get_host_api_info_by_index(info['hostApi'])['name']}")
            print()

    p.terminate()

if __name__ == "__main__":
    list_output_devices()
