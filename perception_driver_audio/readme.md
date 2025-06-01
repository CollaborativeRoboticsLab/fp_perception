# Perception Driver Audio

`perception_driver_audio` provides audio input and output drivers for ROS2 using the [PortAudio](http://www.portaudio.com/) library. It defines two main drivers built on a shared `DriverBase`:

- `MicrophoneAudioDriver`: Captures audio from the system microphone
- `SpeakerAudioDriver`: Plays back audio to the system's default speaker

These drivers are intended to be used within the `perception` ROS2 package and integrate with the event logging and configuration system defined by the `DriverBase` interface.

## Features

- Capture audio in real-time from the microphone
- Play audio buffers through the default output device
- Built on `PortAudio` for cross-platform low-latency audio I/O
- Compatible with ROS2 `rclcpp` and `std::any`-based driver model
- Easily extendable for streaming, audio topics, or speech modules

## Dependencies

Install PortAudio on Ubuntu:

```bash
sudo apt update
sudo apt install libportaudio2 libportaudio-dev python3-pyaudio
```

To use `find_device.py` to identify availabe devices, install 
## Parameters

```yaml
driver:
  audio:
    MicrophoneAudioDriver:
      name: MicrophoneAudioDriver
      device_id: 0
    SpeakerAudioDriver:
      name: SpeakerAudioDriver
      device_id: 0
```

## Usage

This module is not meant to be run as a standalone node. Instead, it is designed to be **loaded as a plugin** or instantiated via your own driver management system in ROS2.

Example integration:

```cpp
auto driver = std::make_shared<perception::MicrophoneAudioDriver>();
driver->start(node, config);
auto data = driver->getData();  // returns std::vector<int16_t> in std::any
```

To play audio using `SpeakerAudioDriver`:

```cpp
perception::SpeakerAudioDriver speaker;
speaker.start(node, config);
speaker.setData(audio_any);  // audio_any must contain std::vector<int16_t>
```

## Notes

- Current implementation uses blocking calls (e.g., `Pa_ReadStream`) for simplicity.
- You can wrap these drivers in a ROS2 node and publish audio buffers over custom messages for real-time processing.

## To-Do
- [ ] Buffer size and sample rate are currently hardcoded to 256 samples at 44.1kHz but can be parameterized.