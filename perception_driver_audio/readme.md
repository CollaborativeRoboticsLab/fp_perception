# Perception Driver Audio

`perception_driver_audio` provides audio input and output drivers for ROS2 using the [PortAudio](http://www.portaudio.com/) library. It defines two main drivers built on a shared `DriverBase`:

- `MicrophoneAudioDriver`: Captures audio from the system microphone
- `SpeakerAudioDriver`: Plays back audio to the system's default speaker

These drivers are intended to be used within the `perception` ROS2 package and integrate with the event logging and configuration system defined by the `DriverBase` interface.

## Features

- Capture audio in real-time from the microphone
- Play audio buffers through the default output device
- Built on `PortAudio` for cross-platform low-latency audio I/O
- Uses typed driver interfaces loaded through pluginlib
- Publishes standard ROS 2 diagnostics on `/diagnostics` when `use_diagnostics=true`
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
driver->initialize(node);
auto data = driver->readChunk();
```

To play audio using `SpeakerAudioDriver`:

```cpp
perception::SpeakerAudioDriver speaker;
speaker.initialize(node);
speaker.play(audio_data);
```

## Notes

- Microphone capture and speaker playback use PortAudio callbacks.
- The perception server can route diagnostics to `/diagnostics` through `diagnostic_updater`.

## To-Do
- [ ] Add long-running runtime validation for simultaneous capture and playback.