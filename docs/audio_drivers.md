
# Audio drivers

This page documents the audio input/output plugins in `perception_driver_audio`.

## Concepts

- Audio data is represented as `perception::audio_data` (used inside drivers) and `perception_msgs/msg/PerceptionAudio` (used on ROS topics/services).
- Device acquisition uses PortAudio.
- Plugins are loaded by the perception server using pluginlib and interact through the `DriverBase` interface.

## MicrophoneAudioDriver

Class: `perception::MicrophoneAudioDriver` (PortAudio input)

### Device acquisition

- Parameter `driver.audio.MicrophoneAudioDriver.device_name` is resolved to a PortAudio device id via `getDeviceIdByName()`.
- The driver opens and starts a PortAudio input stream.
- A background thread captures audio into an internal buffer; `getDataStream()` waits until at least one chunk is available.

### Parameters

- `driver.audio.MicrophoneAudioDriver.name` (string)
- `driver.audio.MicrophoneAudioDriver.device_name` (string)
- `driver.audio.MicrophoneAudioDriver.chunk_size` (int)
- `driver.audio.MicrophoneAudioDriver.sample_rate` (int)
- `driver.audio.MicrophoneAudioDriver.channels` (int)
- `driver.audio.MicrophoneAudioDriver.buffer_time` (int, seconds)

### Typical usage

- The perception server calls `getDataStream()` repeatedly to build a continuous audio feed.
- If transcription is enabled, the server also keeps an internal rolling buffer for a configurable duration.

## SpeakerAudioDriver

Class: `perception::SpeakerAudioDriver` (PortAudio output)

### Device acquisition

- Parameter `driver.audio.SpeakerAudioDriver.device_name` is resolved to a PortAudio device id via `getDeviceIdByName()`.
- The driver opens output streams on-demand keyed by sample format/rate/channels.

### Parameters

- `driver.audio.SpeakerAudioDriver.name` (string)
- `driver.audio.SpeakerAudioDriver.device_name` (string)
- `driver.audio.SpeakerAudioDriver.sample_rate` (int)
- `driver.audio.SpeakerAudioDriver.channels` (int)
- `driver.audio.SpeakerAudioDriver.test_file_path` (string)

### Typical usage

- The perception server can subscribe to an audio topic and forward it to this driver.
- The perception server can also route synthesized speech audio to this driver when speech requests set `use_device_audio=true`.

