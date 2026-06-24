
# Audio drivers

This page documents the audio input/output plugins in `perception_driver_audio`.

## Concepts

- Audio data is represented as `perception::audio_data` (used inside drivers) and `perception_msgs/msg/PerceptionAudio` (used on ROS topics/services).
- Device acquisition uses PortAudio.
- Plugins are loaded by the perception server using pluginlib and used through typed audio interfaces.

## MicrophoneAudioDriver

Class: `perception::MicrophoneAudioDriver` (PortAudio input)

### Device acquisition

- Device selection uses `driver.audio.MicrophoneAudioDriver.device_name` first.
- `driver.audio.MicrophoneAudioDriver.device_id` is an optional fallback PortAudio device index; leave it unset for name-based or default-device selection.
- The driver opens and starts a PortAudio input stream.
- A PortAudio callback captures audio into an internal buffer; `readChunk()` waits until at least one chunk is available.

### Parameters

- `driver.audio.MicrophoneAudioDriver.name` (string)
- `driver.audio.MicrophoneAudioDriver.device_name` (string)
- `driver.audio.MicrophoneAudioDriver.device_id` (int, optional fallback)
- `driver.audio.MicrophoneAudioDriver.chunk_size` (int)
- `driver.audio.MicrophoneAudioDriver.sample_rate` (int)
- `driver.audio.MicrophoneAudioDriver.channels` (int)
- `driver.audio.MicrophoneAudioDriver.buffer_time` (int, seconds)

### Typical usage

- The perception server calls `readChunk()` repeatedly to build a continuous audio feed.
- If transcription is enabled, the server also keeps an internal rolling buffer for a configurable duration.
- If `use_diagnostics=true`, the driver publishes capture health on `/diagnostics` via `diagnostic_updater`.

## SpeakerAudioDriver

Class: `perception::SpeakerAudioDriver` (PortAudio output)

### Device acquisition

- Device selection uses `driver.audio.SpeakerAudioDriver.device_name` first.
- `driver.audio.SpeakerAudioDriver.device_id` is an optional fallback PortAudio device index; leave it unset for name-based or default-device selection.
- The driver opens output streams on-demand keyed by sample format/rate/channels.

### Parameters

- `driver.audio.SpeakerAudioDriver.name` (string)
- `driver.audio.SpeakerAudioDriver.device_name` (string)
- `driver.audio.SpeakerAudioDriver.device_id` (int, optional fallback)
- `driver.audio.SpeakerAudioDriver.sample_rate` (int)
- `driver.audio.SpeakerAudioDriver.channels` (int)
- `driver.audio.SpeakerAudioDriver.test_file_path` (string)

### Typical usage

- The perception server can subscribe to an audio topic and forward it to this driver.
- The perception server can also route synthesized speech audio to this driver when speech requests set `use_device_audio=true`.
- If `use_diagnostics=true`, the driver publishes playback health on `/diagnostics` via `diagnostic_updater`.

