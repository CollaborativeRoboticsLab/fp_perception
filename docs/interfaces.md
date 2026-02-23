
# ROS interfaces

This document describes the topics and services exposed/consumed by the perception server and how they map to the plugin drivers.

## Messages

### `perception_msgs/msg/PerceptionAudio`

Fields:

- `std_msgs/Header header`
- `int16[] samples`
- `int32 sample_rate`
- `int32 channels`
- `int32 chunk_size`
- `int32 chunk_count`
- `bool override`

Usage notes:

- `samples` are signed 16-bit PCM.
- The server publishes this message when `interface.audio_input.publish` is enabled.
- The server can also subscribe to audio output and forward it into the speaker driver.

### `perception_msgs/msg/PerceptionText`

Fields:

- `std_msgs/Header header`
- `string text`
- `string voice`
- `string instructions`

Usage notes:

- Used as the input to the speech synthesis service.
- `voice`/`instructions` are optional; drivers may apply defaults.

## Services

### `perception_msgs/srv/PerceptionTranscribe`

Request:

- `PerceptionAudio audio`
- `bool use_device_audio`
- `int32 device_buffer_time`

Response:

- `std_msgs/Header header`
- `string transcription`
- `bool success`

The server uses `use_device_audio` to decide whether to use its internal microphone buffer or the request-provided `audio`.
When `use_device_audio=true`, `device_buffer_time` controls how many seconds of audio the server collects from its public ring buffer.

### `perception_msgs/srv/PerceptionSpeech`

Request:

- `PerceptionText input`
- `bool use_device_audio`

Response:

- `bool success`
- `PerceptionAudio audio`

When `use_device_audio` is `true`, the server routes the synthesized audio to the speaker driver instead of returning it.

### `perception_msgs/srv/PerceptionSentiment`

Request:

- `std_msgs/Header header`
- `string text`
- `bool use_device_audio`
- `int32 device_buffer_time`

Response:

- `string label`
- `float64 score`

When `use_device_audio` is `true`, the server transcribes its microphone buffer first and then runs sentiment analysis on the transcribed text.

### `perception_msgs/srv/PerceptionImageAnalysis`

Request:

- `std_msgs/Header header`
- `sensor_msgs/Image image`
- `string prompt`
- `bool use_device_vision`

Response:

- `string response`

When `use_device_vision` is `true`, the server pulls the latest frame from the configured vision driver instead of using the request-provided `image`.

