
# Test the system

This page is a practical, copy/paste guide to validate the perception server, audio device routing, generated WAV files, and ROS 2 service interfaces from a terminal.

## Prerequisites

In one terminal, build, source, and launch the server:

```bash
export OPENAI_API_KEY=
export HUGGINGFACE_API_KEY=

source install/setup.bash
ros2 launch perception server.launch.py
```

In a second terminal, source the workspace:

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

The default development config has `run_tests: true`, so launch will exercise the loaded plugins before the server settles into normal operation.

Optional sanity checks:

```bash
ros2 service list | grep perception
ros2 interface show perception_msgs/srv/PerceptionTranscribe
ros2 interface show perception_msgs/srv/PerceptionSentiment
ros2 interface show perception_msgs/srv/PerceptionSpeech
ros2 interface show perception_msgs/srv/PerceptionImageAnalysis
```

## Startup device and plugin validation

When the server starts, first verify that the microphone and speaker select the intended PortAudio device by name.

Expected log shape:

```text
PortAudio device count: ...
PortAudio device 5 ['HD-Audio Generic: ALC285 Analog (hw:2,0)'] ... max_input=2 max_output=2 ...
Resolved microphone device_name 'ALC285 Analog' to device_id 5.
Assigned driver device: PortAudio device 5 ['HD-Audio Generic: ALC285 Analog (hw:2,0)'] ...
Resolved speaker device_name 'ALC285 Analog' to device_id 5.
Assigned driver device: PortAudio device 5 ['HD-Audio Generic: ALC285 Analog (hw:2,0)'] ...
```

Notes:

- ALSA/JACK warnings during PortAudio device enumeration are common in containers. They are not a failure if the driver later resolves the expected device and starts successfully.
- The `hw:2,0` part tells you the ALSA hardware route. Use that as `plughw:2,0` for direct `aplay` checks.
- The numeric PortAudio `device_id` may change between machines or boots; the config should prefer `device_name`.

With `run_tests: true`, the launch log should also show the built-in checks:

```text
Testing microphone driver...
Microphone test signal stats: samples=240000 ...
Audio data written to file: test/mic_test.wav
Testing speaker driver...
Audio data queued to stream: int16_48000_2
Testing transcription driver...
Transcription result: Hello
Testing speech synthesis driver...
Speech synthesis result received and saved to test/speech.wav
Testing sentiment driver...
Analysis results with sentiment: POSITIVE ...
Testing image analysis driver...
Image analysis result: ...
```

After the startup tests finish, verify the generated WAV files directly through ALSA from another terminal:

```bash
cd ~/colcon_ws

# find the device name and route for your speaker from the PortAudio log, e.g. 'ALC285 Analog' with 'hw:3,0' route
python3 src/perception/perception_driver_audio/find_devices.py

# Confirm the microphone test recording is audible.
aplay -D plughw:2,0 test/mic_test.wav

# Confirm the speech synthesis output file is audible.
aplay -D plughw:2,0 test/speech.wav
```

Expected output looks like:

```text
Playing WAVE 'test/mic_test.wav' : Signed 16 bit Little Endian, Rate 48000 Hz, Mono
Playing WAVE 'test/speech.wav' : Signed 16 bit Little Endian, Rate 24000 Hz, Mono
```

## Live audio topic check

The microphone publisher should stream captured chunks on the configured audio topic.

```bash
ros2 topic hz /perception/microphone
ros2 topic echo /perception/microphone --once
```

You should see a nonzero publish rate and messages with populated `samples`. The default config publishes at `interface.audio_input.frequency`, typically 10 Hz.

## Transcription service (device audio)

Default service name is typically `perception/transcription`.

This reads from the server's public audio buffer for `device_buffer_time` seconds and transcribes it. Speak into the microphone immediately before or during the call.

It only works after the microphone driver has initialized the public audio buffer; otherwise the service returns `Device audio not available: public audio buffer not initialized`.

Latest-buffer smoke test:

```bash
source install/setup.bash
ros2 service call /perception/transcription perception_msgs/srv/PerceptionTranscribe "{
  audio: {
    header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
    sample_rate: 0,
    channels: 0,
    chunk_size: 0,
    chunk_count: 0,
    samples: []
  },
  use_device_audio: true,
  device_buffer_time: 5
}"
```

Timestamp-window smoke test:

```bash
STAMP_SEC=$(date +%s)
STAMP_NSEC=$(date +%N)
echo "Speak for the next 5 seconds..."
sleep 5

source install/setup.bash
ros2 service call /perception/transcription perception_msgs/srv/PerceptionTranscribe "{
  audio: {
    header: {stamp: {sec: ${STAMP_SEC}, nanosec: ${STAMP_NSEC}}, frame_id: ''},
    sample_rate: 0,
    channels: 0,
    chunk_size: 0,
    chunk_count: 0,
    samples: []
  },
  use_device_audio: true,
  device_buffer_time: 5
}"
```

### Parameters and expected behavior

- `audio` is ignored when `use_device_audio: true`, but it must still be present to satisfy the request type.
- `device_buffer_time` must be **≤** the configured server ring buffer duration (`interface.audio_input.buffer_duration`).

- If the request header stamp is zero, the server uses the latest buffered audio window.
- If the timestamped window is only partially available, the server returns the available overlap and logs a warning.
- If the timestamped window has no overlap with the ring buffer, the server logs a warning and falls back to the latest buffered audio instead of failing the node.

## Sentiment service (device audio)

Default service name is typically `perception/sentiment_analysis`.

If `use_device_audio: true`, the server will:

1) wait until the public audio buffer has `device_buffer_time` seconds of new audio
2) transcribe it
3) run sentiment on the transcribed text

The response includes `analyzed_text`, which is the exact text that was sent into sentiment analysis. This is useful for debugging device-audio runs where the sentiment label looks plausible but the upstream transcription may be wrong.

Latest-buffer smoke test:

```bash
source install/setup.bash
ros2 service call /perception/sentiment_analysis perception_msgs/srv/PerceptionSentiment "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  text: '',
  use_device_audio: true,
  device_buffer_time: 5
}"
```

Timestamp-window smoke test:

```bash
STAMP_SEC=$(date +%s)
STAMP_NSEC=$(date +%N)
echo "Speak with a positive or negative phrase for the next 5 seconds..."
sleep 5

source install/setup.bash
ros2 service call /perception/sentiment_analysis perception_msgs/srv/PerceptionSentiment "{
  header: {stamp: {sec: ${STAMP_SEC}, nanosec: ${STAMP_NSEC}}, frame_id: ''},
  text: '',
  use_device_audio: true,
  device_buffer_time: 5
}"
```

## Speech synthesis service (device audio playback)

Default service name is typically `perception/speech`.

If `use_device_audio: true`, the server will synthesize speech and play it through the configured speaker driver.

Device-playback smoke test:

```bash
source install/setup.bash
ros2 service call /perception/speech perception_msgs/srv/PerceptionSpeech "{
  input: {
    header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
    text: 'Hello from perception',
    voice: '',
    instructions: ''
  },
  use_device_audio: true
}"
```

With `use_device_audio: true`, the main success signal is audible playback through the configured speaker. If playback is unclear, inspect the startup-generated `test/speech.wav` and replay it with `aplay` as shown above.

Expected outcome for these three service smoke tests:

- transcription returns `success: true` and a non-empty `transcription`
- sentiment returns a label such as `POSITIVE` or `NEGATIVE` with a confidence score, plus `analyzed_text` for debugging
- speech returns `success: true` and audible playback through the configured speaker

### Troubleshooting

- If device-audio calls time out, ensure the microphone driver is enabled and producing samples.
- If transcription returns `Device audio not available: public audio buffer not initialized`, verify the microphone driver is running and `ros2 topic hz /perception/microphone` shows samples before calling the service.
- If you request a longer `device_buffer_time` than the server buffer duration, increase `interface.audio_input.buffer_duration`.
- If you don’t hear speech output with `use_device_audio: true`, ensure the speaker driver is enabled and the container can access an output device.
- If `aplay` works but the speaker driver does not, compare the `aplay -D plughw:X,Y` route against the `hw:X,Y` shown in the PortAudio resolved-device log.
- If `test/mic_test.wav` is silent, confirm the microphone input source is selected in the host audio settings and rerun `ros2 launch perception server.launch.py` with `run_tests: true`.

### Image analysis service (device vision)

The system is tested with Realsense D435 Camera. So the current devcontainer includes the `realsense2_camera` ROS package and a launch file to start the camera node. 

```bash
ros2 launch realsense2_camera rs_launch.py
```

Default service name is typically `perception/image_analysis`.

This is the easiest way to test from the CLI because you don't need to embed image bytes into the request.

```bash
ros2 service call /perception/image_analysis perception_msgs/srv/PerceptionImageAnalysis "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  image: {
    header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
    height: 0,
    width: 0,
    encoding: '',
    is_bigendian: 0,
    step: 0,
    data: []
  },
  prompt: 'Describe the most important objects in this image',
  use_device_vision: true
}"
```

Notes:

- Requires `interface.image_analysis.provide_service: true` and at least one enabled vision driver such as `use_ros_vision_driver: true` or `use_non_ros_vision_driver: true` in config.
- `image` is ignored when `use_device_vision: true`, but must still be present to satisfy the request type.
