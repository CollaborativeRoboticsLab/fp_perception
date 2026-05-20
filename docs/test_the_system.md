
# Test the system

This page is a practical, copy/paste guide to validate the perception server, audio device routing, generated WAV files, and ROS 2 service interfaces from a terminal.

## Prereqs

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

If the PortAudio log reports a different hardware device such as `hw:3,0`, update the `aplay` device accordingly:

```bash
aplay -D plughw:3,0 test/mic_test.wav
aplay -D plughw:3,0 test/speech.wav
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

```bash
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
	device_buffer_time: 3
}"
```

Notes:

- `audio` is ignored when `use_device_audio: true`, but it must still be present to satisfy the request type.
- `device_buffer_time` must be **≤** the configured server ring buffer duration (`interface.audio_input.buffer_duration`).
- If the request header stamp is zero, the server uses the latest buffered audio window.

## Sentiment service (device audio)

Default service name is typically `perception/sentiment_analysis`.

If `use_device_audio: true`, the server will:

1) wait until the public audio buffer has `device_buffer_time` seconds of new audio
2) transcribe it
3) run sentiment on the transcribed text

```bash
ros2 service call /perception/sentiment_analysis perception_msgs/srv/PerceptionSentiment "{
	header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
	text: '',
	use_device_audio: true,
	device_buffer_time: 3
}"
```

## Speech synthesis service (device audio playback)

Default service name is typically `perception/speech`.

If `use_device_audio: true`, the server will synthesize speech and play it through the configured speaker driver.

```bash
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

The response should also contain audio data. If the service succeeds but playback is unclear, save or inspect the generated `test/speech.wav` from startup tests and replay it with `aplay` as shown above.

## Troubleshooting

- If device-audio calls time out, ensure the microphone driver is enabled and producing samples.
- If you request a longer `device_buffer_time` than the server buffer duration, increase `interface.audio_input.buffer_duration`.
- If you don’t hear speech output with `use_device_audio: true`, ensure the speaker driver is enabled and the container can access an output device.
- If `aplay` works but the speaker driver does not, compare the `aplay -D plughw:X,Y` route against the `hw:X,Y` shown in the PortAudio resolved-device log.
- If `test/mic_test.wav` is silent, confirm the microphone input source is selected in the host audio settings and rerun `ros2 launch perception server.launch.py` with `run_tests: true`.

## Image analysis service (device vision)

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

- Requires `use_vision_driver: true` and `interface.image_analysis.provide_service: true` in config.
- `image` is ignored when `use_device_vision: true`, but must still be present to satisfy the request type.

## Image analysis service (external image file)

`ros2 service call` can pass `sensor_msgs/Image`, but it’s cumbersome to paste raw `data` bytes.
For an external image file test, use this minimal Python snippet:

```bash
python3 - <<'PY'
import sys
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from perception_msgs.srv import PerceptionImageAnalysis

class Client(Node):
	def __init__(self):
		super().__init__('image_analysis_test_client')
		self.cli = self.create_client(PerceptionImageAnalysis, '/perception/image_analysis')

def main(path, prompt):
	rclpy.init()
	node = Client()
	if not node.cli.wait_for_service(timeout_sec=10.0):
		raise RuntimeError('service /perception/image_analysis not available')

	img = cv2.imread(path, cv2.IMREAD_COLOR)
	if img is None:
		raise RuntimeError(f'failed to read image: {path}')

	req = PerceptionImageAnalysis.Request()
	req.prompt = prompt
	req.use_device_vision = False
	req.image = CvBridge().cv2_to_imgmsg(img, encoding='bgr8')

	future = node.cli.call_async(req)
	rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
	if future.result() is None:
		raise RuntimeError('no response')
	print(future.result().response)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	path = sys.argv[1] if len(sys.argv) > 1 else 'test/image.png'
	prompt = sys.argv[2] if len(sys.argv) > 2 else "What's in this image?"
	main(path, prompt)
PY
```

