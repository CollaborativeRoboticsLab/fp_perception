
# Test the system

This page is a practical, copy/paste guide to validate the ROS 2 service interfaces from a terminal.

## Prereqs

In one terminal, build and launch the server:

```bash
cd ~/colcon_ws
. install/setup.sh

# Example launch
ros2 launch perception server.launch.py
```

In a second terminal, source the workspace:

```bash
cd ~/colcon_ws
. install/setup.sh
```

Optional sanity checks:

```bash
ros2 service list | grep perception
ros2 interface show perception_msgs/srv/PerceptionTranscribe
ros2 interface show perception_msgs/srv/PerceptionSentiment
ros2 interface show perception_msgs/srv/PerceptionSpeech
ros2 interface show perception_msgs/srv/PerceptionImageAnalysis
```

## Transcription service (device audio)

Default service name is typically `perception/transcription`.

This records from the server’s public audio buffer for `device_buffer_time` seconds and transcribes it.

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

## Troubleshooting

- If device-audio calls time out, ensure the microphone driver is enabled and producing samples.
- If you request a longer `device_buffer_time` than the server buffer duration, increase `interface.audio_input.buffer_duration`.
- If you don’t hear speech output with `use_device_audio: true`, ensure the speaker driver is enabled and the container can access an output device.

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

