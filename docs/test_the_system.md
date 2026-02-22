
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

