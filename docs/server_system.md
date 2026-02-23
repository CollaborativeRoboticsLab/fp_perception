
# Server system

The perception server (`perception::PerceptionServer`) is a ROS 2 node that loads perception "drivers" as pluginlib plugins and exposes a simple set of ROS interfaces (topics and services) for audio, vision, transcription, speech synthesis, sentiment, and image analysis.

This document focuses on:

- the main concepts
- how devices are acquired and used
- the server’s ROS interfaces and configuration

## Main concepts

### Drivers are plugins

- Drivers implement `perception::DriverBase`.
- The server loads drivers at runtime using `pluginlib::ClassLoader<perception::DriverBase>`.
- Each driver is selected by a fully-qualified class name parameter (for example `perception::MicrophoneAudioDriver`).

### Uniform data exchange

- Drivers exchange data through the `DriverBase` methods using `std::any`.
- The server converts between ROS messages (`perception_msgs/*`) and internal structs (`perception::audio_data`, `perception::text_data`).

### Device acquisition lives in the plugins

The server orchestrates and routes data, but the actual device I/O happens in the plugins:

- Microphone / speaker use PortAudio and resolve devices by name.
- Non-ROS vision uses OpenCV device capture (`cv::VideoCapture`).
- ROS vision subscribes to a topic via `image_transport`.

## What the server runs

### Background threads

If enabled by parameters, the server starts periodic loops:

- `publishAudio()`: pulls microphone audio chunks and optionally maintains a rolling buffer for transcription; can also publish audio to a ROS topic.
- `publishVideo()`: pulls frames from a vision driver and optionally publishes them as `sensor_msgs/msg/Image`.

### Services

If enabled by parameters, the server provides:

- transcription (`PerceptionTranscribe`)
- speech synthesis (`PerceptionSpeech`)
- sentiment analysis (`PerceptionSentiment`)
- image analysis (`PerceptionImageAnalysis`)

## Parameters

### Feature enable flags

The server uses boolean flags to decide which drivers to load:

- `use_vision_driver`
- `use_microphone_driver`
- `use_speaker_driver`
- `use_transcription_driver`
- `use_speech_driver`
- `use_sentiment_driver`
- `use_image_analysis_driver`

And:

- `run_tests`: if true, calls `test()` on each loaded driver during startup.

### Plugin selection (which class to load)

These parameters select which pluginlib class gets loaded:

- `ros_vision_driver` (default `perception::DefaultDriver`)
- `non_ros_vision_driver` (default `perception::OpenCVDriver`)
- `microphone_driver` (default `perception::MicrophoneAudioDriver`)
- `speaker_driver` (default `perception::SpeakerAudioDriver`)
- `transcription_driver` (default `perception::OpenAIDriver`)
- `speech_synthesis_driver` (default `perception::OpenAISpeechDriver`)
- `sentiment_driver` (default `perception::SentimentDriver`)
- `image_analysis_driver` (default `perception::OpenAIImageAnalysisDriver`)

Note: when `use_vision_driver=true`, the current server code loads **both** the ROS vision driver and the non-ROS vision driver; `interface.vision_input.non_ros` controls which one is used for publishing.

### ROS interface configuration

#### Audio input topic (optional publish)

- `interface.audio_input.publish` (bool)
- `interface.audio_input.topic` (string)
- `interface.audio_input.frame_id` (string)
- `interface.audio_input.frequency` (int)

If enabled, the server publishes `perception_msgs/msg/PerceptionAudio` at the configured rate.

#### Audio output topic (optional subscribe)

- `interface.audio_output.subscribe` (bool)
- `interface.audio_output.topic` (string)

If enabled, the server subscribes to `PerceptionAudio` and forwards audio samples into the speaker driver.

#### Transcription service

- `interface.transcription.provide_service` (bool)
- `interface.transcription.service` (string)
- `interface.transcription.buffer_duration` (int, seconds)

The rolling microphone buffer is sized as:

$$\text{max\_samples} = sample\_rate \times channels \times buffer\_duration$$

When the buffer exceeds this size, the server drops the oldest samples and keeps the latest window.

#### Speech service

- `interface.speech.provide_service` (bool)
- `interface.speech.service_name` (string)

#### Sentiment service

- `interface.sentiment.provide_service` (bool)
- `interface.sentiment.service_name` (string)

#### Image analysis service

- `interface.image_analysis.provide_service` (bool)
- `interface.image_analysis.service_name` (string)

#### Vision publish

- `interface.vision_input.publish` (bool)
- `interface.vision_input.topic` (string)
- `interface.vision_input.frame_id` (string)
- `interface.vision_input.frequency` (int)
- `interface.vision_input.non_ros` (bool)

If `non_ros=true`, the server publishes frames from the OpenCV driver; otherwise it republishes frames from the ROS image subscriber driver.

## End-to-end flows

### Device microphone -> transcription service

1. Microphone plugin acquires audio from PortAudio.
2. Server accumulates a rolling `transcription_buffer_`.
3. A client calls `PerceptionTranscribe` with `use_device_audio=true`.
4. Server calls the transcription driver with the buffer and returns the text.

### Text -> speech

1. Client calls `PerceptionSpeech` with `PerceptionText`.
2. Server calls the speech driver.
3. If `use_device_audio=true`, server forwards synthesized audio to the speaker driver.
4. Otherwise, server returns synthesized audio in the service response.

### Device microphone -> sentiment

1. Client calls `PerceptionSentiment` with `use_device_audio=true`.
2. Server transcribes the rolling audio buffer.
3. Server runs sentiment on the transcription and returns `(label, score)`.

### Device vision or external image -> image analysis

1. Client calls `PerceptionImageAnalysis` with a `prompt`.
2. If `use_device_vision=true`, server captures a frame from the configured vision driver; otherwise it uses the request-provided `sensor_msgs/Image`.
3. Server calls the image analysis driver and returns the model output text.

