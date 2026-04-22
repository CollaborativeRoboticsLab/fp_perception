# Perception

## Overview

The `perception` package provides a **standardized, plugin-based architecture** for managing **drivers** (and related processing) that power perception systems in **human-robot interaction (HRI)** scenarios.

It enables modular development and dynamic loading of components at runtime using ROS 2's [`pluginlib`](https://github.com/ros/pluginlib), simplifying integration, testing, and extension of perception pipelines.

## Documentation

Start here:

- [Server](docs/server_system.md) — how the server loads plugins, acquires devices, and routes data
- [ROS Interface](docs/interfaces.md) — topics/services and message/service definitions
- [Base Classes](docs/base_classes.md) — `DriverBase` and `RestBase`

Drivers:

- [Audio Drivers](docs/audio_drivers.md)
- [Vision Drivers](docs/vision_drivers.md)
- [Transcription Drivers](docs/transribe_drivers.md)
- [Speech Drivers](docs/speech_drivers.md)
- [Sentiment Drivers](docs/sentiment_drivers.md)
- [Image Analysis Drivers](docs/image_analysis_drivers.md)


## Key Features

* **Plugin-based driver framework**
  Integrate new sensors and drivers without changing the core system.

* **Reusable base classes** for drivers (`DriverBase`) and REST-backed drivers (`RestBase`)

* Designed with **HRI** in mind – enabling perception for social, assistive, and interactive robots.

* Server exposes a small set of **ROS topics/services** for audio, vision, transcription, speech synthesis, and sentiment.
* Server also exposes **image analysis** as a ROS service (optional).


## Plugins

### Driver Plugins

- `DriverBase` is used to implement drivers that interact with perception hardware (e.g., cameras, audio devices).

- Package READMEs:
  - [perception_driver_vision/readme.md](perception_driver_vision/readme.md)
  - [perception_driver_audio/readme.md](perception_driver_audio/readme.md)


## Build the system 

Create the workspace folder
```sh
mkdir -p perception_ws/src
cd perception_ws/src
```

Clone the repository
```sh
git clone https://github.com/CollaborativeRoboticsLab/perception.git
```

Install dependencies
```sh
cd ..
sudo apt update
sudo apt install libportaudio2 portaudio19-dev python3-pyaudio ros-humble-vision-opencv
```

for any missing dependencies
```sh
rosdep install --from-paths src --ignore-src -r -y
```

build the workspace
```sh
colcon build
```

## Plugin configuration

The perception server reads parameters from [perception/config/config.yaml](perception/perception/config/config.yaml).

At a high level:

1. Enable the feature group with `use_*`.
2. Pick the plugin class to load (e.g. `microphone_driver: perception::MicrophoneAudioDriver`).
3. Configure the ROS interfaces under `interface.*`.
4. Configure per-plugin parameters under `driver.*`.

```yaml
  # Enable/disable plugin groups
  use_microphone_driver: true
  use_speaker_driver: true
  use_transcription_driver: true
  use_speech_driver: true
  use_sentiment_driver: false
  use_vision_driver: false
  use_image_analysis_driver: false

  # Select which pluginlib classes to load
  microphone_driver: perception::MicrophoneAudioDriver
  speaker_driver: perception::SpeakerAudioDriver
  transcription_driver: perception::OpenAIDriver
  speech_synthesis_driver: perception::OpenAISpeechDriver
  sentiment_driver: perception::SentimentDriver
  image_analysis_driver: perception::OpenAIImageAnalysisDriver

  # Server ROS interfaces
  interface:
    audio_input:
      publish: true
      topic: perception/microphone
      frame_id: microphone_frame
      frequency: 10
    transcription:
      provide_service: true
      service: perception/transcription
      buffer_duration: 10
    speech:
      provide_service: true
      service_name: perception/speech
    sentiment:
      provide_service: false
      service_name: perception/sentiment_analysis

    image_analysis:
      provide_service: false
      service_name: perception/image_analysis

  # Per-plugin parameters (maps to dot-separated params)
  driver:
    audio:
      MicrophoneAudioDriver:
        # PortAudio device index (set via find_devices.py)
        device_id: 0
        sample_rate: 48000
        channels: 2
        chunk_size: 48000
        buffer_time: 10
    transcription:
      OpenAIDriver:
        model: whisper-1
        rest:
          uri: https://api.openai.com/v1/audio/transcriptions

    image_analysis:
      OpenAIDriver:
        model: gpt-4.1
        detail: auto
        rest:
          uri: https://api.openai.com/v1/responses
```

build the workspace to update the configuration
```sh
colcon build
```

## Start the system

If using microphone or speaker run the following code to find the required device id

```sh
python3 src/perception/perception_driver_audio/find_devices.py
```

```sh
source install/setup.bash
export OPENAI_API_KEY=
export HUGGINGFACE_API_KEY=
ros2 launch perception server.launch.py
```

## Writing Your Own Plugins

1. Inherit from `perception::DriverBase` (or `perception::RestBase` for REST-backed drivers).
2. Implement required methods (`initialize`, `deinitialize`) and whichever data methods you need (`getData*`, `setData*`).
3. Register your plugin using `PLUGINLIB_EXPORT_CLASS`.
4. Define a plugin XML manifest.


## Future Extensions

- [ ] Dynamic runtime reconfiguration
- [ ] Built-in diagnostics
- [ ] Visualization tools for debugging perception pipelines
- [ ] Performance benchmarking and logging
