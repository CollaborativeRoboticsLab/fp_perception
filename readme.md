# Perception

## Overview

The `perception` package provides a **standardized, plugin-based architecture** for managing **drivers** (and related processing) that power perception systems in **human-robot interaction (HRI)** scenarios.

It enables modular development and dynamic loading of components at runtime using ROS 2's [`pluginlib`](https://github.com/ros/pluginlib), simplifying integration, testing, and extension of perception pipelines.

## Documentation

Start here:

- [Server plugin loading and device and data acquisition](docs/server_system.md) 
- [ROS Topic, Service and Action Interfaces](docs/interfaces.md)
- [Testing the System and Interfaces](docs/test_the_system.md)
- [Base Classes](docs/base_classes.md)

Drivers:

- [Audio Drivers and AudioData Acquisition](docs/audio_drivers.md) 
- [Vision Drivers and ImageData Acquisition](docs/vision_drivers.md)
- [Transcription Drivers and Transcription Service](docs/transribe_drivers.md)
- [Speech Drivers and Speech Synthesis](docs/speech_drivers.md)
- [Sentiment Drivers and Sentiment Analysis](docs/sentiment_drivers.md)
- [Image Analysis Drivers](docs/image_analysis_drivers.md)


## Key Features

* **Plugin-based driver framework**
  Integrate new sensors and drivers without changing the core system.

* **Reusable base classes** for Physical device drivers (`DriverBase`) and REST-backed drivers (`RestBase`)

* Designed with **HRI** in mind – enabling perception for robots that does not have built-in perception capabilities.

## Plugins

### Driver Plugins

- `DriverBase` is used to implement drivers that interact with perception hardware (e.g., cameras, audio devices).
- `RestBase` is used to implement drivers that interact with REST APIs (e.g., OpenAI, HuggingFace).

- Driver READMEs:
  - [perception_driver_vision/readme.md](perception_driver_vision/readme.md)
  - [perception_driver_audio/readme.md](perception_driver_audio/readme.md)

- Plugin Configuration:
  - [docs/plugin_configuration.md](docs/plugin_configuration.md)

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
sudo apt install libportaudio2 portaudio19-dev python3-pyaudio ros-${ROS_DISTRO}-vision-opencv
```

for any missing dependencies
```sh
rosdep install --from-paths src --ignore-src -r -y
```

build the workspace
```sh
colcon build
```

## Start the system

If using microphone or speaker, configure the audio drivers by `device_name` when possible. The test guide shows how to confirm the selected PortAudio device and map it to an ALSA `plughw:X,Y` route for direct WAV playback checks.

```sh
python3 src/perception/perception_driver_audio/find_devices.py
```

See [Test the System](docs/test_the_system.md) for startup validation, generated WAV checks, and service-call examples.

```sh
export OPENAI_API_KEY=
export HUGGINGFACE_API_KEY=
source install/setup.bash
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
