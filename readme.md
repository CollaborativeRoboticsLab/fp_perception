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

## Start the system

If using microphone or speaker run the following code to find the required device id

```sh
python3 src/perception/perception_driver_audio/find_devices.py
```

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
