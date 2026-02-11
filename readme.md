# Perception

## Overview

The `perception` package provides a **standardized, plugin-based architecture** for managing **drivers** and **algorithms** that power perception systems in **human-robot interaction (HRI)** scenarios.

It enables modular development and dynamic loading of components at runtime using ROS 2's [`pluginlib`](https://github.com/ros/pluginlib), simplifying integration, testing, and extension of perception pipelines.


## Key Features

* **Plugin-based driver and algorithm framework**
  Easily integrate new sensors and processing modules without changing the core system.

* **Reusable base classes** for drivers (`DriverBase`) and algorithms (`AlgorithmBase`)

* Designed with **HRI** in mind – enabling perception for social, assistive, and interactive robots.

* Integrated **event publishing** using the `EventClient` abstraction for tracing and introspection.


## Plugins

### Driver Plugins

- `DriverBase` used to implement drivers that interact with perception hardware (e.g., cameras, audio devices and depth sensors).

- [Perception Vision Driver](https://github.com/CollaborativeRoboticsLab/perception/blob/main/perception_driver_vision/readme.md) plugins required to interact with visual data
- [Perception Audio Driver](https://github.com/CollaborativeRoboticsLab/perception/blob/main/perception_driver_audio/readme.md) plugins required to interact with visual data

### Algorithm Plugins

- `AlgorithmBase` used to implement perception algorithms (e.g., face detection, object tracking).

- [Perception Eye Gaze Detection](https://github.com/CollaborativeRoboticsLab/perception/blob/main/perception_detect_eye_gaze/readme.md) plugins required to interact with visual data

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
sudo apt install libportaudio2 portaudio19-dev ros-humble-vision-opencv 
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

Set the bool `true` to use a type of a plugin and then give the name of the specific plugin to load it.

```yaml
  use_vision_driver: true
  use_microphone_driver: false
  use_speaker_driver: false
  use_eye_gaze_algorithm: true
  
  # Uncomment the following lines to enable the respective drivers
  vision_driver: perception::DefaultDriver              # Default ROS driver for vision
  # vision_driver: perception::OpenCVDriver             # OpenCV driver for vision
  microphone_driver: perception::MicrophoneAudioDriver  # Driver for microphone audio
  speaker_driver: perception::SpeakerAudioDriver        # Driver for speaker audio
  eye_gaze_algorithm: perception::GazeAlgorithm         # Algorithm for eye gaze detection
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
ros2 launch perception server.launch.py
```

## Writing Your Own Plugins

1. Inherit from `perception::DriverBase` or `perception::AlgorithmBase`.
2. Implement required methods (`initialize`, `start`, `stop`, `getData`, `getDataStream`, `setData`, `setDataStream`).
3. Register your plugin using `PLUGINLIB_EXPORT_CLASS`.
4. Define a plugin XML manifest.


## Future Extensions

- [ ] Dynamic runtime reconfiguration
- [ ] Built-in diagnostics
- [ ] Visualization tools for debugging perception pipelines
- [ ] Performance benchmarking and logging
