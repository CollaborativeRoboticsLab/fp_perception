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



## Plugin Architecture

### Base Classes

#### `DriverBase`

Used for implementing drivers that interact with perception hardware (e.g., cameras, depth sensors).

```cpp
void initialize(const rclcpp::Node::SharedPtr& node, const driver_options& config);
void start();
void stop();
```

#### `AlgorithmBase`

Used for implementing perception algorithms (e.g., face detection, object tracking).

```cpp
void initialize(const rclcpp::Node::SharedPtr& node, const algorithm_options& config);
void start();
void stop();
```

Each base class includes a helper `initialize_base()` method to handle common setup like ROS node storage and event client creation.


## Driver Plugins

- [Perception Vision Driver](https://github.com/CollaborativeRoboticsLab/perception/blob/main/perception_driver_vision/readme.md) plugins required to interact with visual data
- [Perception Audio Driver](https://github.com/CollaborativeRoboticsLab/perception/blob/main/perception_driver_audio/readme.md) plugins required to interact with visual data

## Algorithm Plugins

- [Perception Eye Gaze Detection](https://github.com/CollaborativeRoboticsLab/perception/blob/main/perception_detect_eye_gaze/readme.md) plugins required to interact with visual data

## Plugin selection

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
