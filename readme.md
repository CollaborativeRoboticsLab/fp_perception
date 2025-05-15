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



## 🔌 Plugin Architecture

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


## Example Use Case

1. Load a camera driver plugin (e.g., RealSense, OpenCV-based) via a plugin loader.
2. Load a face detection algorithm plugin.
3. Stream image data from the driver to the algorithm.
4. Emit events (e.g., `face_detected`) through the `EventClient`.

This modular setup allows swapping or upgrading components independently.

## Writing Your Own Plugins

1. Inherit from `perception::DriverBase` or `perception::AlgorithmBase`.
2. Implement required methods (`initialize`, `start`, `stop`, `getName`).
3. Register your plugin using `PLUGINLIB_EXPORT_CLASS`.
4. Define a plugin XML manifest.


## Dependencies

* ROS 2 (Humble or newer)
* `pluginlib`
* `rclcpp`
* Custom packages:

  * `perception_events`

## Future Extensions

* Dynamic runtime reconfiguration
* Built-in diagnostics
* Visualization tools for debugging perception pipelines
* Performance benchmarking and logging
