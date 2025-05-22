# Perception Driver Vision

The `vision_driver` package provides a plugin-based framework for integrating vision input sources into the perception system. It defines a standard interface for image acquisition drivers and includes reference implementations such as:

- `DefaultVisionDriver` – subscribes to an image topic via `image_transport`
- `OpenCVVisionDriver` – captures images directly from a local camera device using OpenCV

This package abstracts vision input devices so they can be easily plugged into a perception pipeline for human-robot interaction (HRI) scenarios. Drivers are dynamically loaded using ROS 2 `pluginlib`, allowing flexible sensor integration without recompilation.

## Features

- Plugin-based driver loading
- ROS topic-based and direct camera support
- Thread-safe access to latest image frame
- Unified data interface** (`getData()`) returning `sensor_msgs::msg::Image::SharedPtr`


## Available Drivers

### 1. `DefaultDriver`

- Subscribes to a ROS image topic using `image_transport`
- Stores the latest image for retrieval

### 2. `OpenCVDriver`

* Captures images directly from a camera device using `cv::VideoCapture`
* Useful for quick prototyping without ROS camera drivers

## Parameters

```yaml
driver:
  vision:
    DefaultDriver:
      name: DefaultDriver
      topic: /camera/image_raw
    OpenCVDriver:
      name: OpenCVDriver
      device_id: 0
```

## Using as a Plugin

Both drivers implement the `DriverBase` interface from the `perception` core package. To load one via pluginlib:

```cpp
pluginlib::ClassLoader<perception::DriverBase> loader("perception_driver_vision", "perception::DriverBase");
auto driver = loader.createSharedInstance("perception::OpenCVVisionDriver");
driver->start(node, config);
```

## Example Usage

Call `getData()` to retrieve the latest image from any loaded driver:

```cpp
cv::Mat img = std::any_cast<cv::Mat>(driver->getData());
// Use image for processing or visualization
```


## To-Do

-[ ] Camera info publisher integration
-[ ] Dynamic reconfiguration of camera parameters
-[ ] Stereo camera and RGB-D support


