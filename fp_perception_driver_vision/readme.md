# Perception Driver Vision

The `fp_perception_driver_vision` package provides a plugin-based framework for integrating vision input sources into the perception system. It defines a standard interface for image acquisition drivers and includes reference implementations such as:

- `DefaultVisionDriver` – subscribes to an image topic via `image_transport`
- `OpenCVVisionDriver` – captures images directly from a local camera device using OpenCV

This package abstracts vision input devices so they can be easily plugged into a perception pipeline for human-robot interaction (HRI) scenarios. Drivers are dynamically loaded using ROS 2 `pluginlib`, allowing flexible sensor integration without recompilation.

## Features

- Plugin-based driver loading
- ROS topic-based and direct camera support
- Thread-safe access to latest image frame
- Typed `VisionSourceDriver` interface returning `fp_perception::vision_frame`


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

Both drivers are pluginlib plugins with `DriverBase` as the plugin base and `VisionSourceDriver` as the typed runtime interface. To load one via pluginlib:

```cpp
pluginlib::ClassLoader<fp_perception::DriverBase> loader("fp_perception_driver_vision", "fp_perception::DriverBase");
auto driver_base = loader.createSharedInstance("fp_perception::OpenCVDriver");
auto driver = std::dynamic_pointer_cast<fp_perception::VisionSourceDriver>(driver_base);
driver->initialize(node);
```

## Example Usage

Call `captureFrame()` to retrieve the latest image from any loaded driver:

```cpp
fp_perception::vision_frame frame = driver->captureFrame();
cv::Mat img = frame.image;
// Use image for processing or visualization
```


## To-Do

-[ ] Camera info publisher integration
-[ ] Dynamic reconfiguration of camera parameters
-[ ] Stereo camera and RGB-D support


