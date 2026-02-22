
# Vision drivers

This page documents the vision plugins in `perception_driver_vision`.

## DefaultDriver (ROS image subscription)

Class: `perception::DefaultDriver`

### What it does

- Subscribes to a ROS image topic using `image_transport`.
- Stores the latest `sensor_msgs/msg/Image` and returns it from `getData()`.

### Parameters

- `driver.vision.DefaultDriver.name` (string)
- `driver.vision.DefaultDriver.topic` (string)

### Usage

- Used when your camera feed already exists on a ROS topic.
- The perception server can optionally republish frames when `interface.vision_input.publish=true`.

## OpenCVDriver (direct device capture)

Class: `perception::OpenCVDriver`

### What it does

- Opens a camera device via OpenCV (`cv::VideoCapture`).
- Returns a `cv::Mat` from `getData()`.

### Parameters

- `driver.vision.OpenCVDriver.name` (string)
- `driver.vision.OpenCVDriver.device_id` (int)

### Usage

- Used when you want the perception stack to acquire frames directly from a local camera device.
- The perception server publishes frames as `sensor_msgs/msg/Image` when `interface.vision_input.publish=true` and `interface.vision_input.non_ros=true`.

