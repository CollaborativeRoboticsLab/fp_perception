
# Vision drivers

This page documents the vision plugins in `perception_driver_vision`.

## DefaultDriver (ROS image subscription)

Class: `perception::DefaultDriver`

### What it does

- Subscribes to a ROS image topic using `image_transport`.
- Stores the latest `sensor_msgs/msg/Image` and returns `vision_frame` from `captureFrame()`.

### Parameters

- `driver.vision.DefaultDriver.name` (string)
- `driver.vision.DefaultDriver.topic` (string)

### Usage

- Used when your camera feed already exists on a ROS topic.
- The perception server can optionally republish frames when `interface.vision_input.publish=true`.
- If `use_diagnostics=true`, the driver publishes frame-receive status on `/diagnostics` via `diagnostic_updater`.

## OpenCVDriver (direct device capture)

Class: `perception::OpenCVDriver`

### What it does

- Opens a camera device via OpenCV (`cv::VideoCapture`).
- Returns `vision_frame` from `captureFrame()`.

### Parameters

- `driver.vision.OpenCVDriver.name` (string)
- `driver.vision.OpenCVDriver.device_id` (int)

### Usage

- Used when you want the perception stack to acquire frames directly from a local camera device.
- The perception server publishes frames as `sensor_msgs/msg/Image` when `interface.vision_input.publish=true` and `use_non_ros_vision_driver=true`.
- If `use_diagnostics=true`, the driver publishes capture status on `/diagnostics` via `diagnostic_updater`.

