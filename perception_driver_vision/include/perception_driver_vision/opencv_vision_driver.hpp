#pragma once

#include <mutex>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <perception_base/driver_base.hpp>

namespace perception
{

class OpenCVDriver : public DriverBase
{
public:
  OpenCVDriver() = default;
  ~OpenCVDriver() override
  {
    stop();
  }

  /**
   * @brief Initialize the driver
   *
   * This function should be overridden in derived classes to provide specific initialization.
   *
   * @param node Shared pointer to the ROS node
   */
  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    // Configure parameters for the node
    node_->declare_parameter("driver.vision.OpenCVDriver.name", "OpenCVDriver");
    node_->declare_parameter("driver.vision.OpenCVDriver.device_id", "0");

    // Load parameters from the node
    config_.name = node_->get_parameter("driver.vision.OpenCVDriver.name").as_string();
    config_.device_id = node_->get_parameter("driver.vision.OpenCVDriver.device_id").as_int();

    // Publish about the assigned driver parameters
    event_->info("Assigned driver name: " + config_.name);
    event_->info("Assigned driver device_id: " + std::to_string(config_.device_id));

    initialize_base(node);

    event_->info("Initialized");
  }

  /**
   * @brief Start the driver streaming with a ROS node and namespace
   */
  void start() override
  {
    // open the camera device
    capture_device.open(config_.device_id);

    // check if the camera opened successfully
    if (!capture_device.isOpened())
    {
      throw perception_exception("OpenCVDriver failed to open video device ID: " + config_.device_id);
    }

    event_->info("OpenCVDriver started on video device " + config_.device_id);
  }

  /**
   * @brief Stop driver streaming
   *
   */
  void stop() override
  {
    if (capture_device.isOpened())
    {
      capture_device.release();
      event_->info("OpenCVDriver stopped.");
    }
  }

  /**
   * @brief Get latest image data from the driver. cast to cv::Mat before using
   *
   * @return std::any The latest data from the driver of type cv::Mat
   * @throws perception_exception if not implemented in derived classes
   */
  std::any getData() const override
  {
    if (!capture_device.isOpened())
      throw perception_exception("Camera device is not opened");

    cv::Mat frame;
    capture_device >> frame;

    if (frame.empty())
      throw perception_exception("Captured empty frame from camera");

    return frame;
  }

protected:
  mutable cv::VideoCapture capture_device;
};

}  // namespace perception
