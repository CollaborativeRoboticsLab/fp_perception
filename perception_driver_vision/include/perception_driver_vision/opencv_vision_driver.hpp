#pragma once

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <perception_base/driver_base.hpp>

namespace perception
{
/**
 * @brief OpenCVDriver class for handling video input from a camera using OpenCV.
 *
 * This class is responsible for managing the video input from a camera using OpenCV.
 * It provides methods to start and stop the video stream, as well as retrieve image data.
 */

class OpenCVDriver : public DriverBase
{
public:
  OpenCVDriver()
  {
  }
  ~OpenCVDriver() override
  {
    deinitialize();
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
    node->declare_parameter("driver.vision.OpenCVDriver.name", "OpenCVDriver");
    node->declare_parameter("driver.vision.OpenCVDriver.device_id", 0);

    // Load parameters from the node
    name_ = node->get_parameter("driver.vision.OpenCVDriver.name").as_string();
    device_id = node->get_parameter("driver.vision.OpenCVDriver.device_id").as_int();

    // Initialize the base driver
    initialize_base(node);

    // Publish about the assigned driver parameters 
    RCLCPP_INFO(node_->get_logger(), "Assigned driver name: %s", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver device_id: %d", device_id);


    RCLCPP_INFO(node_->get_logger(), "OpenCVDriver starting on video device %d", device_id);

    // open the camera device
    capture_device.open(device_id);

    // check if the camera opened successfully
    if (!capture_device.isOpened())
    {
      RCLCPP_ERROR(node_->get_logger(), "OpenCVDriver failed to open video device ID: %d", device_id);
      throw perception_exception("OpenCVDriver failed to open video device ID: " + std::to_string(device_id));
    }

    RCLCPP_INFO(node_->get_logger(), "OpenCVDriver started on video device %d", device_id);
  }

  /**
   * @brief Stop driver streaming
   *
   */
  void deinitialize() override
  {
    if (capture_device.isOpened())
    {
      capture_device.release();
      RCLCPP_INFO(node_->get_logger(), "OpenCVDriver stopped.");
    }

    if (driver_thread_.joinable())
    {
      driver_thread_.join();
      RCLCPP_INFO(node_->get_logger(), "OpenCVDriver thread stopped.");
    }
  }

  /**
   * @brief Get latest image data from the driver. cast to cv::Mat before using
   *
   * @return std::any The latest data from the driver of type cv::Mat
   * @throws perception_exception if not implemented in derived classes
   */
  std::any getData() override
  {
    if (!capture_device.isOpened())
      throw perception_exception("Camera device is not opened");

    cv::Mat frame;

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    capture_device >> frame;

    if (frame.empty())
      throw perception_exception("Captured empty frame from camera");

    return frame;
  }

  /**
   * @brief Test function to check the driver functionality by writing the image to a file
   * This function creates a "test" directory if it doesn't exist and saves the captured image
   */
  void test() override
  {
    RCLCPP_INFO(node_->get_logger(), "OpenCVDriver test function called");

    // Create the "test" directory if it doesn't exist
    DriverBase::check_directory("test");

    // Save image to the "test" folder
    cv::Mat frame = std::any_cast<cv::Mat>(getData());
    cv::imwrite("test/opencv_vision_image.jpg", frame);

    RCLCPP_INFO(node_->get_logger(), "OpenCVDriver test image saved to 'test/opencv_vision_image.jpg'. Test "
                                     "completed.");
  }

protected:
  int device_id;
  cv::VideoCapture capture_device;
};

}  // namespace perception
