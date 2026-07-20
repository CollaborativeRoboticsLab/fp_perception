#pragma once

#include <atomic>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <fp_perception_base/vision/vision_source_driver.hpp>
#include <fp_perception_base/vision/structs.hpp>

namespace fp_perception
{
/**
 * @brief OpenCVDriver class for handling video input from a camera using OpenCV.
 *
 * This class is responsible for managing the video input from a camera using OpenCV.
 * It provides methods to start and stop the video stream, as well as retrieve image data.
 */

class OpenCVDriver : public VisionSourceDriver
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
      throw fp_perception_exception("OpenCVDriver failed to open video device ID: " + std::to_string(device_id));
    }

    if (diagnostics_enabled())
    {
      enable_diagnostics("opencv-camera-" + std::to_string(device_id), name_ + " status",
                         [this](diagnostic_updater::DiagnosticStatusWrapper& status) { produce_diagnostics(status); });
    }

    RCLCPP_INFO(node_->get_logger(), "OpenCVDriver started on video device %d", device_id);
  }

  /**
   * @brief Stop driver streaming
   *
   */
  void deinitialize() override
  {
    disable_diagnostics();
    if (capture_device.isOpened())
    {
      capture_device.release();
      RCLCPP_INFO(node_->get_logger(), "OpenCVDriver stopped.");
    }
  }

  /**
   * @brief Get the latest image data from the driver.
   *
   * @return vision_frame The latest frame captured from the camera.
   * @throws fp_perception_exception if not implemented in derived classes
   */
  vision_frame captureFrame() override
  {
    if (!capture_device.isOpened())
      throw fp_perception_exception("Camera device is not opened");

    cv::Mat frame;

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    capture_device >> frame;

    if (frame.empty())
    {
      capture_failure_count_++;
      throw fp_perception_exception("Captured empty frame from camera");
    }

    successful_capture_count_++;

    vision_frame vision;
    vision.image = frame;
    vision.frame_id = name_;
    vision.stamp = node_->now();

    return vision;
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
    cv::imwrite("test/opencv_vision_image.jpg", captureFrame().image);

    RCLCPP_INFO(node_->get_logger(), "OpenCVDriver test image saved to 'test/opencv_vision_image.jpg'. Test "
                                     "completed.");
  }

protected:
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (!capture_device.isOpened())
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Camera device is not open");
    else if (capture_failure_count_.load() > 0)
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Camera capture has recent failures");
    else
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Camera capture healthy");

    status.add("device_id", device_id);
    status.add("device_open", capture_device.isOpened() ? "true" : "false");
    status.add("successful_capture_count", successful_capture_count_.load());
    status.add("capture_failure_count", capture_failure_count_.load());
  }

  int device_id;
  cv::VideoCapture capture_device;
  std::atomic<uint64_t> successful_capture_count_{ 0 };
  std::atomic<uint64_t> capture_failure_count_{ 0 };
};

}  // namespace fp_perception
