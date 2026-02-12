#pragma once

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
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
    node->declare_parameter("driver.vision.OpenCVDriver.name", "OpenCVDriver");
    node->declare_parameter("driver.vision.OpenCVDriver.device_id", 0);
    node->declare_parameter("driver.vision.OpenCVDriver.publish", false);
    node->declare_parameter("driver.vision.OpenCVDriver.topic", "camera/image_raw");
    node->declare_parameter("driver.vision.OpenCVDriver.frame_id", "camera_frame");
    node->declare_parameter("driver.vision.OpenCVDriver.frequency", 30.0);

    // Load parameters from the node
    config_.name = node->get_parameter("driver.vision.OpenCVDriver.name").as_string();
    config_.device_id = node->get_parameter("driver.vision.OpenCVDriver.device_id").as_int();
    config_.interface_enabled = node->get_parameter("driver.vision.OpenCVDriver.publish").as_bool();
    config_.interface_name = node->get_parameter("driver.vision.OpenCVDriver.topic").as_string();
    config_.frame_id = node->get_parameter("driver.vision.OpenCVDriver.frame_id").as_string();
    config_.frequency = node->get_parameter("driver.vision.OpenCVDriver.frequency").as_double();

    // Initialize the base driver
    initialize_base(node);

    // Publish about the assigned driver parameterse
    RCLCPP_INFO(node_->get_logger(), "Assigned driver name: %s", config_.name.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver device_id: %d", config_.device_id);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver publish: %s", config_.interface_enabled ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), "Assigned driver topic: %s", config_.interface_name.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver frame_id: %s", config_.frame_id.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver frequency: %f", config_.frequency);

    // Log that the driver has been initialized
    RCLCPP_INFO(node_->get_logger(), "Initialized");
    

    // If publishing is enabled, create a publisher for the image topic
    if (config_.interface_enabled)
    {
      image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>(config_.interface_name, 10);
      RCLCPP_INFO(node_->get_logger(), "Publisher created for topic: %s", config_.interface_name.c_str());
    }
  }

  /**
   * @brief Start the driver streaming with a ROS node and namespace
   */
  void start() override
  {
    RCLCPP_INFO(node_->get_logger(), "OpenCVDriver starting on video device %d", config_.device_id);
    
    // open the camera device
    capture_device.open(config_.device_id);

    // check if the camera opened successfully
    if (!capture_device.isOpened())
    {
      RCLCPP_ERROR(node_->get_logger(), "OpenCVDriver failed to open video device ID: %d", config_.device_id);
      throw perception_exception("OpenCVDriver failed to open video device ID: " + std::to_string(config_.device_id));
    }

    RCLCPP_INFO(node_->get_logger(), "OpenCVDriver started on video device %d", config_.device_id);

    // Start the driver thread to capture and publish images

    if (config_.interface_enabled)
    {
      RCLCPP_INFO(node_->get_logger(), "Starting OpenCVDriver thread for publishing images.");
      driver_thread_ = std::thread(&OpenCVDriver::driver_thread, this);
    }
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
  std::any getData()  override
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
   * @brief collect data from the driver and publish it to the topic
   */
  void driver_thread() override
  {
    RCLCPP_INFO(node_->get_logger(), "OpenCVDriver thread started for device ID: %d", config_.device_id);

    sensor_msgs::msg::Image::SharedPtr msg;

    while (rclcpp::ok())
    {
      // Capture frame from the camera
      cv::Mat frame = std::any_cast<cv::Mat>(getData());

      // If publishing is enabled, publish the image
      msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      msg->header.stamp = node_->now();
      msg->header.frame_id = config_.frame_id;
      image_publisher_->publish(*msg);

      // Sleep for the configured frequency
      std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 / config_.frequency)));
    }
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

    RCLCPP_INFO(node_->get_logger(), "OpenCVDriver test image saved to 'test/opencv_vision_image.jpg'. Test completed.");
  }

protected:
  cv::VideoCapture capture_device;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

}  // namespace perception
