#pragma once

#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <perception_base/driver_base.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

namespace perception
{

class DefaultDriver : public DriverBase
{
public:
  DefaultDriver() = default;

  ~DefaultDriver()
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
    node->declare_parameter("driver.vision.DefaultDriver.name", "DefaultDriver");
    node->declare_parameter("driver.vision.DefaultDriver.topic", "/camera/raw");

    // Load parameters from the node
    name_ = node->get_parameter("driver.vision.DefaultDriver.name").as_string();
    interface_name_ = node->get_parameter("driver.vision.DefaultDriver.topic").as_string();

    // Initialize the base driver
    initialize_base(node);

    // Publish about the assigned driver parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver name: %s", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver topic: %s", interface_name_.c_str());

    RCLCPP_INFO(node_->get_logger(), "DefaultDriver subscribing on topic %s", interface_name_.c_str());

    image_transport::ImageTransport transport(node_);

    image_sub_ =
        transport.subscribe(interface_name_, 1, std::bind(&DefaultDriver::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "Started. Subscribed to image topic: %s", interface_name_.c_str());
  }

  /**
   * @brief Stop driver streaming
   *
   */
  void deinitialize() override
  {
    image_sub_.shutdown();
    RCLCPP_INFO(node_->get_logger(), "DefaultDriver unsubscribed from topic: %s", interface_name_.c_str());
  }

  /**
   * @brief Get latest image data from the driver. cast to cv::Mat befure using

   * @return std::any The latest data from the driver of type cv::Mat
   * @throws perception_exception if not implemented in derived classes
   */
  std::any getData() override
  {
    // Wait for the latest image to be available
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    buffer_cv_.wait(lock, [this] { return latest_image_ != nullptr; });

    return latest_image_;
  }

  /**
   * @brief Test function to check the driver functionality by writing the image to a file
   */
  void test() override
  {
    RCLCPP_INFO(node_->get_logger(), "DefaultDriver test function called");

    // Create the "test" directory if it doesn't exist
    DriverBase::check_directory("test");

    try
    {
      // Convert sensor_msgs::Image to OpenCV Mat
      auto image = std::any_cast<sensor_msgs::msg::Image::ConstSharedPtr>(getData());
      auto cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
      cv::Mat frame = cv_ptr->image.clone();
      cv::imwrite("test/default_vision_image.jpg", frame);
    }
    catch (const cv_bridge::Exception& e)
    {
      throw perception_exception("cv_bridge conversion failed: " + std::string(e.what()));
    }

    RCLCPP_INFO(node_->get_logger(), "Test image saved to test/default_vision_image.jpg. Test completed.");
  }

protected:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    latest_image_ = msg;

    buffer_cv_.notify_all();
  }

  std::string interface_name_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
};

}  // namespace perception
