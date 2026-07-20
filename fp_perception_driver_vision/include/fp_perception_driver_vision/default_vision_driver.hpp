#pragma once

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <fp_perception_base/vision/vision_source_driver.hpp>
#include <fp_perception_base/exceptions.hpp>
#include <fp_perception_base/vision/structs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

namespace fp_perception
{

class DefaultDriver : public VisionSourceDriver
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

    if (diagnostics_enabled())
    {
      enable_diagnostics("vision-topic-" + name_, name_ + " status",
                         [this](diagnostic_updater::DiagnosticStatusWrapper& status) { produce_diagnostics(status); });
    }

    RCLCPP_INFO(node_->get_logger(), "Started. Subscribed to image topic: %s", interface_name_.c_str());
  }

  /**
   * @brief Stop driver streaming
   *
   */
  void deinitialize() override
  {
    disable_diagnostics();
    image_sub_.shutdown();
    RCLCPP_INFO(node_->get_logger(), "DefaultDriver unsubscribed from topic: %s", interface_name_.c_str());
  }

  /**
   * @brief Get the latest image data from the driver.
   *
   * @return vision_frame The latest frame received from the subscribed ROS topic.
   * @throws fp_perception_exception if not implemented in derived classes
   */
  vision_frame captureFrame() override
  {
    // Wait for the latest image to be available
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    buffer_cv_.wait(lock, [this] { return latest_image_ != nullptr; });

    vision_frame frame;
    frame.frame_id = latest_image_->header.frame_id;
    frame.stamp = latest_image_->header.stamp;

    try
    {
      auto cv_ptr = cv_bridge::toCvCopy(latest_image_, latest_image_->encoding);
      frame.image = cv_ptr->image.clone();
    }
    catch (const cv_bridge::Exception& e)
    {
      throw fp_perception_exception("cv_bridge conversion failed: " + std::string(e.what()));
    }

    return frame;
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
      cv::imwrite("test/default_vision_image.jpg", captureFrame().image);
    }
    catch (const cv_bridge::Exception& e)
    {
      throw fp_perception_exception("cv_bridge conversion failed: " + std::string(e.what()));
    }

    RCLCPP_INFO(node_->get_logger(), "Test image saved to test/default_vision_image.jpg. Test completed.");
  }

protected:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    latest_image_ = msg;
    received_image_count_++;

    buffer_cv_.notify_all();
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      image = latest_image_;
    }

    if (!image)
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for image frames");
    else
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Receiving image frames");

    status.add("topic", interface_name_);
    status.add("received_image_count", received_image_count_.load());
    status.add("latest_frame_available", image ? "true" : "false");
    if (image)
    {
      status.add("latest_frame_id", image->header.frame_id);
      status.add("latest_stamp_sec", image->header.stamp.sec);
      status.add("latest_stamp_nanosec", image->header.stamp.nanosec);
      status.add("latest_width", static_cast<int>(image->width));
      status.add("latest_height", static_cast<int>(image->height));
    }
  }

  std::string interface_name_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::atomic<uint64_t> received_image_count_{ 0 };
};

}  // namespace fp_perception
