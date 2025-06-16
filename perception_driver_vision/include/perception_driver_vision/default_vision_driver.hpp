#pragma once

#include <mutex>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <condition_variable>
#include <perception_base/driver_base.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

namespace perception
{

class DefaultDriver : public DriverBase
{
public:
  DefaultDriver() = default;
  ~DefaultDriver() override = default;

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
    config_.name = node->get_parameter("driver.vision.DefaultDriver.name").as_string();
    config_.topic = node->get_parameter("driver.vision.DefaultDriver.topic").as_string();

    // Initialize the base driver
    initialize_base(node);

    // Publish about the assigned driver parameters
    event_->info("Assigned driver name: " + config_.name);
    event_->info("Assigned driver topic: " + config_.topic);

    event_->info("Initialized");
  }

  /**
   * @brief Start the driver streaming with a ROS node and namespace
   *
   */
  void start() override
  {
    event_->info("DefaultDriver starting on topic " + config_.topic);
    image_transport::ImageTransport transport(node_);

    image_sub_ =
        transport.subscribe(config_.topic, 1, std::bind(&DefaultDriver::imageCallback, this, std::placeholders::_1));

    event_->info("Started. Subscribed to image topic: " + config_.topic);
  }

  /**
   * @brief Stop driver streaming
   *
   */
  void stop() override
  {
    image_sub_.shutdown();
    event_->info("Driver stopped.");
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

    try
    {
      // Convert sensor_msgs::Image to OpenCV Mat
      auto cv_ptr = cv_bridge::toCvCopy(latest_image_, latest_image_->encoding);
      return cv_ptr->image.clone();  // Return a deep copy of cv::Mat
    }
    catch (const cv_bridge::Exception& e)
    {
      throw perception_exception("cv_bridge conversion failed: " + std::string(e.what()));
    }
  }

  /**
   * @brief Test function to check the driver functionality by writing the image to a file
   */
  void test() override
  {
    event_->info("DefaultDriver test function called");

    // Create the "test" directory if it doesn't exist
    DriverBase::check_directory("test");

    cv::Mat frame = std::any_cast<cv::Mat>(getData());
    cv::imwrite("test/default_vision_image.jpg", frame);

    event_->info("Test image saved to test/default_vision_image.jpg. Test completed.");
  }

protected:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    latest_image_ = msg;

    buffer_cv_.notify_all();
  }

  image_transport::Subscriber image_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
};

}  // namespace perception
