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
  std::any getData() const override
  {
    std::unique_lock<std::mutex> lock(image_mutex_);

    image_ready_cv_.wait(lock, [this] { return latest_image_ != nullptr; });

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

protected:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    event_->debug("Received image: width=" + std::to_string(msg->width) + ", height=" + std::to_string(msg->height));

    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_ = msg;

    image_ready_cv_.notify_all();

    event_->debug("Image data updated.");
  }

  image_transport::Subscriber image_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;

  mutable std::mutex image_mutex_;
  mutable std::condition_variable image_ready_cv_;
};

}  // namespace perception
