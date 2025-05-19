#pragma once

#include <mutex>
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
    node_->declare_parameter("driver.vision.DefaultDriver.name", "DefaultDriver");
    node_->declare_parameter("driver.vision.DefaultDriver.topic", "/camera/raw");

    // Load parameters from the node
    config_.name = node_->get_parameter("driver.vision.DefaultDriver.name").as_string();
    config_.topic = node_->get_parameter("driver.vision.DefaultDriver.topic").as_string();

    // Publish about the assigned driver parameters
    event_->info("Assigned driver name: " + config_.name);
    event_->info("Assigned driver topic: " + config_.topic);

    initialize_base(node);

    event_->info("Initialized");
  }

  /**
   * @brief Start the driver streaming with a ROS node and namespace
   *
   */
  void start() override
  {
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
   * @brief Get latest image data from the driver. cast to sensor_msgs::msg::Image::SharedPtr
   * befure using

   * @return std::any The latest data from the driver of type sensor_msgs::msg::Image::SharedPtr
   * @throws perception_exception if not implemented in derived classes
   */
  std::any getData() const override
  {
    std::unique_lock<std::mutex> lock(image_mutex_);

    image_ready_cv_.wait(lock, [this] { return latest_image_ != nullptr; });

    return std::make_shared<sensor_msgs::msg::Image>(*latest_image_);
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
