#ifndef DEFAULT_VISION_DRIVER_HPP_
#define DEFAULT_VISION_DRIVER_HPP_

#include <mutex>
#include <perception/driver_base.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

namespace perception
{

class DefaultVisionDriver : public DriverBase
{
public:
  DefaultVisionDriver() = default;
  ~DefaultVisionDriver() override = default;

  /**
   * @brief Start the driver streaming with a ROS node and namespace
   *
   * @param node Shared pointer to the ROS node
   * @param config configuration loaded from the yaml file
   */
  void start(const rclcpp::Node::SharedPtr& node, const driver_options& config) override
  {
    initialize_base(node, config);

    image_transport::ImageTransport transport(node_);

    image_sub_ = transport.subscribe(config_.topic, 1,
                                     std::bind(&DefaultVisionDriver::imageCallback, this, std::placeholders::_1));

    event_->info("Initialized. Subscribed to image topic: " + config_.topic);
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
    return std::make_shared<sensor_msgs::msg::Image>(*latest_image_);
  }

protected:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    event_->debug("Received image: width=" + std::to_string(msg->width) + ", height=" + std::to_string(msg->height));

    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_ = msg;

    event_->debug("Image data updated.");
  }

  image_transport::Subscriber image_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::mutex image_mutex_;
};

}  // namespace perception

#endif  // DEFAULT_VISION_DRIVER_HPP_
