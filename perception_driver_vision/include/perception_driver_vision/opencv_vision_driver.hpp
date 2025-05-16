#ifndef OPENCV_VISION_DRIVER_HPP_
#define OPENCV_VISION_DRIVER_HPP_

#include <mutex>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <perception/driver_base.hpp>

namespace perception
{

class OpenCVVisionDriver : public DriverBase
{
public:
  OpenCVVisionDriver() = default;
  ~OpenCVVisionDriver() override
  {
    stop();
  }

  /**
   * @brief Start the driver streaming with a ROS node and namespace
   *
   * @param node Shared pointer to the ROS node
   * @param config configuration loaded from the yaml file
   */
  void start(const rclcpp::Node::SharedPtr& node, const driver_options& config) override
  {
    initialize_base(node, config);

    // open the camera device
    capture_device.open(config_.device_id);

    // check if the camera opened successfully
    if (!capture_device.isOpened())
    {
      throw perception_exception("OpenCVVisionDriver failed to open video device ID: " + config_.device_id);
    }

    event_->info("OpenCVVisionDriver started on video device " + config_.device_id);
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
      event_->info("OpenCVVisionDriver stopped.");
    }
  }

  /**
   * @brief Get latest image data from the driver. cast to sensor_msgs::msg::Image::SharedPtr
   * before using
   *
   * @return std::any The latest data from the driver of type sensor_msgs::msg::Image::SharedPtr
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

    std_msgs::msg::Header header;
    header.stamp = node_->now();
    header.frame_id = "camera_frame";

    sensor_msgs::msg::Image::SharedPtr ros_img = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    return ros_img;
  }

  std::string getName() const override
  {
    return "OpenCVVisionDriver";
  }

protected:
  mutable cv::VideoCapture capture_device;
};

}  // namespace perception

#endif  // OPENCV_VISION_DRIVER_HPP_
