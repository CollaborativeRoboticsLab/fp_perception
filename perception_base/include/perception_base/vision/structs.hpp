#pragma once

#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

namespace perception
{

struct vision_frame
{
  cv::Mat image;
  std::string frame_id;
  rclcpp::Time stamp;
};

}  // namespace perception