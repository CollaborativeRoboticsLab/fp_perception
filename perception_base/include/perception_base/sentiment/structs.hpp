#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace perception
{

struct sentiment_request
{
  std::string text;
  bool use_device_audio = false;
  int device_buffer_time = 0;
  bool use_device_audio_time_window = false;
  rclcpp::Time device_audio_start_time;
};

struct sentiment_result
{
  std::string label;
  double score = 0.0;
  bool success = false;
  std::string error;
};

}  // namespace perception