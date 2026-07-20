#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <fp_perception_base/audio/structs.hpp>

namespace fp_perception
{

struct transcription_request
{
  audio_data audio;
  bool use_device_audio = false;
  int audio_request_window = 0;
  bool use_device_audio_time_window = false;
  rclcpp::Time device_audio_start_time;
};

struct transcription_result
{
  std::string text;
  bool success = false;
  std::string error;
};

}  // namespace fp_perception