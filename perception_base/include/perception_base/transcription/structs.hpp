#pragma once

#include <string>

#include <perception_base/audio/structs.hpp>

namespace perception
{

struct transcription_request
{
  audio_data audio;
  bool use_device_audio = false;
  int device_buffer_time = 0;
};

struct transcription_result
{
  std::string text;
  bool success = false;
  std::string error;
};

}  // namespace perception