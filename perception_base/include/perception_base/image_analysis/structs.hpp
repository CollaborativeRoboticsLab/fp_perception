#pragma once

#include <string>

#include <perception_base/vision/structs.hpp>

namespace perception
{

struct image_analysis_request
{
  vision_frame frame;
  std::string prompt;
  bool use_device_vision = false;
};

struct image_analysis_result
{
  std::string response;
  bool success = false;
  std::string error;
};

}  // namespace perception