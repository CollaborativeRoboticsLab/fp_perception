#pragma once

#include <string>

#include <fp_perception_base/vision/structs.hpp>

namespace fp_perception
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

}  // namespace fp_perception