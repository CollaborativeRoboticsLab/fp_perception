#pragma once

#include <perception_base/driver_base.hpp>
#include <perception_base/image_analysis/structs.hpp>

namespace perception
{

class ImageAnalysisDriver : public virtual DriverBase
{
public:
  virtual image_analysis_result analyze(const image_analysis_request& request) = 0;
};

}  // namespace perception