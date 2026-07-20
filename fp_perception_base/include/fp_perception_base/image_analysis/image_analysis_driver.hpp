#pragma once

#include <fp_perception_base/driver_base.hpp>
#include <fp_perception_base/image_analysis/structs.hpp>

namespace fp_perception
{

class ImageAnalysisDriver : public virtual DriverBase
{
public:
  virtual image_analysis_result analyze(const image_analysis_request& request) = 0;
};

}  // namespace fp_perception