#pragma once

#include <fp_perception_base/driver_base.hpp>
#include <fp_perception_base/vision/structs.hpp>

namespace fp_perception
{

class VisionSourceDriver : public virtual DriverBase
{
public:
  virtual vision_frame captureFrame() = 0;
};

}  // namespace fp_perception