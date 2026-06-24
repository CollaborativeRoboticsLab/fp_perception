#pragma once

#include <perception_base/driver_base.hpp>
#include <perception_base/vision/structs.hpp>

namespace perception
{

class VisionSourceDriver : public virtual DriverBase
{
public:
  virtual vision_frame captureFrame() = 0;
};

}  // namespace perception