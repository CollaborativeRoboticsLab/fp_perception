#pragma once

#include <perception_base/driver_base.hpp>
#include <perception_base/audio/structs.hpp>

namespace perception
{

class AudioSinkDriver : public virtual DriverBase
{
public:
  virtual void play(const audio_data& data) = 0;
};

}  // namespace perception