#pragma once

#include <perception_base/driver_base.hpp>
#include <perception_base/audio/structs.hpp>

namespace perception
{

class SpeechSynthesisDriver : public virtual DriverBase
{
public:
  virtual audio_data synthesize(const text_data& input) = 0;
};

}  // namespace perception