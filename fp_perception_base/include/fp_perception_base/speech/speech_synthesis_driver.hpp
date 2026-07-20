#pragma once

#include <fp_perception_base/driver_base.hpp>
#include <fp_perception_base/audio/structs.hpp>

namespace fp_perception
{

class SpeechSynthesisDriver : public virtual DriverBase
{
public:
  virtual audio_data synthesize(const text_data& input) = 0;
};

}  // namespace fp_perception