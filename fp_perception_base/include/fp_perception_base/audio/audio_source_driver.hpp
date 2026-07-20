#pragma once

#include <fp_perception_base/driver_base.hpp>
#include <fp_perception_base/audio/structs.hpp>

namespace fp_perception
{

class AudioSourceDriver : public virtual DriverBase
{
public:
  virtual audio_data readChunk() = 0;

  virtual audio_data readBufferedAudio(int duration_seconds)
  {
    (void)duration_seconds;
    throw fp_perception_exception("readBufferedAudio() not implemented for this audio source.");
  }
};

}  // namespace fp_perception