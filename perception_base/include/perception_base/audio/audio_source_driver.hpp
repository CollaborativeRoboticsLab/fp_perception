#pragma once

#include <perception_base/driver_base.hpp>
#include <perception_base/audio/structs.hpp>

namespace perception
{

class AudioSourceDriver : public virtual DriverBase
{
public:
  virtual audio_data readChunk() = 0;

  virtual audio_data readBufferedAudio(int duration_seconds)
  {
    (void)duration_seconds;
    throw perception_exception("readBufferedAudio() not implemented for this audio source.");
  }
};

}  // namespace perception