#pragma once

#include <perception_base/driver_base.hpp>
#include <perception_base/audio/structs.hpp>

namespace perception
{

class AudioSinkDriver : public virtual DriverBase
{
public:
  virtual void play(const audio_data& data) = 0;

  virtual void enqueuePlayback(const audio_data& data)
  {
    play(data);
  }

  virtual void stopPlayback()
  {
  }
};

}  // namespace perception