#pragma once

#include <perception_base/driver_base.hpp>
#include <perception_base/transcription/structs.hpp>

namespace perception
{

class TranscriptionDriver : public virtual DriverBase
{
public:
  virtual transcription_result transcribe(const transcription_request& request) = 0;
};

}  // namespace perception