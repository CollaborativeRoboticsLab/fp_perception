#pragma once

#include <fp_perception_base/driver_base.hpp>
#include <fp_perception_base/transcription/structs.hpp>

namespace fp_perception
{

class TranscriptionDriver : public virtual DriverBase
{
public:
  virtual transcription_result transcribe(const transcription_request& request) = 0;
};

}  // namespace fp_perception