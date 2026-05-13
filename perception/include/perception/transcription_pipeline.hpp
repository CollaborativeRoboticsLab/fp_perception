#pragma once

#include <functional>
#include <memory>

#include <perception_base/audio/structs.hpp>
#include <perception_base/exceptions.hpp>
#include <perception_base/transcription/structs.hpp>
#include <perception_base/transcription/transcription_driver.hpp>

namespace perception
{

class TranscriptionPipeline
{
public:
  using AudioReader = std::function<audio_data(int)>;

  TranscriptionPipeline(std::shared_ptr<TranscriptionDriver> driver, AudioReader readDeviceAudio)
    : driver_(std::move(driver)), read_device_audio_(std::move(readDeviceAudio))
  {
  }

  transcription_result run(transcription_request request)
  {
    if (!driver_)
      throw perception_exception("Transcription driver is not loaded.");

    if (request.use_device_audio)
    {
      if (!read_device_audio_)
        throw perception_exception("Device audio reader is not available.");

      request.audio = read_device_audio_(request.device_buffer_time);
    }

    return driver_->transcribe(request);
  }

private:
  std::shared_ptr<TranscriptionDriver> driver_;
  AudioReader read_device_audio_;
};

}  // namespace perception