#pragma once

#include <functional>
#include <memory>

#include <fp_perception_base/audio/structs.hpp>
#include <fp_perception_base/exceptions.hpp>
#include <fp_perception_base/transcription/structs.hpp>
#include <fp_perception_base/transcription/transcription_driver.hpp>

namespace fp_perception
{

class TranscriptionPipeline
{
public:
  using AudioReader = std::function<audio_data(const audio_buffer_request&)>;

  TranscriptionPipeline(std::shared_ptr<TranscriptionDriver> driver, AudioReader readDeviceAudio)
    : driver_(std::move(driver)), read_device_audio_(std::move(readDeviceAudio))
  {
  }

  transcription_result run(transcription_request request)
  {
    if (!driver_)
      throw fp_perception_exception("Transcription driver is not loaded.");

    if (request.use_device_audio)
    {
      if (!read_device_audio_)
        throw fp_perception_exception("Device audio reader is not available.");

      audio_buffer_request audio_request;
      audio_request.duration_seconds = request.audio_request_window;
      audio_request.use_time_window = request.use_device_audio_time_window;
      audio_request.start_time = request.device_audio_start_time;
      request.audio = read_device_audio_(audio_request);
    }

    return driver_->transcribe(request);
  }

private:
  std::shared_ptr<TranscriptionDriver> driver_;
  AudioReader read_device_audio_;
};

}  // namespace fp_perception