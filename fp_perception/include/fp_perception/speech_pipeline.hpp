#pragma once

#include <memory>

#include <fp_perception_base/audio/audio_sink_driver.hpp>
#include <fp_perception_base/audio/structs.hpp>
#include <fp_perception_base/exceptions.hpp>
#include <fp_perception_base/speech/speech_synthesis_driver.hpp>

namespace fp_perception
{

struct speech_pipeline_result
{
  audio_data audio;
  bool success = false;
  bool used_device_audio = false;
};

class SpeechPipeline
{
public:
  SpeechPipeline(std::shared_ptr<SpeechSynthesisDriver> speech_driver,
                 std::shared_ptr<AudioSinkDriver> speaker_driver = nullptr)
    : speech_driver_(std::move(speech_driver)), speaker_driver_(std::move(speaker_driver))
  {
  }

  speech_pipeline_result run(const text_data& input, bool use_device_audio)
  {
    if (!speech_driver_)
      throw fp_perception_exception("Speech driver is not loaded.");

    speech_pipeline_result result;
    result.audio = speech_driver_->synthesize(input);
    result.success = !result.audio.samples.empty();
    result.used_device_audio = use_device_audio;

    if (!result.success)
      return result;

    if (use_device_audio)
    {
      if (!speaker_driver_)
        throw fp_perception_exception("Speaker driver is not loaded.");

      speaker_driver_->play(result.audio);
    }

    return result;
  }

private:
  std::shared_ptr<SpeechSynthesisDriver> speech_driver_;
  std::shared_ptr<AudioSinkDriver> speaker_driver_;
};

}  // namespace fp_perception