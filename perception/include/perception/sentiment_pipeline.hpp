#pragma once

#include <functional>
#include <memory>

#include <perception_base/audio/structs.hpp>
#include <perception_base/exceptions.hpp>
#include <perception_base/sentiment/sentiment_analysis_driver.hpp>
#include <perception_base/sentiment/structs.hpp>
#include <perception_base/transcription/structs.hpp>
#include <perception_base/transcription/transcription_driver.hpp>

namespace perception
{

class SentimentPipeline
{
public:
  using AudioReader = std::function<audio_data(const audio_buffer_request&)>;

  SentimentPipeline(std::shared_ptr<SentimentAnalysisDriver> sentiment_driver,
                    std::shared_ptr<TranscriptionDriver> transcription_driver,
                    AudioReader readDeviceAudio)
    : sentiment_driver_(std::move(sentiment_driver)),
      transcription_driver_(std::move(transcription_driver)),
      read_device_audio_(std::move(readDeviceAudio))
  {
  }

  sentiment_result run(sentiment_request request)
  {
    if (!sentiment_driver_)
      throw perception_exception("Sentiment driver is not loaded.");

    if (request.use_device_audio)
    {
      if (!transcription_driver_)
        throw perception_exception("Transcription driver is not loaded.");

      if (!read_device_audio_)
        throw perception_exception("Device audio reader is not available.");

      transcription_request transcription_request_data;
      audio_buffer_request audio_request;
      audio_request.duration_seconds = request.audio_request_window;
      audio_request.use_time_window = request.use_device_audio_time_window;
      audio_request.start_time = request.device_audio_start_time;
      transcription_request_data.audio = read_device_audio_(audio_request);
      transcription_request_data.use_device_audio = true;
      transcription_request_data.audio_request_window = request.audio_request_window;
      transcription_request_data.use_device_audio_time_window = request.use_device_audio_time_window;
      transcription_request_data.device_audio_start_time = request.device_audio_start_time;

      const auto transcription_result_data = transcription_driver_->transcribe(transcription_request_data);
      if (!transcription_result_data.success)
        throw perception_exception(transcription_result_data.error.empty()
                                       ? "No transcription result available for sentiment analysis."
                                       : transcription_result_data.error);

      request.text = transcription_result_data.text;
    }

    auto result = sentiment_driver_->analyze(request);
    result.analyzed_text = request.text;
    return result;
  }

private:
  std::shared_ptr<SentimentAnalysisDriver> sentiment_driver_;
  std::shared_ptr<TranscriptionDriver> transcription_driver_;
  AudioReader read_device_audio_;
};

}  // namespace perception