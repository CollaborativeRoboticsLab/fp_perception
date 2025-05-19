#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <perception_base/driver_base.hpp>

namespace perception
{

/**
 * @brief SpeakerAudioDriver class for handling audio output to a speaker.
 *
 * This class is responsible for managing the audio output to a speaker using PortAudio.
 * It provides methods to start and stop the audio stream, as well as play audio data.
 */
class SpeakerAudioDriver : public DriverBase
{
public:
  SpeakerAudioDriver() : stream_(nullptr)
  {
  }
  ~SpeakerAudioDriver() override
  {
    stop();
    Pa_Terminate();
  }

  /**
   * @brief Initialize the driver
   *
   * This function should be overridden in derived classes to provide specific initialization.
   *
   * @param node Shared pointer to the ROS node
   */
  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    // Configure parameters for the nodevision
    node_->declare_parameter("driver.audio.SpeakerAudioDriver.name", "SpeakerAudioDriver");
    node_->declare_parameter("driver.audio.SpeakerAudioDriver.device_id", "0");

    // Load parameters from the node
    config_.name = node_->get_parameter("driver.audio.SpeakerAudioDriver.name").as_string();
    config_.device_id = node_->get_parameter("driver.audio.SpeakerAudioDriver.device_id").as_int();

    // Publish about the assigned driver parameters
    event_->info("Assigned driver name: " + config_.name);
    event_->info("Assigned driver device_id: " + std::to_string(config_.device_id));

    initialize_base(node);

    event_->info("Initialized");
  }

  /**
   * @brief Start the driver streaming with a ROS node. This function initializes the speaker driver
   * and starts the audio stream. It uses PortAudio to open the default audio stream and starts it.
   *
   */
  void start() override
  {
    Pa_Initialize();

    PaStreamParameters outputParams;
    outputParams.device = config_.device_id;
    outputParams.channelCount = 1;
    outputParams.sampleFormat = paInt16;
    outputParams.suggestedLatency = Pa_GetDeviceInfo(config_.device_id)->defaultLowOutputLatency;
    outputParams.hostApiSpecificStreamInfo = nullptr;

    PaError err =
        Pa_OpenStream(&stream_, nullptr, &outputParams, 44100, 256, paNoFlag, &SpeakerAudioDriver::paCallback, this);

    if (err != paNoError)
    {
      throw perception_exception("Failed to open speaker stream: " + std::string(Pa_GetErrorText(err)));
    }

    Pa_StartStream(stream_);

    event_->info("SpeakerAudioDriver started (non-blocking).");
  }

  /**
   * @brief Stop driver streaming. This function stops the audio stream and closes it.
   * It also terminates the PortAudio library.
   */
  void stop() override
  {
    if (stream_)
    {
      Pa_StopStream(stream_);
      Pa_CloseStream(stream_);
      stream_ = nullptr;
      event_->info("SpeakerAudioDriver stopped.");
    }
  }

  /**
   * @brief Set the latest data to the driver.
   * This function sends the latest audio data to the speaker driver.
   *
   * @param input The latest audio data to the driver in the form of `const std::vector<int16_t>&`.
   * @throws perception_exception if not implemented in derived classes
   */
  void setData(std::any& input) const override
  {
    try
    {
      const auto& new_samples = std::any_cast<const std::vector<int16_t>&>(input);

      std::lock_guard<std::mutex> lock(buffer_mutex_);
      audio_queue_.insert(audio_queue_.end(), new_samples.begin(), new_samples.end());
    }
    catch (const std::bad_any_cast&)
    {
      throw perception_exception("Invalid audio data passed to SpeakerAudioDriver::setData");
    }
  }

protected:
  static int paCallback(const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData)
  {
    auto* self = static_cast<SpeakerAudioDriver*>(userData);
    auto* out = static_cast<int16_t*>(outputBuffer);

    std::lock_guard<std::mutex> lock(self->buffer_mutex_);

    for (unsigned long i = 0; i < framesPerBuffer; ++i)
    {
      if (!self->audio_queue_.empty())
      {
        out[i] = self->audio_queue_.front();
        self->audio_queue_.pop_front();
      }
      else
      {
        out[i] = 0;  // output silence if buffer is empty
      }
    }

    return paContinue;
  }

  mutable PaStream* stream_;
  mutable std::deque<int16_t> audio_queue_;
  mutable std::mutex buffer_mutex_;
};

}  // namespace perception