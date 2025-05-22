#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <perception_base/driver_base.hpp>

namespace perception
{

/**
 * @brief MicrophoneAudioDriver class for handling audio input from a microphone.
 *
 * This class is responsible for managing the audio input from a microphone using PortAudio.
 * It provides methods to start and stop the audio stream, as well as retrieve audio data.
 */
class MicrophoneAudioDriver : public DriverBase
{
public:
  MicrophoneAudioDriver() : stream_(nullptr)
  {
  }
  ~MicrophoneAudioDriver() override
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
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.name", "MicrophoneAudioDriver");
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.device_id", 0);  // default device

    // Load parameters from the node
    config_.name = node->get_parameter("driver.audio.MicrophoneAudioDriver.name").as_string();
    config_.device_id = node->get_parameter("driver.audio.MicrophoneAudioDriver.device_id").as_int();

    // Initialize the base driver
    initialize_base(node);

    // Publish about the assigned driver parameters
    event_->info("Assigned driver name: " + config_.name);
    event_->info("Assigned driver device_id: " + std::to_string(config_.device_id));

    event_->info("Initialized");
  }

  /**
   * @brief Start the driver streaming. This function initializes the microphone driver and
   * starts the audio stream. It uses PortAudio to open the default audio stream and starts it.
   *
   */
  void start() override
  {
    event_->info("MicrophoneAudioDriver starting on device " + std::to_string(config_.device_id));

    Pa_Initialize();

    PaStreamParameters inputParams;
    inputParams.device = config_.device_id;
    inputParams.channelCount = 1;
    inputParams.sampleFormat = paInt16;
    inputParams.suggestedLatency = Pa_GetDeviceInfo(config_.device_id)->defaultLowInputLatency;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    PaError err =
        Pa_OpenStream(&stream_, &inputParams, nullptr, 44100, 256, paNoFlag, &MicrophoneAudioDriver::paCallback, this);

    if (err != paNoError)
    {
      event_->error("Failed to open microphone stream: " + std::string(Pa_GetErrorText(err)));
      throw perception_exception("Failed to open microphone stream: " + std::string(Pa_GetErrorText(err)));
    }

    Pa_StartStream(stream_);

    event_->info("MicrophoneAudioDriver started (non-blocking).");
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
      event_->info("MicrophoneAudioDriver stopped.");
    }
  }

  /**
   * @brief Get latest audio data from the driver. This function reads audio data from
   * the microphone stream and returns it as a vector of int16_t.
   *
   * @return std::any The latest audio data from the driver.
   * @throws perception_exception if the stream is not active
   */
  std::any getData() const override
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (audio_buffer_.empty())
      throw perception_exception("No audio data available");

    std::vector<int16_t> chunk;
    const size_t chunk_size = 256;

    for (size_t i = 0; i < chunk_size && !audio_buffer_.empty(); ++i)
    {
      chunk.push_back(audio_buffer_.front());
      audio_buffer_.pop_front();
    }

    return chunk;
  }

protected:
  static int paCallback(const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData)
  {
    auto* self = static_cast<MicrophoneAudioDriver*>(userData);
    const auto* in = static_cast<const int16_t*>(inputBuffer);

    if (in)
    {
      std::lock_guard<std::mutex> lock(self->buffer_mutex_);
      self->audio_buffer_.insert(self->audio_buffer_.end(), in, in + framesPerBuffer);
    }

    return paContinue;
  }

  PaStream* stream_;
  mutable std::deque<int16_t> audio_buffer_;
  mutable std::mutex buffer_mutex_;
};

}  // namespace perception
