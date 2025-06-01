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
    node->declare_parameter("driver.audio.SpeakerAudioDriver.name", "SpeakerAudioDriver");
    node->declare_parameter("driver.audio.SpeakerAudioDriver.device_id", 0);
    node->declare_parameter("driver.audio.SpeakerAudioDriver.subscribe", false);
    node->declare_parameter("driver.audio.SpeakerAudioDriver.topic", "audio/speaker");
    node->declare_parameter("driver.audio.SpeakerAudioDriver.frame_id", "speaker_frame");
    node->declare_parameter("driver.audio.SpeakerAudioDriver.chunk_size", 256);     // default chunk size
    node->declare_parameter("driver.audio.SpeakerAudioDriver.sample_rate", 44100);  // default sample rate
    node->declare_parameter("driver.audio.SpeakerAudioDriver.channels", 1);         // default number of channels

    // Load parameters from the node
    config_.name = node->get_parameter("driver.audio.SpeakerAudioDriver.name").as_string();
    config_.device_id = node->get_parameter("driver.audio.SpeakerAudioDriver.device_id").as_int();
    config_.subscribe = node->get_parameter("driver.audio.SpeakerAudioDriver.subscribe").as_bool();
    config_.topic = node->get_parameter("driver.audio.SpeakerAudioDriver.topic").as_string();
    config_.frame_id = node->get_parameter("driver.audio.SpeakerAudioDriver.frame_id").as_string();
    chunk_size_ = node->get_parameter("driver.audio.SpeakerAudioDriver.chunk_size").as_int();
    sample_rate_ = node->get_parameter("driver.audio.SpeakerAudioDriver.sample_rate").as_int();
    channels_ = node->get_parameter("driver.audio.SpeakerAudioDriver.channels").as_int();

    // Initialize the base driver
    initialize_base(node);

    // Publish about the assigned driver parameters
    event_->info("Assigned driver name: " + config_.name);
    event_->info("Assigned driver device_id: " + std::to_string(config_.device_id));
    event_->info("Assigned driver subscribe: " + std::string(config_.subscribe ? "true" : "false"));
    event_->info("Assigned driver topic: " + config_.topic);
    event_->info("Assigned driver frame_id: " + config_.frame_id);
    event_->info("Assigned driver chunk_size: " + std::to_string(chunk_size_));
    event_->info("Assigned driver sample_rate: " + std::to_string(sample_rate_));
    event_->info("Assigned driver channels: " + std::to_string(channels_));

    event_->info("Initialized");

    // If subscribing to audio data, set up the subscriber
    if (config_.subscribe)
    {
      audio_subscriber_ = node->create_subscription<std_msgs::msg::Int16MultiArray>(
          config_.topic, 10, std::bind(&SpeakerAudioDriver::receiveData, this, std::placeholders::_1));
      event_->info("Audio subscriber created for topic: " + config_.topic);
    }
  }

  /**
   * @brief Start the driver streaming with a ROS node. This function initializes the speaker driver
   * and starts the audio stream. It uses PortAudio to open the default audio stream and starts it.
   *
   */
  void start() override
  {
    event_->info("SpeakerAudioDriver starting on device " + std::to_string(config_.device_id));
    Pa_Initialize();

    PaStreamParameters outputParams;
    outputParams.device = config_.device_id;
    outputParams.channelCount = channels_;
    outputParams.sampleFormat = paInt16;
    outputParams.suggestedLatency = Pa_GetDeviceInfo(config_.device_id)->defaultLowOutputLatency;
    outputParams.hostApiSpecificStreamInfo = nullptr;

    PaError err = Pa_OpenStream(&stream_, nullptr, &outputParams, sample_rate_, chunk_size_, paNoFlag,
                                &SpeakerAudioDriver::paCallback, this);

    if (err != paNoError)
    {
      event_->error("Failed to open speaker stream: " + std::string(Pa_GetErrorText(err)));
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
  void setData(const std::any& input) const override
  {
    try
    {
      const auto& new_samples = std::any_cast<const std::vector<int16_t>&>(input);

      std::lock_guard<std::mutex> lock(driver_mutex_);
      audio_queue_.insert(audio_queue_.end(), new_samples.begin(), new_samples.end());
    }
    catch (const std::bad_any_cast&)
    {
      throw perception_exception("Invalid audio data passed to SpeakerAudioDriver::setData");
    }
  }

  /**
   * @brief Set the latest audio data to the driver.
   * This function sends the latest audio data to the speaker driver.
   *
   * @param input The latest audio data to the driver in the form of `const std::vector<std::vector<int16_t>>&`.
   * @throws perception_exception if not implemented in derived classes
   */
  void setDataStream(const std::any& input) const override
  {
    try
    {
      const auto& new_samples = std::any_cast<const std::vector<std::vector<int16_t>>&>(input);

      for (const auto& sample : new_samples)
        setData(sample);
    }
    catch (const perception_exception& error)
    {
      throw error;
    }
  }

  /**
   * @brief Set the latest audio data to the driver from topic subscription.
   * This function is called when new audio data is received from the subscribed topic.
   *
   * @return std::any The latest audio data from the driver.
   * @throws perception_exception if the stream is not active
   */
  void receiveData(const std_msgs::msg::Int16MultiArray& msg) const
  {
    int chunk_size = msg.layout.dim[1].size;
    int chunk_count = msg.layout.dim[0].size;

    for (int i = 0; i < chunk_count; ++i)
    {
      std::vector<int16_t> chunk(msg.data.begin() + i * chunk_size, msg.data.begin() + (i + 1) * chunk_size);
      setData(chunk);
    }

    event_->info("Received audio data from topic: " + config_.topic);
  }

protected:
  static int paCallback(const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData)
  {
    auto* self = static_cast<SpeakerAudioDriver*>(userData);
    auto* out = static_cast<int16_t*>(outputBuffer);

    std::lock_guard<std::mutex> lock(self->driver_mutex_);

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
  unsigned long chunk_size_;  // Default chunk size
  int sample_rate_;           // Default sample rate
  int channels_;              // Default number of channels

  // Subscriber for audio data
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr audio_subscriber_;
};

}  // namespace perception