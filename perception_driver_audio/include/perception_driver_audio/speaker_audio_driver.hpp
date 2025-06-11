#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <fstream>
#include <perception_base/driver_base.hpp>
#include <perception_base/utils/audio.hpp>
#include <perception_msgs/msg/perception_audio.hpp>

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
      audio_subscriber_ = node->create_subscription<perception_msgs::msg::PerceptionAudio>(
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
      const auto& data = std::any_cast<const perception::audio_data&>(input);

      std::lock_guard<std::mutex> lock(driver_mutex_);
      audio_queue_.insert(audio_queue_.end(), data.samples.begin(), data.samples.end());
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
      const auto& data = std::any_cast<const perception::audio_data&>(input);

      std::lock_guard<std::mutex> lock(driver_mutex_);
      audio_queue_.insert(audio_queue_.end(), data.samples.begin(), data.samples.end());
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
   * @param msg The audio data message received from the topic.
   * @throws perception_exception if the stream is not active
   */
  void receiveData(const perception_msgs::msg::PerceptionAudio& msg) const
  {
    auto data = perception::msg_to_audio_data(msg);
    setDataStream(data);
    event_->info("Received audio data from topic: " + config_.topic);
  }

  /**
   * @brief Read test/mic_test.wav and play it through the speaker.
   */
  void test() override
  {
    event_->info("Testing by playing test/mic_test.wav...");

    const std::filesystem::path filepath("test/mic_test.wav");

    if (!std::filesystem::exists(filepath))
    {
      throw perception_exception("Audio file not found: " + filepath.string());
      event_->error("Audio file not found: " + filepath.string());
      return;
    }

    try
    {
      auto audio_data = readWavFile(filepath.string());
      setDataStream(audio_data);

      // Wait long enough for audio to play
      int duration_ms = static_cast<int>((audio_data.samples.size() * 1000.0) / (audio_data.sample_rate * audio_data.channels));

      event_->info("Playing duration estimated: " + std::to_string(duration_ms) + " ms");
      std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
    }
    catch (const perception_exception& e)
    {
      event_->error("Error during test: " + std::string(e.what()));
    }

    event_->info("Test completed.");
  }

protected:
  /**
   * @brief PortAudio callback function for audio output.
   *
   * This function is called by PortAudio to fill the output buffer with audio data.
   * It retrieves audio samples from the audio queue and writes them to the output buffer.
   *
   * @param inputBuffer Pointer to the input buffer (not used in this case).
   * @param outputBuffer Pointer to the output buffer where audio samples will be written.
   * @param framesPerBuffer Number of frames per buffer.
   * @param timeInfo Pointer to time information (not used in this case).
   * @param statusFlags Status flags (not used in this case).
   * @param userData Pointer to user data (this instance of SpeakerAudioDriver).
   * @return int Return value indicating whether to continue or stop the stream.
   */
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
  rclcpp::Subscription<perception_msgs::msg::PerceptionAudio>::SharedPtr audio_subscriber_;
};

}  // namespace perception