#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <fstream>
#include <perception_base/driver_base.hpp>
#include <perception_base/utils/audio.hpp>
#include <perception_msgs/msg/perception_audio.hpp>
#include <perception_driver_audio/utils.hpp>

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
  /**
   * @brief Constructor for SpeakerAudioDriver
   *
   * Initializes the PortAudio library and prepares the audio stream.
   */
  SpeakerAudioDriver() : stream_(nullptr)
  {
  }

  /**
   * @brief Destructor for SpeakerAudioDriver
   *
   * Stops the audio stream and terminates the PortAudio library.
   */
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
    node->declare_parameter("driver.audio.SpeakerAudioDriver.device_name", "default");
    node->declare_parameter("driver.audio.SpeakerAudioDriver.subscribe", false);
    node->declare_parameter("driver.audio.SpeakerAudioDriver.topic", "audio/speaker");
    node->declare_parameter("driver.audio.SpeakerAudioDriver.frame_id", "speaker_frame");
    node->declare_parameter("driver.audio.SpeakerAudioDriver.chunk_size", 256);     // default chunk size
    node->declare_parameter("driver.audio.SpeakerAudioDriver.sample_rate", 44100);  // default sample rate
    node->declare_parameter("driver.audio.SpeakerAudioDriver.channels", 1);         // default number of channels

    // Load parameters from the node
    config_.name = node->get_parameter("driver.audio.SpeakerAudioDriver.name").as_string();
    config_.device_name = node->get_parameter("driver.audio.SpeakerAudioDriver.device_name").as_string();
    config_.subscribe = node->get_parameter("driver.audio.SpeakerAudioDriver.subscribe").as_bool();
    config_.topic = node->get_parameter("driver.audio.SpeakerAudioDriver.topic").as_string();
    config_.frame_id = node->get_parameter("driver.audio.SpeakerAudioDriver.frame_id").as_string();
    chunk_size_ = node->get_parameter("driver.audio.SpeakerAudioDriver.chunk_size").as_int();
    sample_rate_ = node->get_parameter("driver.audio.SpeakerAudioDriver.sample_rate").as_int();
    channels_ = node->get_parameter("driver.audio.SpeakerAudioDriver.channels").as_int();

    // Initialize the base driver
    initialize_base(node);

    // Initialize PortAudio
    err = Pa_Initialize();
    if (err != paNoError)
    {
      event_->error("PortAudio initialization failed: " + std::string(Pa_GetErrorText(err)));
      throw perception_exception("PortAudio initialization failed: " + std::string(Pa_GetErrorText(err)));
    }

    // get the device ID by name
    try
    {
      config_.device_id = perception::getDeviceIdByName(config_.device_name);
      event_->info("Device ID for name '" + config_.device_name + "' is " + std::to_string(config_.device_id));
    }
    catch (const std::exception& e)
    {
      event_->error("Failed to get device ID for name '" + config_.device_name + "': " + e.what());
      throw perception_exception("Failed to get device ID for name '" + config_.device_name + "': " + e.what());
    }

    // Publish about the assigned driver parameters
    event_->info("Assigned driver name: " + config_.name);
    event_->info("Assigned driver device_name: " + config_.device_name);
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
    event_->info("starting on device " + std::to_string(config_.device_id));

    PaStreamParameters outputParams;
    outputParams.device = config_.device_id;
    outputParams.channelCount = channels_;
    outputParams.sampleFormat = paInt16;
    outputParams.suggestedLatency = Pa_GetDeviceInfo(config_.device_id)->defaultLowOutputLatency;
    outputParams.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&stream_, nullptr, &outputParams, sample_rate_, chunk_size_, paNoFlag, nullptr, nullptr);
    if (err != paNoError)
    {
      event_->error("Failed to open speaker stream: " + std::string(Pa_GetErrorText(err)));
      throw perception_exception("Failed to open speaker stream: " + std::string(Pa_GetErrorText(err)));
    }

    err = Pa_StartStream(stream_);
    if (err != paNoError)
    {
      event_->error("Failed to start speaker stream: " + std::string(Pa_GetErrorText(err)));
      throw perception_exception("Failed to start speaker stream: " + std::string(Pa_GetErrorText(err)));
    }

    // Start the driver thread to capture audio data
    event_->info("starting driver thread for audio capture...");
    is_running_ = true;
    driver_thread_ = std::thread(&SpeakerAudioDriver::driver_thread, this);

    event_->info("started.");
  }

  /**
   * @brief Stop driver streaming. This function stops the audio stream and closes it.
   * It also terminates the PortAudio library.
   */
  void stop() override
  {
    is_running_ = false;

    if (driver_thread_.joinable())
    {
      driver_thread_.join();
      event_->info("thread stopped.");
    }

    if (stream_)
    {
      Pa_StopStream(stream_);
      Pa_CloseStream(stream_);
      stream_ = nullptr;
      event_->info("stopped.");
    }
  }

  /**
   * @brief Set the latest audio data to the driver. This function sends the latest audio data to the speaker driver.
   *
   * @param input The latest audio data to the driver in the form of `const std::vector<std::vector<int16_t>>&`.
   * @throws perception_exception if not implemented in derived classes
   */
  void setDataStream(const std::any& input) const override
  {
    try
    {
      // convert std::any to audio_data
      const auto& data = std::any_cast<const perception::audio_data&>(input);

      if (data.samples.empty())
      {
        event_->error("Received empty audio data, nothing to set.");
        throw perception_exception("Received empty audio data, nothing to set.");
      }

      // Lock the mutex to safely access the audio queue
      if (data.sample_rate != sample_rate_ || data.channels != channels_)
      {
        event_->error("Sample rate or channels do not match the configured values.");
        throw perception_exception("Sample rate or channels do not match the configured values.");
      }

      if (data.chunk_size != chunk_size_)
      {
        event_->error("Chunk size does not match the configured value.");
        throw perception_exception("Chunk size does not match the configured value.");
      }

      if (data.chunk_count == 0)
      {
        event_->error("Chunk count is zero, cannot set data.");
        throw perception_exception("Chunk count is zero, cannot set data.");
      }

      if (data.samples.size() < data.chunk_size)
      {
        event_->error("Samples size is less than chunk size.");
        throw perception_exception("Samples size is less than chunk size.");
      }

      if (data.samples.size() % data.chunk_size != 0)
      {
        event_->error("Samples size is not a multiple of chunk size.");
        throw perception_exception("Samples size is not a multiple of chunk size.");
      }

      // lock the mutex and insert the audio samples into the queue
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      audio_queue_.insert(audio_queue_.end(), data.samples.begin(), data.samples.end());
      buffer_cv_.notify_one();
    }
    catch (const perception_exception& error)
    {
      throw perception_exception("Invalid audio data passed to SpeakerAudioDriver::setData");
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
    }
    catch (const perception_exception& e)
    {
      event_->error("Error during test: " + std::string(e.what()));
    }

    event_->info("Test completed.");
  }

protected:
  /**
   * @brief Thread function for setting audio data for the speaker.
   *
   * This function runs in a separate thread and continuously sets audio data to the speaker.
   * It fills the audio queue with captured samples.
   */
  void driver_thread() override
  {
    while (rclcpp::ok() && is_running_)
    {
      // Ensure the audio queue is not empty before writing to the stream
      if (audio_queue_.empty())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep to avoid busy waiting
        continue;
      }

      // Lock the mutex to safely access the audio queue
      std::unique_lock<std::mutex> lock(buffer_mutex_);
      buffer_cv_.wait(lock, [this] { return audio_queue_.size() >= chunk_size_; });

      // extract the chunk size from the audio queue and write it to the PortAudio stream
      while (audio_queue_.size() >= chunk_size_)
      {
        std::vector<int16_t> chunk(audio_queue_.begin(), audio_queue_.begin() + chunk_size_);
        audio_queue_.erase(audio_queue_.begin(), audio_queue_.begin() + chunk_size_);

        err = Pa_WriteStream(stream_, chunk.data(), chunk_size_);
        if (err != paNoError)
        {
          event_->error("Error writing to audio stream: " + std::string(Pa_GetErrorText(err)));
          continue;
        }
      }
    }
  }

  mutable PaStream* stream_;
  PaError err = paNoError;
  mutable std::vector<int16_t> audio_queue_;
  unsigned long chunk_size_;  // Default chunk size
  int sample_rate_;           // Default sample rate
  int channels_;              // Default number of channels

  // Subscriber for audio data
  rclcpp::Subscription<perception_msgs::msg::PerceptionAudio>::SharedPtr audio_subscriber_;
};

}  // namespace perception