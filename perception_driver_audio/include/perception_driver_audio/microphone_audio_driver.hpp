#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_base/utils/audio.hpp>
#include <perception_msgs/msg/perception_audio.hpp>

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
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.publish", false);
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.topic", "audio/microphone");
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.frame_id", "microphone_frame");
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.frequency", 30.0);     // default frequency
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.chunk_size", 256);     // default chunk size
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.sample_rate", 44100);  // default sample rate
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.channels", 1);         // default number of channels
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.buffer_size", 10000);  // default device ID

    // Load parameters from the node
    config_.name = node->get_parameter("driver.audio.MicrophoneAudioDriver.name").as_string();
    config_.device_id = node->get_parameter("driver.audio.MicrophoneAudioDriver.device_id").as_int();
    config_.publish = node->get_parameter("driver.audio.MicrophoneAudioDriver.publish").as_bool();
    config_.topic = node->get_parameter("driver.audio.MicrophoneAudioDriver.topic").as_string();
    config_.frame_id = node->get_parameter("driver.audio.MicrophoneAudioDriver.frame_id").as_string();
    config_.frequency = node->get_parameter("driver.audio.MicrophoneAudioDriver.frequency").as_double();
    chunk_size_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.chunk_size").as_int();
    sample_rate_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.sample_rate").as_int();
    channels_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.channels").as_int();
    buffer_size_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.buffer_size").as_int();

    // Initialize the base driver
    initialize_base(node);

    // Publish about the assigned driver parameters
    event_->info("Assigned driver name: " + config_.name);
    event_->info("Assigned driver device_id: " + std::to_string(config_.device_id));
    event_->info("Assigned driver publish: " + std::string(config_.publish ? "true" : "false"));
    event_->info("Assigned driver topic: " + config_.topic);
    event_->info("Assigned driver frame_id: " + config_.frame_id);
    event_->info("Assigned driver frequency: " + std::to_string(config_.frequency));
    event_->info("Assigned driver chunk_size: " + std::to_string(chunk_size_));
    event_->info("Assigned driver sample_rate: " + std::to_string(sample_rate_));
    event_->info("Assigned driver channels: " + std::to_string(channels_));
    event_->info("Assigned driver buffer_size: " + std::to_string(buffer_size_));

    // Log that the driver has been initialized
    event_->info("Initialized");

    // If publishing is enabled, create a publisher for the audio topic
    if (config_.publish)
    {
      audio_publisher_ = node->create_publisher<perception_msgs::msg::PerceptionAudio>(config_.topic, 10);
      event_->info("Publisher created for topic: " + config_.topic);
    }
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
    inputParams.channelCount = channels_;
    inputParams.sampleFormat = paInt16;
    inputParams.suggestedLatency = Pa_GetDeviceInfo(config_.device_id)->defaultLowInputLatency;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    PaError err = Pa_OpenStream(&stream_, &inputParams, nullptr, sample_rate_, chunk_size_, paNoFlag,
                                &MicrophoneAudioDriver::paCallback, this);

    if (err != paNoError)
    {
      event_->error("Failed to open microphone stream: " + std::string(Pa_GetErrorText(err)));
      throw perception_exception("Failed to open microphone stream: " + std::string(Pa_GetErrorText(err)));
    }

    Pa_StartStream(stream_);

    event_->info("MicrophoneAudioDriver configured.");

    // Start the driver thread to capture and publish audio data
    if (config_.publish)
    {
      event_->info("MicrophoneAudioDriver will publish audio data to topic: " + config_.topic);
      driver_thread_ = std::thread(&MicrophoneAudioDriver::driver_thread, this);
    }
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

    if (config_.publish)
    {
      if (driver_thread_.joinable())
      {
        driver_thread_.join();
        event_->info("MicrophoneAudioDriver thread stopped.");
      }
    }
  }

  /**
   * @brief Get latest audio data from the driver. This function reads a single audio data chunk
   * from the microphone stream and returns it as a perception::audio_data object
   *
   * @return std::any The latest audio data from the driver as type `perception::audio_data`.
   * @throws perception_exception if the stream is not active
   */
  std::any getData() const override
  {
    std::lock_guard<std::mutex> lock(driver_mutex_);

    // Check if the audio buffer is empty
    if (audio_buffer_.empty())
      throw perception_exception("No audio data available");

    // check if the audio buffer is smaller than the chunk size
    if (audio_buffer_.size() < chunk_size_)
      throw perception_exception("Audio buffer does not contain enough data for a full chunk");

    audio_data data;
    data.sample_rate = sample_rate_;
    data.channels = channels_;
    data.chunk_size = chunk_size_;
    data.chunk_count = 1;

    // Pop the first chunk_size_ elements from the audio buffer
    for (size_t i = 0; i < chunk_size_; ++i)
    {
      data.samples.push_back(audio_buffer_.front());
      audio_buffer_.pop_front();
    }

    return data;
  }

  /**
   * @brief Get latest audio data from the driver as a stream. This function reads
   * audio data from the microphone stream and returns it as a perception::audio_data.
   *
   * @return std::any The latest audio data from the driver as type `perception::audio_data`
   * @throws perception_exception if the stream is not active
   */
  std::any getDataStream() const override
  {
    std::lock_guard<std::mutex> lock(driver_mutex_);

    // Check if the audio buffer is empty
    if (audio_buffer_.empty())
      throw perception_exception("No audio data available");

    // check if the audio buffer is smaller than the chunk size
    if (audio_buffer_.size() < chunk_size_)
      throw perception_exception("Audio buffer does not contain enough data for a full chunk");

    audio_data data;
    data.sample_rate = sample_rate_;
    data.channels = channels_;
    data.chunk_size = chunk_size_;
    data.chunk_count = 0;

    // Pop the first chunk_size_ elements from the audio buffer
    while (audio_buffer_.size() >= chunk_size_)
    {
      for (size_t i = 0; i < chunk_size_; ++i)
      {
        data.samples.push_back(audio_buffer_.front());
        audio_buffer_.pop_front();
      }
      data.chunk_count++;
    }

    return data;
  }

  /**
   * @brief driver_thread function that runs in a separate thread. used to periodically
   * publish audio data if the publish flag is set. It checks the audio buffer and publishes
   * the data as a std_msgs::msg::Int16MultiArray message.
   *
   */
  void driver_thread()
  {
    event_->info("MicrophoneAudioDriver thread started.");

    while (rclcpp::ok())
    {
      perception_msgs::msg::PerceptionAudio msg;
      msg.sample_rate = sample_rate_;
      msg.channels = channels_;
      msg.chunk_size = chunk_size_;
      msg.chunk_count = 0;

      std::lock_guard<std::mutex> publish_lock(publish_mutex_);

      while (publish_buffer_.size() >= chunk_size_)
      {
        for (size_t i = 0; i < chunk_size_; ++i)
        {
          msg.samples.push_back(publish_buffer_.front());
          publish_buffer_.pop_front();
        }
        msg.chunk_count++;
      }

      if (msg.chunk_count > 1)
      {
        event_->info("More than one chunk of audio data available, publishing all chunks.");
        audio_publisher_->publish(msg);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 / config_.frequency)));
    }

    event_->info("MicrophoneAudioDriver thread stopped.");
  }

  /**
   * @brief Test the driver
   *
   * This function is used to test the driver functionality. It can be overridden in derived classes.
   */
  void test() override
  {
    event_->info("Testing started. Please speak into the microphone. waiting 5 seconds...");

    // Wait for a short duration to capture some audio data
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Retrieve and log the audio data
    try
    {
      auto data = std::any_cast<audio_data>(getDataStream());

      // Create the "test" directory if it doesn't exist
      check_test_directory("test");

      // Write the data to a file for further analysis
      writeWavFile("test/mic_test.wav", data);

      event_->info("Audio data written to file: microphone_audio_data.wav");
    }
    catch (const perception_exception& e)
    {
      event_->error("Error during test: " + std::string(e.what()));
    }

    event_->info("Test completed.");
  }

protected:
  /**
   * @brief Callback function for PortAudio to process audio input.
   *
   * This function is called by PortAudio when there is audio data available to read.
   * It reads the audio data from the input buffer and stores it in the audio queue.
   *
   * @param inputBuffer Pointer to the input buffer containing audio data.
   * @param outputBuffer Pointer to the output buffer (not used in this case).
   * @param framesPerBuffer Number of frames per buffer.
   * @param timeInfo Pointer to time information (not used in this case).
   * @param statusFlags Status flags (not used in this case).
   * @param userData Pointer to user data (this instance of SpeakerAudioDriver).
   * @return int Returns paContinue to keep the stream open.
   */
  static int paCallback(const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData)
  {
    auto* self = static_cast<MicrophoneAudioDriver*>(userData);
    const auto* in = static_cast<const int16_t*>(inputBuffer);

    if (in)
    {
      std::lock_guard<std::mutex> lock(self->driver_mutex_);

      self->audio_buffer_.insert(self->audio_buffer_.end(), in, in + framesPerBuffer);

      if (self->audio_buffer_.size() > self->buffer_size_)
      {
        // If the audio buffer exceeds the buffer size, remove the oldest data
        self->audio_buffer_.erase(self->audio_buffer_.begin(),
                                  self->audio_buffer_.begin() + (self->audio_buffer_.size() - self->buffer_size_));
      }

      // If publishing is enabled, add the audio data to the publish buffer
      if (self->config_.publish)
      {
        std::lock_guard<std::mutex> publish_lock(self->publish_mutex_);
        self->publish_buffer_.insert(self->publish_buffer_.end(), in, in + framesPerBuffer);
      }
    }

    return paContinue;
  }

  PaStream* stream_;
  mutable std::deque<int16_t> audio_buffer_;
  mutable std::deque<int16_t> publish_buffer_;
  mutable std::mutex publish_mutex_;
  unsigned long chunk_size_;  // Default chunk size
  int sample_rate_;           // Default sample rate
  int channels_;              // Default number of channels
  int buffer_size_;           // Default buffer size

  // Publisher for audio data
  rclcpp::Publisher<perception_msgs::msg::PerceptionAudio>::SharedPtr audio_publisher_;
};

}  // namespace perception
