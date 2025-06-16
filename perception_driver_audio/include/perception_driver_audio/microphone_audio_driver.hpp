#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_base/utils/audio/structs.hpp>
#include <perception_base/utils/audio/wav.hpp>
#include <perception_msgs/msg/perception_audio.hpp>
#include <perception_driver_audio/utils.hpp>

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
  /**
   * @brief Constructor for MicrophoneAudioDriver
   *
   * Initializes the PortAudio library and prepares the audio stream.
   */
  MicrophoneAudioDriver() : stream_(nullptr)
  {
  }

  /**
   * @brief Start the audio stream
   *
   * This function starts the audio stream using PortAudio.
   */
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
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.device_name", "default");
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.publish", false);
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.topic", "audio/microphone");
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.frame_id", "microphone_frame");
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.chunk_size", 256);     // default chunk size
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.sample_rate", 44100);  // default sample rate
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.channels", 1);         // default number of channels
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.buffer_time", 10);     // default device ID

    // Load parameters from the node
    config_.name = node->get_parameter("driver.audio.MicrophoneAudioDriver.name").as_string();
    config_.device_name = node->get_parameter("driver.audio.MicrophoneAudioDriver.device_name").as_string();
    config_.publish = node->get_parameter("driver.audio.MicrophoneAudioDriver.publish").as_bool();
    config_.topic = node->get_parameter("driver.audio.MicrophoneAudioDriver.topic").as_string();
    config_.frame_id = node->get_parameter("driver.audio.MicrophoneAudioDriver.frame_id").as_string();
    chunk_size_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.chunk_size").as_int();
    sample_rate_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.sample_rate").as_int();
    channels_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.channels").as_int();
    buffer_time_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.buffer_time").as_int();

    buffer_size_ = sample_rate_ * channels_ * buffer_time_;

    // Initialize the base driver
    initialize_base(node);

    // Initialize PortAudio
    err = Pa_Initialize();
    if (err != paNoError)
    {
      event_->error("PortAudio initialization failed: " + std::string(Pa_GetErrorText(err)));
      throw perception_exception("PortAudio initialization failed: " + std::string(Pa_GetErrorText(err)));
    }

    // Get the device ID by name
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
    event_->info("Assigned driver publish: " + std::string(config_.publish ? "true" : "false"));
    event_->info("Assigned driver topic: " + config_.topic);
    event_->info("Assigned driver frame_id: " + config_.frame_id);
    event_->info("Assigned driver chunk_size: " + std::to_string(chunk_size_));
    event_->info("Assigned driver sample_rate: " + std::to_string(sample_rate_));
    event_->info("Assigned driver channels: " + std::to_string(channels_));
    event_->info("Assigned driver buffer_time: " + std::to_string(buffer_time_));
    event_->info("Assigned driver buffer size: " + std::to_string(buffer_size_));

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
    event_->info("starting on device " + std::to_string(config_.device_id));

    PaStreamParameters inputParams;
    inputParams.device = config_.device_id;
    inputParams.channelCount = channels_;
    inputParams.sampleFormat = paInt16;
    inputParams.suggestedLatency = Pa_GetDeviceInfo(config_.device_id)->defaultLowInputLatency;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&stream_, &inputParams, nullptr, sample_rate_, chunk_size_, paNoFlag, nullptr, nullptr);
    if (err != paNoError)
    {
      event_->error("Failed to open microphone stream: " + std::string(Pa_GetErrorText(err)));
      throw perception_exception("Failed to open microphone stream: " + std::string(Pa_GetErrorText(err)));
    }

    err = Pa_StartStream(stream_);
    if (err != paNoError)
    {
      event_->error("Failed to start microphone stream: " + std::string(Pa_GetErrorText(err)));
      throw perception_exception("Failed to start microphone stream: " + std::string(Pa_GetErrorText(err)));
    }

    if (!Pa_IsStreamActive(stream_))
    {
      event_->error("Stream is not active after starting.");
      throw perception_exception("PortAudio stream failed to activate.");
    }

    // Start the driver thread to capture and publish audio data
    event_->info("starting driver thread for audio capture...");
    is_running_ = true;
    driver_thread_ = std::thread(&MicrophoneAudioDriver::driver_thread, this);

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
   * @brief Get latest audio data from the driver as a stream. This function reads
   * audio data from the microphone stream and returns it as a perception::audio_data. Best
   * approach is to use this to get a certain amount of audio data by repeatedly calling
   * this function until enough data is collected.
   *
   * @return std::any The latest audio data from the driver as type `perception::audio_data`
   * @throws perception_exception if the stream is not active
   */
  std::any getDataStream()  override
  {
    // Check if the audio buffer is empty and proceed only if it has enough data
    std::unique_lock<std::mutex> lock(buffer_mutex_);

    if (!buffer_cv_.wait_for(lock, std::chrono::seconds(5), [this] { return audio_buffer_.size() >= chunk_size_; }))
    {
      throw perception_exception("Timeout waiting for audio data.");
    }

    audio_data data;
    data.sample_rate = sample_rate_;
    data.channels = channels_;
    data.chunk_size = chunk_size_;
    data.chunk_count = 0;

    while (audio_buffer_.size() >= chunk_size_)
    {
      auto begin_iter = audio_buffer_.begin();
      auto end_iter = begin_iter + chunk_size_;

      // Append the chunk to the output data
      data.samples.insert(data.samples.end(), begin_iter, end_iter);

      // Erase the chunk from the buffer in one go
      audio_buffer_.erase(begin_iter, end_iter);

      data.chunk_count++;
    }

    return data;
  }

  /**
   * @brief Test the driver
   *
   * This function is used to test the driver functionality. It can be overridden in derived classes.
   */
  void test() override
  {
    event_->info("Testing started. Please speak into the microphone. waiting 5 seconds...");

    // amount of frames required for 5 seconds of audio
    unsigned long frames_to_capture = sample_rate_ * channels_ * 5;  // 5 seconds of audio
    unsigned long frames_captured = 0;

    audio_data accumilated_samples;

    // Retrieve and log the audio data
    while (frames_captured < frames_to_capture)
    {
      try
      {
        auto data = std::any_cast<audio_data>(getDataStream());

        frames_captured += (data.chunk_size * data.chunk_count);

        accumilated_samples.samples.insert(accumilated_samples.samples.end(), data.samples.begin(), data.samples.end());
        accumilated_samples.sample_rate = data.sample_rate;
        accumilated_samples.channels = data.channels;
        accumilated_samples.chunk_size = data.chunk_size;
        accumilated_samples.chunk_count += data.chunk_count;

        event_->info("Captured " + std::to_string(frames_captured) + "/" + std::to_string(frames_to_capture) +
                     " frames of audio data.");
      }
      catch (const perception_exception& e)
      {
        event_->error("Error during test: " + std::string(e.what()));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;  // Retry getting data if an error occurs
      }
    }

    // Create the "test" directory if it doesn't exist
    check_directory("test");

    // Write the data to a file for further analysis
    writeWavFile("test/mic_test.wav", accumilated_samples);

    event_->info("Audio data written to file: test/mic_test.wav");

    event_->info("Test completed.");
  }

protected:
  void driver_thread() override
  {
    while (rclcpp::ok() && is_running_)
    {
      // Read audio data from the PortAudio stream
      std::vector<int16_t> buffer(chunk_size_ * channels_);

      err = Pa_ReadStream(stream_, buffer.data(), chunk_size_);

      if (err != paNoError)
      {
        event_->error("Error reading from PortAudio stream: " + std::string(Pa_GetErrorText(err)));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      // Lock the mutex to safely access the audio buffer
      std::lock_guard<std::mutex> lock(buffer_mutex_);

      // Add the read samples to the audio buffer
      audio_buffer_.insert(audio_buffer_.end(), buffer.begin(), buffer.end());

      if (audio_buffer_.size() > buffer_size_)
      {
        // If the buffer exceeds the maximum size, remove the 2s oldest samples
        audio_buffer_.erase(audio_buffer_.begin(), audio_buffer_.begin() + chunk_size_);
      }

      buffer_cv_.notify_one();

      // If publishing is enabled, publish the audio data
      if (config_.publish)
      {
        perception_msgs::msg::PerceptionAudio msg;
        msg.header.stamp = rclcpp::Clock().now();
        msg.header.frame_id = config_.frame_id;
        msg.sample_rate = sample_rate_;
        msg.channels = channels_;
        msg.chunk_size = chunk_size_;
        msg.chunk_count = 1;
        msg.samples = buffer;

        audio_publisher_->publish(msg);
      }
    }

    event_->info("driver thread stopped.");
  }

  PaStream* stream_;
  PaError err = paNoError;
  std::deque<int16_t> audio_buffer_;
  std::mutex publish_mutex_;
  unsigned long chunk_size_;  // Default chunk size
  int sample_rate_;           // Default sample rate
  int channels_;              // Default number of channels
  int buffer_time_;           // Buffer time in seconds
  int buffer_size_;           // Buffer size in samples

  // Publisher for audio data
  rclcpp::Publisher<perception_msgs::msg::PerceptionAudio>::SharedPtr audio_publisher_;
};

}  // namespace perception
