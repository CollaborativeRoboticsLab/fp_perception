#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_base/audio/structs.hpp>
#include <perception_base/audio/wav.hpp>
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
    deinitialize();
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
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.chunk_size", 256);     // default chunk size
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.sample_rate", 44100);  // default sample rate
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.channels", 1);         // default number of channels
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.buffer_time", 10);     // default buffer time in seconds
    // Load parameters from the node
    name_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.name").as_string();
    device_name_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.device_name").as_string();
    chunk_size_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.chunk_size").as_int();
    sample_rate_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.sample_rate").as_int();
    channels_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.channels").as_int();
    buffer_time_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.buffer_time").as_int();

    // Initialize the base driver
    initialize_base(node);

    // Initialize PortAudio
    err = Pa_Initialize();
    if (err != paNoError)
    {
      RCLCPP_ERROR(node_->get_logger(), "PortAudio initialization failed: %s", Pa_GetErrorText(err));
      throw perception_exception("PortAudio initialization failed: " + std::string(Pa_GetErrorText(err)));
    }

    // Get the device ID by name
    try
    {
      device_id_ = perception::getDeviceIdByName(device_name_);
      RCLCPP_INFO(node_->get_logger(), "Device ID for name '%s' is %d", device_name_.c_str(), device_id_);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get device ID for name '%s': %s", device_name_.c_str(), e.what());
      throw perception_exception("Failed to get device ID for name '" + device_name_ + "': " + e.what());
    }

    // Publish about the assigned driver parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver name: %s", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver device_name: %s", device_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver device_id: %d", device_id_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver chunk_size: %d", chunk_size_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver sample_rate: %d", sample_rate_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver channels: %d", channels_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver buffer_time: %d", buffer_time_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver buffer size: %d", buffer_size_);

    RCLCPP_INFO(node_->get_logger(), "starting on device %d", device_id_);

    PaStreamParameters inputParams;
    inputParams.device = device_id_;
    inputParams.channelCount = channels_;
    inputParams.sampleFormat = paInt16;
    inputParams.suggestedLatency = Pa_GetDeviceInfo(device_id_)->defaultLowInputLatency;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    // Try to open with the configured sample rate first; if the device rejects it,
    // fall back to the device default sample rate and resample to the configured rate
    // when returning data.
    capture_sample_rate_ = sample_rate_;
    err = Pa_OpenStream(&stream_, &inputParams, nullptr, sample_rate_, chunk_size_, paNoFlag, nullptr, nullptr);
    if (err == paInvalidSampleRate)
    {
      const PaDeviceInfo* info = Pa_GetDeviceInfo(device_id_);
      const int fallback_rate = info ? static_cast<int>(std::lround(info->defaultSampleRate)) : sample_rate_;
      RCLCPP_WARN(node_->get_logger(),
                  "Requested mic sample_rate (%d) not supported by device '%s'. Falling back to %d and resampling.",
                  sample_rate_, device_name_.c_str(), fallback_rate);

      capture_sample_rate_ = fallback_rate;
      err = Pa_OpenStream(&stream_, &inputParams, nullptr, capture_sample_rate_, chunk_size_, paNoFlag, nullptr, nullptr);
    }

    if (err != paNoError)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open microphone stream: %s", Pa_GetErrorText(err));
      throw perception_exception("Failed to open microphone stream: " + std::string(Pa_GetErrorText(err)));
    }

    // Buffer stores captured samples at the capture sample rate.
    buffer_size_ = capture_sample_rate_ * channels_ * buffer_time_;

    err = Pa_StartStream(stream_);
    if (err != paNoError)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to start microphone stream: %s", Pa_GetErrorText(err));
      throw perception_exception("Failed to start microphone stream: " + std::string(Pa_GetErrorText(err)));
    }

    if (!Pa_IsStreamActive(stream_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Stream is not active after starting.");
      throw perception_exception("PortAudio stream failed to activate.");
    }

    // Start the driver thread to capture and publish audio data
    RCLCPP_INFO(node_->get_logger(), "starting driver thread for audio capture...");
    is_running_ = true;
    driver_thread_ = std::thread(&MicrophoneAudioDriver::driver_thread, this);

    // Log that the driver has been initialized
    RCLCPP_INFO(node_->get_logger(), "Initialized");
  }

  /**
   * @brief Stop driver streaming. This function stops the audio stream and closes it.
   * It also terminates the PortAudio library.
   */
  void deinitialize() override
  {
    is_running_ = false;

    if (driver_thread_.joinable())
    {
      driver_thread_.join();
      RCLCPP_INFO(node_->get_logger(), "thread stopped.");
    }

    if (stream_)
    {
      Pa_StopStream(stream_);
      Pa_CloseStream(stream_);
      stream_ = nullptr;
      RCLCPP_INFO(node_->get_logger(), "stopped.");
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
  std::any getDataStream() override
  {
    // Check if the audio buffer is empty and proceed only if it has enough data
    std::unique_lock<std::mutex> lock(buffer_mutex_);

    const size_t chunk_samples = static_cast<size_t>(chunk_size_) * static_cast<size_t>(std::max(1, channels_));

    if (!buffer_cv_.wait_for(lock, std::chrono::seconds(5), [this, chunk_samples] {
          return audio_buffer_.size() >= chunk_samples;
        }))
    {
      throw perception_exception("Timeout waiting for audio data.");
    }

    audio_data data;
    data.sample_rate = sample_rate_;
    data.channels = channels_;
    data.chunk_size = chunk_size_;
    data.chunk_count = 0;

    std::vector<int16_t> captured;
    while (audio_buffer_.size() >= chunk_samples)
    {
      auto begin_iter = audio_buffer_.begin();
      auto end_iter = begin_iter + static_cast<std::ptrdiff_t>(chunk_samples);

      // Append the chunk to the output data
      captured.insert(captured.end(), begin_iter, end_iter);

      // Erase the chunk from the buffer in one go
      audio_buffer_.erase(begin_iter, end_iter);

      data.chunk_count++;
    }

    // If the capture sample rate differs from the configured sample rate, resample.
    if (capture_sample_rate_ != sample_rate_)
    {
      data.samples = resample_linear_interleaved(captured, channels_, capture_sample_rate_, sample_rate_);
      data.sample_rate = sample_rate_;
      data.chunk_size = std::max<int>(1, static_cast<int>(data.samples.size() / static_cast<size_t>(std::max(1, channels_))));
      data.chunk_count = 1;
    }
    else
    {
      data.samples = std::move(captured);
      data.sample_rate = capture_sample_rate_;
      data.chunk_size = std::max<int>(1, static_cast<int>(data.samples.size() / static_cast<size_t>(std::max(1, channels_))));
      data.chunk_count = 1;
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
    RCLCPP_INFO(node_->get_logger(), "Testing started. Please speak into the microphone. waiting 5 seconds...");
    
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

        RCLCPP_INFO(node_->get_logger(), "Captured %lu/%lu frames of audio data.", frames_captured, frames_to_capture);
      }
      catch (const perception_exception& e)
      {
        RCLCPP_ERROR(node_->get_logger(), "Error during test: %s", e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;  // Retry getting data if an error occurs
      }
    }

    // Create the "test" directory if it doesn't exist
    check_directory("test");

    // Write the data to a file for further analysis
    writeWavFile("test/mic_test.wav", accumilated_samples);

    RCLCPP_INFO(node_->get_logger(), "Audio data written to file: test/mic_test.wav");

    RCLCPP_INFO(node_->get_logger(), "Test completed.");
  }

protected:
  static std::vector<int16_t> resample_linear_interleaved(const std::vector<int16_t>& input, int channels,
                                                          int input_rate, int output_rate)
  {
    if (input_rate <= 0 || output_rate <= 0)
      throw perception_exception("Invalid sample rate for resampling");

    channels = std::max(1, channels);
    const size_t input_frames = input.size() / static_cast<size_t>(channels);

    if (input_frames == 0 || input_rate == output_rate)
      return input;

    const double ratio = static_cast<double>(output_rate) / static_cast<double>(input_rate);
    const size_t output_frames = std::max<size_t>(1, static_cast<size_t>(std::llround(input_frames * ratio)));
    std::vector<int16_t> output(output_frames * static_cast<size_t>(channels));

    for (int ch = 0; ch < channels; ++ch)
    {
      for (size_t out_i = 0; out_i < output_frames; ++out_i)
      {
        const double src_pos = static_cast<double>(out_i) / ratio;
        const size_t i0 = static_cast<size_t>(std::floor(src_pos));
        const size_t i1 = std::min(i0 + 1, input_frames - 1);
        const double frac = src_pos - static_cast<double>(i0);

        const int16_t s0 = input[i0 * static_cast<size_t>(channels) + static_cast<size_t>(ch)];
        const int16_t s1 = input[i1 * static_cast<size_t>(channels) + static_cast<size_t>(ch)];

        const double mixed = (1.0 - frac) * static_cast<double>(s0) + frac * static_cast<double>(s1);
        const long rounded = std::lround(mixed);
        const long clamped = std::clamp<long>(rounded, -32768, 32767);

        output[out_i * static_cast<size_t>(channels) + static_cast<size_t>(ch)] = static_cast<int16_t>(clamped);
      }
    }

    return output;
  }

  void driver_thread()
  {
    while (rclcpp::ok() && is_running_)
    {
      // Read audio data from the PortAudio stream
      std::vector<int16_t> buffer(static_cast<size_t>(chunk_size_) * static_cast<size_t>(std::max(1, channels_)));

      err = Pa_ReadStream(stream_, buffer.data(), chunk_size_);

      if (err != paNoError)
      {
        RCLCPP_ERROR(node_->get_logger(), "Error reading from PortAudio stream: %s", Pa_GetErrorText(err));
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
    }

    RCLCPP_INFO(node_->get_logger(), "driver thread stopped.");
  }

  PaStream* stream_;
  PaError err = paNoError;
  std::deque<int16_t> audio_buffer_;
  std::mutex publish_mutex_;
  int device_id_;             // Device ID for the microphone
  unsigned long chunk_size_;  // Default chunk size
  int sample_rate_;           // Default sample rate
  int capture_sample_rate_{ 0 };  // Actual device capture rate (may differ from sample_rate_)
  int channels_;              // Default number of channels
  int buffer_time_;           // Buffer time in seconds
  int buffer_size_;           // Buffer size in samples
};

}  // namespace perception
