#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <cmath>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <perception_base/audio/audio_source_driver.hpp>
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
class MicrophoneAudioDriver : public AudioSourceDriver
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
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.device_id", -1);
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.device_name", "");
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.publish", false);
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.chunk_size", 256);     // default chunk size
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.sample_rate", 44100);  // default sample rate
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.channels", 1);         // default number of channels
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.buffer_time", 10);     // default buffer time in seconds
    node->declare_parameter("driver.audio.MicrophoneAudioDriver.timeout_sec", -1);     // -1 waits indefinitely
    // Load parameters from the node
    name_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.name").as_string();
    device_id_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.device_id").as_int();
    device_name_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.device_name").as_string();
    chunk_size_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.chunk_size").as_int();
    sample_rate_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.sample_rate").as_int();
    channels_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.channels").as_int();
    buffer_time_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.buffer_time").as_int();
    timeout_sec_ = node->get_parameter("driver.audio.MicrophoneAudioDriver.timeout_sec").as_int();

    // Initialize the base driver
    initialize_base(node);

    // Initialize PortAudio
    err = Pa_Initialize();
    if (err != paNoError)
    {
      RCLCPP_ERROR(node_->get_logger(), "PortAudio initialization failed: %s", Pa_GetErrorText(err));
      throw perception_exception("PortAudio initialization failed: " + std::string(Pa_GetErrorText(err)));
    }

    const int device_count = Pa_GetDeviceCount();
    if (device_count < 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Pa_GetDeviceCount failed: %s", Pa_GetErrorText(device_count));
      throw perception_exception("PortAudio failed to enumerate devices");
    }

    logPortAudioDevices(node_->get_logger());

    int resolved_device_id = device_id_;
    if (!device_name_.empty())
    {
      try
      {
        resolved_device_id = getDeviceIdByName(device_name_);
        RCLCPP_INFO(node_->get_logger(), "Resolved microphone device_name '%s' to device_id %d.",
                    device_name_.c_str(), resolved_device_id);
      }
      catch (const perception_exception& e)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Failed to map microphone device_name '%s' to an ID: %s. Falling back to default input device.",
                    device_name_.c_str(), e.what());
        resolved_device_id = Pa_GetDefaultInputDevice();
      }
    }
    else if (resolved_device_id < 0 || resolved_device_id >= device_count)
    {
      resolved_device_id = Pa_GetDefaultInputDevice();
    }

    if (resolved_device_id < 0 || resolved_device_id >= device_count)
    {
      for (int i = 0; i < device_count; ++i)
      {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (info && info->maxInputChannels > 0)
        {
          resolved_device_id = i;
          RCLCPP_WARN(node_->get_logger(), "Falling back to first available microphone device_id %d ('%s').",
                      resolved_device_id, info->name);
          break;
        }
      }
    }

    const PaDeviceInfo* device_info = Pa_GetDeviceInfo(resolved_device_id);
    if (!device_info)
    {
      throw perception_exception("PortAudio returned null device info for device_id " + std::to_string(resolved_device_id));
    }
    if (resolved_device_id != device_id_)
    {
      RCLCPP_WARN(node_->get_logger(), "Microphone device id %d was replaced by %d (%s).", device_id_, resolved_device_id,
                  device_info->name);
      device_id_ = resolved_device_id;
    }
    if (device_info->maxInputChannels <= 0)
    {
      throw perception_exception("Selected device_id " + std::to_string(device_id_) + " ('" + device_info->name +
                                 "') has no input channels");
    }

    // Initial expected buffer size. It is recomputed after stream open if capture_sample_rate_ changes.
    buffer_size_ = sample_rate_ * channels_ * buffer_time_;

    // Publish about the assigned driver parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver name: %s", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver device_id: %d", device_id_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver device: %s", describePortAudioDevice(device_id_).c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver chunk_size: %lu", chunk_size_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver sample_rate: %d", sample_rate_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver channels: %d", channels_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver buffer_time: %d", buffer_time_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver timeout_sec: %d", timeout_sec_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver buffer size: %d", buffer_size_);

    RCLCPP_INFO(node_->get_logger(), "starting on device %d", device_id_);

    PaStreamParameters inputParams;
    inputParams.device = device_id_;
    inputParams.channelCount = channels_;
    inputParams.sampleFormat = paInt16;
    inputParams.suggestedLatency = device_info->defaultLowInputLatency;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    // Try to open with the configured sample rate first; if the device rejects it,
    // fall back to the device default sample rate and resample to the configured rate
    // when returning data.
    capture_sample_rate_ = sample_rate_;
    err = Pa_OpenStream(&stream_, &inputParams, nullptr, sample_rate_, chunk_size_, paNoFlag,
              &MicrophoneAudioDriver::pa_input_callback, this);
    if (err == paInvalidSampleRate)
    {
      const PaDeviceInfo* info = Pa_GetDeviceInfo(device_id_);
      const int fallback_rate = info ? static_cast<int>(std::lround(info->defaultSampleRate)) : sample_rate_;
      RCLCPP_WARN(node_->get_logger(),
                  "Requested mic sample_rate (%d) not supported by device '%s'. Falling back to %d and resampling.",
                  sample_rate_, (info && info->name) ? info->name : "unknown", fallback_rate);

      capture_sample_rate_ = fallback_rate;
        err = Pa_OpenStream(&stream_, &inputParams, nullptr, capture_sample_rate_, chunk_size_, paNoFlag,
                  &MicrophoneAudioDriver::pa_input_callback, this);
    }

    if (err != paNoError)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open microphone stream: %s", Pa_GetErrorText(err));
      throw perception_exception("Failed to open microphone stream: " + std::string(Pa_GetErrorText(err)));
    }

    // Buffer stores captured samples at the capture sample rate.
    buffer_size_ = capture_sample_rate_ * channels_ * buffer_time_;
    RCLCPP_INFO(node_->get_logger(), "Assigned driver capture_sample_rate: %d", capture_sample_rate_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver final buffer size: %d samples", buffer_size_);

    // The callback can be invoked as soon as Pa_StartStream succeeds.
    is_running_ = true;
    err = Pa_StartStream(stream_);
    if (err != paNoError)
    {
      is_running_ = false;
      RCLCPP_ERROR(node_->get_logger(), "Failed to start microphone stream: %s", Pa_GetErrorText(err));
      throw perception_exception("Failed to start microphone stream: " + std::string(Pa_GetErrorText(err)));
    }

    if (!Pa_IsStreamActive(stream_))
    {
      is_running_ = false;
      RCLCPP_ERROR(node_->get_logger(), "Stream is not active after starting.");
      throw perception_exception("PortAudio stream failed to activate.");
    }

    // Start callback-driven audio capture.
    RCLCPP_INFO(node_->get_logger(), "starting callback-driven audio capture...");

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
    buffer_cv_.notify_all();

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
  audio_data readChunk() override
  {
    // Check if the audio buffer is empty and proceed only if it has enough data
    std::unique_lock<std::mutex> lock(buffer_mutex_);

    const size_t chunk_samples = static_cast<size_t>(chunk_size_) * static_cast<size_t>(std::max(1, channels_));

    const auto has_enough_audio = [this, chunk_samples] { return audio_buffer_.size() >= chunk_samples || !is_running_; };
    if (timeout_sec_ < 0)
    {
      buffer_cv_.wait(lock, has_enough_audio);
    }
    else
    {
      if (!buffer_cv_.wait_for(lock, std::chrono::seconds(timeout_sec_), has_enough_audio))
      {
        throw perception_exception("Timeout waiting for audio data.");
      }
    }

    if (audio_buffer_.size() < chunk_samples && !is_running_)
      throw perception_exception("Microphone stream is not running.");

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
      data.chunk_size =
          std::max<int>(1, static_cast<int>(data.samples.size() / static_cast<size_t>(std::max(1, channels_))));
      data.chunk_count = 1;
    }
    else
    {
      data.samples = std::move(captured);
      data.sample_rate = capture_sample_rate_;
      data.chunk_size =
          std::max<int>(1, static_cast<int>(data.samples.size() / static_cast<size_t>(std::max(1, channels_))));
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

    // Amount of frames required for 5 seconds of audio. Frames already include all channels.
    unsigned long frames_to_capture = sample_rate_ * 5;  // 5 seconds of audio
    unsigned long frames_captured = 0;

    audio_data accumilated_samples;

    // Retrieve and log the audio data
    while (frames_captured < frames_to_capture)
    {
      try
      {
        auto data = readChunk();

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

    int max_abs = 0;
    long double sum_squares = 0.0;
    for (const auto sample : accumilated_samples.samples)
    {
      max_abs = std::max(max_abs, std::abs(static_cast<int>(sample)));
      sum_squares += static_cast<long double>(sample) * static_cast<long double>(sample);
    }
    const double rms = accumilated_samples.samples.empty() ? 0.0 :
                                                            std::sqrt(static_cast<double>(sum_squares /
                                                                                          accumilated_samples.samples.size()));
    RCLCPP_INFO(node_->get_logger(), "Microphone test signal stats: samples=%zu max_abs=%d rms=%.2f",
                accumilated_samples.samples.size(), max_abs, rms);
    if (max_abs < 128)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Microphone test appears nearly silent. Check capture device_id, input source, gain, mute state, and channel count.");
    }

    // Write the data to a file for further analysis
    writeWavFile("test/mic_test.wav", accumilated_samples);

    RCLCPP_INFO(node_->get_logger(), "Audio data written to file: test/mic_test.wav");

    RCLCPP_INFO(node_->get_logger(), "Test completed.");
  }

protected:
  static int pa_input_callback(const void* input_buffer, void* output_buffer, unsigned long frames_per_buffer,
                               const PaStreamCallbackTimeInfo* time_info, PaStreamCallbackFlags status_flags,
                               void* user_data)
  {
    (void)output_buffer;
    (void)time_info;

    auto* driver = static_cast<MicrophoneAudioDriver*>(user_data);
    if (!driver)
      return paAbort;

    return driver->capture_input_buffer(input_buffer, frames_per_buffer, status_flags);
  }

  int capture_input_buffer(const void* input_buffer, unsigned long frames_per_buffer, PaStreamCallbackFlags status_flags)
  {
    if (!is_running_)
      return paComplete;

    if (status_flags & paInputOverflow)
      input_overflow_count_++;

    if (!input_buffer || frames_per_buffer == 0)
      return paContinue;

    const auto* samples = static_cast<const int16_t*>(input_buffer);
    const size_t sample_count = static_cast<size_t>(frames_per_buffer) * static_cast<size_t>(std::max(1, channels_));

    std::unique_lock<std::mutex> lock(buffer_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      dropped_callback_chunks_++;
      return paContinue;
    }

    audio_buffer_.insert(audio_buffer_.end(), samples, samples + sample_count);

    if (audio_buffer_.size() > static_cast<size_t>(std::max(0, buffer_size_)))
    {
      const size_t excess = audio_buffer_.size() - static_cast<size_t>(std::max(0, buffer_size_));
      const size_t channels = static_cast<size_t>(std::max(1, channels_));
      const size_t samples_to_remove = (excess / channels) * channels;
      audio_buffer_.erase(audio_buffer_.begin(),
                          audio_buffer_.begin() + static_cast<std::ptrdiff_t>(samples_to_remove));
    }

    lock.unlock();
    buffer_cv_.notify_one();

    return paContinue;
  }

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

  PaStream* stream_;
  PaError err = paNoError;
  std::deque<int16_t> audio_buffer_;
  std::mutex publish_mutex_;
  int device_id_;                 // Device ID for the microphone
  std::string device_name_;
  unsigned long chunk_size_;      // Default chunk size
  int sample_rate_;               // Default sample rate
  int capture_sample_rate_{ 0 };  // Actual device capture rate (may differ from sample_rate_)
  int channels_;                  // Default number of channels
  int buffer_time_;               // Buffer time in seconds
  int timeout_sec_{ -1 };         // Wait timeout for audio data (-1 waits indefinitely)
  int buffer_size_{ 0 };          // Buffer size in samples
  std::atomic<uint64_t> input_overflow_count_{ 0 };
  std::atomic<uint64_t> dropped_callback_chunks_{ 0 };
};

}  // namespace perception
