#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <map>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <atomic>
#include <cstdint>
#include <perception_base/audio/audio_sink_driver.hpp>
#include <perception_base/audio/structs.hpp>
#include <perception_base/audio/wav.hpp>
#include <perception_driver_audio/utils.hpp>

namespace perception
{

/**
 * @brief SpeakerAudioDriver class for handling audio output to a speaker.
 *
 * This class is responsible for managing the audio output to a speaker using PortAudio.
 * It provides methods to start and stop the audio stream, as well as play audio data.
 */
class SpeakerAudioDriver : public AudioSinkDriver
{
public:
  /**
   * @brief Constructor for SpeakerAudioDriver
   *
   * Initializes the PortAudio library and prepares the audio stream.
   */
  SpeakerAudioDriver()
  {
  }

  /**
   * @brief Destructor for SpeakerAudioDriver
   *
   * Stops the audio stream and terminates the PortAudio library.
   */
  ~SpeakerAudioDriver() override
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
    node->declare_parameter("driver.audio.SpeakerAudioDriver.name", "SpeakerAudioDriver");
    node->declare_parameter("driver.audio.SpeakerAudioDriver.device_id", -1);
    node->declare_parameter("driver.audio.SpeakerAudioDriver.device_name", "");
    node->declare_parameter("driver.audio.SpeakerAudioDriver.sample_rate", 44100);  // default sample rate
    node->declare_parameter("driver.audio.SpeakerAudioDriver.channels", 1);         // default number of channels
    node->declare_parameter("driver.audio.SpeakerAudioDriver.test_file_path",
                            "test/mic_test.wav");  // default device ID

    // Load parameters from the node
    name_ = node->get_parameter("driver.audio.SpeakerAudioDriver.name").as_string();
    device_id_ = node->get_parameter("driver.audio.SpeakerAudioDriver.device_id").as_int();
    device_name_ = node->get_parameter("driver.audio.SpeakerAudioDriver.device_name").as_string();
    sample_rate_ = node->get_parameter("driver.audio.SpeakerAudioDriver.sample_rate").as_int();
    channels_ = node->get_parameter("driver.audio.SpeakerAudioDriver.channels").as_int();
    test_file_path_ = node->get_parameter("driver.audio.SpeakerAudioDriver.test_file_path").as_string();

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
        RCLCPP_INFO(node_->get_logger(), "Resolved speaker device_name '%s' to device_id %d.",
                    device_name_.c_str(), resolved_device_id);
      }
      catch (const perception_exception& e)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Failed to map speaker device_name '%s' to an ID: %s. Falling back to default output device.",
                    device_name_.c_str(), e.what());
        resolved_device_id = Pa_GetDefaultOutputDevice();
      }
    }
    else if (resolved_device_id < 0 || resolved_device_id >= device_count)
    {
      resolved_device_id = Pa_GetDefaultOutputDevice();
    }

    if (resolved_device_id < 0 || resolved_device_id >= device_count)
    {
      for (int i = 0; i < device_count; ++i)
      {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (info && info->maxOutputChannels > 0)
        {
          resolved_device_id = i;
          RCLCPP_WARN(node_->get_logger(), "Falling back to first available speaker device_id %d ('%s').",
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
      RCLCPP_WARN(node_->get_logger(), "Speaker device id %d was replaced by %d (%s).", device_id_, resolved_device_id,
                  device_info->name);
      device_id_ = resolved_device_id;
    }
    if (device_info->maxOutputChannels <= 0)
    {
      throw perception_exception("Selected device_id " + std::to_string(device_id_) + " ('" + device_info->name +
                                 "') has no output channels");
    }

    // Publish about the assigned driver parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver name: %s", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver device_id: %d", device_id_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver device: %s", describePortAudioDevice(device_id_).c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver sample_rate: %d", sample_rate_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver channels: %d", channels_);

    if (diagnostics_enabled())
      setup_diagnostics();

    RCLCPP_INFO(node_->get_logger(), "Initialized");
  }

  /**
   * @brief Stop driver streaming. This function stops the audio stream and closes it.
   * It also terminates the PortAudio library.
   */
  void deinitialize() override
  {
    disable_diagnostics();

    for (auto& pair : stream_dict_)
    {
      if (!pair.second)
        continue;

      err = Pa_StopStream(pair.second);
      if (err != paNoError)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to stop speaker stream: %s", Pa_GetErrorText(err));
      }

      err = Pa_CloseStream(pair.second);
      if (err != paNoError)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to close speaker stream: %s", Pa_GetErrorText(err));
      }
      pair.second = nullptr;
    }

    stream_dict_.clear();

    RCLCPP_INFO(node_->get_logger(), "stopped.");
  }

  /**
   * @brief Set the latest audio data to the driver. This function sends the latest audio data to the speaker driver.
   *
   * @param input The latest audio data to the driver in the form of `const std::vector<std::vector<int16_t>>&`.
   * @throws perception_exception if not implemented in derived classes
   */
  void play(const audio_data& input_data) override
  {
    play_internal(input_data, true);
  }

  void enqueuePlayback(const audio_data& input_data) override
  {
    play_internal(input_data, false);
  }

  void stopPlayback() override
  {
    std::lock_guard<std::mutex> lock(playback_mutex_);
    playback_queue_.clear();
    playback_cv_.notify_all();
  }

  void play_internal(const audio_data& input_data, bool wait_for_drain)
  {
    auto data = input_data;

    if (data.samples.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Received empty audio data, nothing to set.");
      throw perception_exception("Received empty audio data, nothing to set.");
    }

    // Normalize incoming audio to the configured output sample rate.
    // PortAudio/ALSA will fail to open for unsupported rates (e.g. 192kHz on some sinks).
    // Channel count normalization is handled in write_data().
    if (data.sample_rate != sample_rate_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Incoming audio sample_rate (%d) differs from configured output sample_rate (%d). Resampling.",
                  data.sample_rate, sample_rate_);
      data.samples = resample_linear_interleaved(data.samples, data.channels, data.sample_rate, sample_rate_);
      data.sample_rate = sample_rate_;

      const auto frames = static_cast<int>(data.samples.size() / std::max(1, data.channels));
      data.chunk_size = std::max(1, frames);
      data.chunk_count = 1;
    }

    // Create a unique stream key based on format, rate, and channels
    std::string stream_key = "int16_" + std::to_string(sample_rate_) + "_" + std::to_string(channels_);

    if (stream_dict_.find(stream_key) == stream_dict_.end())
    {
      PaStreamParameters outputParameters;
      outputParameters.device = device_id_;
      outputParameters.channelCount = channels_;
      outputParameters.sampleFormat = paInt16;
      outputParameters.suggestedLatency = Pa_GetDeviceInfo(device_id_)->defaultLowOutputLatency;
      outputParameters.hostApiSpecificStreamInfo = nullptr;

      RCLCPP_INFO(node_->get_logger(), "Opening speaker stream on %s", describePortAudioDevice(device_id_).c_str());

      // Open a new stream for this format
      PaStream* stream = nullptr;
      err = Pa_OpenStream(&stream, nullptr, &outputParameters, sample_rate_, paFramesPerBufferUnspecified, paClipOff,
              &SpeakerAudioDriver::pa_output_callback, this);

      if (err != paNoError)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open speaker stream: %s", Pa_GetErrorText(err));
        throw perception_exception("Failed to open speaker stream: " + std::string(Pa_GetErrorText(err)));
      }

      err = Pa_StartStream(stream);
      if (err != paNoError)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start speaker stream: %s", Pa_GetErrorText(err));
        throw perception_exception("Failed to start speaker stream: " + std::string(Pa_GetErrorText(err)));
      }

      if (!Pa_IsStreamActive(stream))
      {
        RCLCPP_ERROR(node_->get_logger(), "Stream is not active after starting.");
        throw perception_exception("PortAudio stream failed to activate.");
      }

      // Store the stream in the dictionary
      stream_dict_[stream_key] = stream;
      RCLCPP_INFO(node_->get_logger(), "Opened new speaker stream: %s", stream_key.c_str());
      const PaStreamInfo* stream_info = Pa_GetStreamInfo(stream);
      if (stream_info)
      {
        RCLCPP_INFO(node_->get_logger(), "Speaker stream info: sample_rate=%.0f output_latency=%.4f",
                    stream_info->sampleRate, stream_info->outputLatency);
      }
    }

    // Write data to the stream
    try
    {
      if (wait_for_drain)
      {
        RCLCPP_INFO(node_->get_logger(),
                    "Queueing audio to speaker stream %s and waiting for playback drain.",
                    stream_key.c_str());
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(),
                    "Queueing audio to speaker stream %s without waiting for playback drain.",
                    stream_key.c_str());
      }

      queue_data(data, stream_key, wait_for_drain);

      if (wait_for_drain)
      {
        RCLCPP_INFO(node_->get_logger(), "Speaker playback drained for stream: %s", stream_key.c_str());
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "Audio data queued to stream asynchronously: %s", stream_key.c_str());
      }
    }
    catch (const perception_exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error writing audio data to stream: %s", e.what());
      throw;
    }
  }
  /**
   * @brief Read test/mic_test.wav and play it through the speaker.
   */
  void test() override
  {
    RCLCPP_INFO(node_->get_logger(), "Testing by playing: %s", test_file_path_.c_str());

    auto filepath = check_file(test_file_path_);

    try
    {
      auto audio_data = readWavFile(filepath.string());

      RCLCPP_INFO(node_->get_logger(), "Read audio data from %s with %zu samples, %d Hz, %d channels.",
                  filepath.string().c_str(), audio_data.samples.size(), audio_data.sample_rate, audio_data.channels);

      int max_abs = 0;
      long double sum_squares = 0.0;
      for (const auto sample : audio_data.samples)
      {
        max_abs = std::max(max_abs, std::abs(static_cast<int>(sample)));
        sum_squares += static_cast<long double>(sample) * static_cast<long double>(sample);
      }
      const double rms = audio_data.samples.empty() ? 0.0 :
                                                        std::sqrt(static_cast<double>(sum_squares /
                                                                                      audio_data.samples.size()));
      const double duration = static_cast<double>(audio_data.samples.size()) /
                              static_cast<double>(std::max(1, audio_data.sample_rate) * std::max(1, audio_data.channels));
      RCLCPP_INFO(node_->get_logger(), "Speaker test input stats: max_abs=%d rms=%.2f estimated_duration=%.2fs", max_abs,
                  rms, duration);
      if (max_abs < 128)
      {
        RCLCPP_WARN(node_->get_logger(), "Speaker test input appears nearly silent; use a known-good WAV to test output routing.");
      }

      play(audio_data);
    }
    catch (const perception_exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error during test: %s", e.what());
    }

    RCLCPP_INFO(node_->get_logger(), "Test completed.");
  }

protected:
  static int pa_output_callback(const void* input_buffer, void* output_buffer, unsigned long frames_per_buffer,
                                const PaStreamCallbackTimeInfo* time_info, PaStreamCallbackFlags status_flags,
                                void* user_data)
  {
    (void)input_buffer;
    (void)time_info;
    (void)status_flags;

    auto* driver = static_cast<SpeakerAudioDriver*>(user_data);
    if (!driver)
      return paAbort;

    return driver->fill_output_buffer(output_buffer, frames_per_buffer, status_flags);
  }

  int fill_output_buffer(void* output_buffer, unsigned long frames_per_buffer, PaStreamCallbackFlags status_flags)
  {
    auto* output = static_cast<int16_t*>(output_buffer);
    const size_t samples_needed = static_cast<size_t>(frames_per_buffer) * static_cast<size_t>(std::max(1, channels_));
    size_t copied = 0;

    if (status_flags & paOutputUnderflow)
      underrun_count_++;

    std::unique_lock<std::mutex> lock(playback_mutex_, std::try_to_lock);
    if (lock.owns_lock())
    {
      while (copied < samples_needed && !playback_queue_.empty())
      {
        output[copied++] = playback_queue_.front();
        playback_queue_.pop_front();
      }

      if (playback_queue_.empty())
        playback_cv_.notify_all();
    }
    else
    {
      callback_lock_miss_count_++;
    }

    if (copied < samples_needed)
    {
      std::fill(output + copied, output + samples_needed, static_cast<int16_t>(0));
      underrun_count_++;
    }

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

  void queue_data(audio_data input_data, const std::string& stream_key, bool wait_for_drain)
  {
    (void)stream_key;

    std::vector<int16_t> data;  // Buffer for the actual data to write

    // Handle mono-to-stereo or stereo-to-mono conversions if necessary
    if (input_data.channels != channels_)
    {
      if (input_data.channels == 1 && channels_ == 2)
      {
        // Mono to stereo conversion
        data.resize(input_data.samples.size() * 2);
        for (size_t i = 0; i < input_data.samples.size(); ++i)
        {
          data[2 * i] = input_data.samples[i];
          data[2 * i + 1] = input_data.samples[i];
        }
      }
      else if (input_data.channels == 2 && channels_ == 1)
      {
        // Stereo to mono conversion
        data.resize(input_data.samples.size() / 2);
        for (size_t i = 0; i < data.size(); ++i)
        {
          data[i] = static_cast<int16_t>((input_data.samples[2 * i] + input_data.samples[2 * i + 1]) / 2);
        }
      }
      else
      {
        throw perception_exception("Unsupported channel conversion from " + std::to_string(input_data.channels) +
                                   " to " + std::to_string(channels_));
      }
    }
    else
    {
      // No conversion needed
      data = input_data.samples;
    }

    // Make sure chunk size is correct for frames (not samples)
    const size_t required_samples =
        static_cast<size_t>(std::max(1, input_data.chunk_size)) * static_cast<size_t>(std::max(1, channels_));

    if (data.size() < required_samples)
    {
      throw perception_exception("Insufficient data" + std::to_string(data.size()) + " for requested chunk size " +
                                 std::to_string(required_samples));
    }

    const unsigned long frames_to_queue = static_cast<unsigned long>(data.size() / static_cast<size_t>(channels_));
    RCLCPP_INFO(node_->get_logger(), "Queueing %lu frames (%zu samples) to speaker stream.", frames_to_queue,
                data.size());

    {
      std::lock_guard<std::mutex> lock(playback_mutex_);
      playback_queue_.insert(playback_queue_.end(), data.begin(), data.end());
    }

    playback_cv_.notify_all();

    if (wait_for_drain)
      wait_for_playback_drain();
  }

  void wait_for_playback_drain()
  {
    std::unique_lock<std::mutex> lock(playback_mutex_);
    playback_cv_.wait(lock, [this] { return playback_queue_.empty(); });
  }

  void setup_diagnostics()
  {
    enable_diagnostics("portaudio-speaker-" + std::to_string(device_id_), name_ + " playback",
                       [this](diagnostic_updater::DiagnosticStatusWrapper& status) { produce_diagnostics(status); });
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    const auto underrun_count = underrun_count_.load();
    const auto lock_miss_count = callback_lock_miss_count_.load();

    size_t queued_samples = 0;
    {
      std::lock_guard<std::mutex> lock(playback_mutex_);
      queued_samples = playback_queue_.size();
    }

    const size_t queued_frames = queued_samples / static_cast<size_t>(std::max(1, channels_));
    bool stream_active = false;
    for (const auto& pair : stream_dict_)
    {
      if (pair.second && Pa_IsStreamActive(pair.second) == 1)
      {
        stream_active = true;
        break;
      }
    }

    if (!stream_active && queued_samples > 0)
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Speaker queue has data but no active stream");
    else if (underrun_count > 0 || lock_miss_count > 0)
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Speaker callback underruns observed");
    else if (stream_active)
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Speaker playback healthy");
    else
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Speaker idle");

    status.add("device_id", device_id_);
    status.add("device_name", device_name_);
    status.add("sample_rate", sample_rate_);
    status.add("channels", channels_);
    status.add("queued_samples", queued_samples);
    status.add("queued_frames", queued_frames);
    status.add("queue_drained", queued_samples == 0 ? "true" : "false");
    status.add("callback_underrun_count", underrun_count);
    status.add("callback_lock_miss_count", lock_miss_count);
  }

  int device_id_;
  std::string device_name_;
  PaError err = paNoError;
  int sample_rate_;  // Default sample rate
  int channels_;     // Default number of channels
  std::string test_file_path_;

  std::map<std::string, PaStream*> stream_dict_;
  std::deque<int16_t> playback_queue_;
  std::mutex playback_mutex_;
  std::condition_variable playback_cv_;
  std::atomic<uint64_t> underrun_count_{ 0 };
  std::atomic<uint64_t> callback_lock_miss_count_{ 0 };
};

}  // namespace perception