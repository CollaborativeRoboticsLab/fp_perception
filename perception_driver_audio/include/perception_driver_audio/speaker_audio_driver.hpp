#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <map>
#include <fstream>
#include <perception_base/driver_base.hpp>
#include <perception_base/utils/audio/structs.hpp>
#include <perception_base/utils/audio/wav.hpp>
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
    node->declare_parameter("driver.audio.SpeakerAudioDriver.sample_rate", 44100);  // default sample rate
    node->declare_parameter("driver.audio.SpeakerAudioDriver.channels", 1);         // default number of channels
    node->declare_parameter("driver.audio.SpeakerAudioDriver.test_file_path", "test/mic_test.wav");       // default device ID

    // Load parameters from the node
    config_.name = node->get_parameter("driver.audio.SpeakerAudioDriver.name").as_string();
    config_.device_name = node->get_parameter("driver.audio.SpeakerAudioDriver.device_name").as_string();
    config_.interface_enabled = node->get_parameter("driver.audio.SpeakerAudioDriver.subscribe").as_bool();
    config_.interface_name = node->get_parameter("driver.audio.SpeakerAudioDriver.topic").as_string();
    config_.frame_id = node->get_parameter("driver.audio.SpeakerAudioDriver.frame_id").as_string();
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

    // get the device ID by name
    try
    {
      config_.device_id = perception::getDeviceIdByName(config_.device_name);
      RCLCPP_INFO(node_->get_logger(), "Device ID for name '%s' is %d", config_.device_name.c_str(), config_.device_id);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get device ID for name '%s': %s", config_.device_name.c_str(), e.what());
      throw perception_exception("Failed to get device ID for name '" + config_.device_name + "': " + e.what());
    }

    // Publish about the assigned driver parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver name: %s", config_.name.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver device_name: %s", config_.device_name.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver device_id: %d", config_.device_id);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver subscribe: %s", config_.interface_enabled ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), "Assigned driver topic: %s", config_.interface_name.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver frame_id: %s", config_.frame_id.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver sample_rate: %d", sample_rate_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver channels: %d", channels_);

    RCLCPP_INFO(node_->get_logger(), "Initialized");
    // If subscribing to audio data, set up the subscriber
    if (config_.interface_enabled)
    {
      audio_subscriber_ = node->create_subscription<perception_msgs::msg::PerceptionAudio>(
          config_.interface_name, 10, std::bind(&SpeakerAudioDriver::receiveData, this, std::placeholders::_1));
      RCLCPP_INFO(node_->get_logger(), "Audio subscriber created for topic: %s", config_.interface_name.c_str());
    }
  } 

  /**
   * @brief Start the driver streaming with a ROS node. This function initializes the speaker driver
   * and starts the audio stream. It uses PortAudio to open the default audio stream and starts it.
   *
   */
  void start() override
  {
    RCLCPP_INFO(node_->get_logger(), "started.");
  }

  /**
   * @brief Stop driver streaming. This function stops the audio stream and closes it.
   * It also terminates the PortAudio library.
   */
  void stop() override
  {
    for (auto& pair : stream_dict_)
    {
      err = Pa_StopStream(pair.second);
      if (err != paNoError)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to stop speaker stream: %s", Pa_GetErrorText(err));
        throw perception_exception("Failed to stop speaker stream: " + std::string(Pa_GetErrorText(err)));
      }

      err = Pa_CloseStream(pair.second);
      if (err != paNoError)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to close speaker stream: %s", Pa_GetErrorText(err));
        throw perception_exception("Failed to close speaker stream: " + std::string(Pa_GetErrorText(err)));
      }
      pair.second = nullptr;
    }

    RCLCPP_INFO(node_->get_logger(), "stopped.");
  }

  /**
   * @brief Set the latest audio data to the driver. This function sends the latest audio data to the speaker driver.
   *
   * @param input The latest audio data to the driver in the form of `const std::vector<std::vector<int16_t>>&`.
   * @throws perception_exception if not implemented in derived classes
   */
  void setDataStream(const std::any& input) override
  {
    // convert std::any to audio_data
    const auto data = std::any_cast<const perception::audio_data&>(input);

    if (data.samples.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Received empty audio data, nothing to set.");
      throw perception_exception("Received empty audio data, nothing to set.");
    }

    // Create a unique stream key based on format, rate, and channels
    std::string stream_key = "int16_" + std::to_string(data.sample_rate) + "_" + std::to_string(channels_);

    if (stream_dict_.find(stream_key) == stream_dict_.end())
    {
      PaStreamParameters outputParameters;
      outputParameters.device = config_.device_id;
      outputParameters.channelCount = channels_;
      outputParameters.sampleFormat = paInt16;
      outputParameters.suggestedLatency = Pa_GetDeviceInfo(config_.device_id)->defaultLowOutputLatency;
      outputParameters.hostApiSpecificStreamInfo = nullptr;

      // Open a new stream for this format
      PaStream* stream = nullptr;
      err = Pa_OpenStream(&stream, nullptr, &outputParameters, data.sample_rate, paFramesPerBufferUnspecified,
                          paClipOff, nullptr, nullptr);

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
    }

    // Write data to the stream
    try
    {
      write_data(data, stream_key);
      RCLCPP_INFO(node_->get_logger(), "Audio data written to stream: %s", stream_key.c_str());
    }
    catch (const perception_exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error writing audio data to stream: %s", e.what());
      throw;
    }
  }

  /**
   * @brief Set the latest audio data to the driver from topic subscription.
   * This function is called when new audio data is received from the subscribed topic.
   *
   * @param msg The audio data message received from the topic.
   * @throws perception_exception if the stream is not active
   */
  void receiveData(const perception_msgs::msg::PerceptionAudio& msg)
  {
    auto data = perception::msg_to_audio_data(msg);
    setDataStream(data);
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

      setDataStream(audio_data);
    }
    catch (const perception_exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error during test: %s", e.what());
    }

    RCLCPP_INFO(node_->get_logger(), "Test completed.");
  }

protected:
  void write_data(audio_data input_data, const std::string& stream_key)
  {
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
    }
    else
    {
      // No conversion needed
      data = input_data.samples;
    }

    // Make sure chunk size is correct for frames (not samples)
    if (data.size() < input_data.chunk_size * channels_)
    {
      throw perception_exception("Insufficient data" + std::to_string(data.size()) + " for requested chunk size " +
                                 std::to_string(input_data.chunk_size * channels_));
    }

    // Write to PortAudio stream
    PaError err = Pa_WriteStream(stream_dict_[stream_key], data.data(), data.size() / channels_);
    if (err != paNoError && err != paOutputUnderflowed)
    {
      throw perception_exception("PortAudio write error: " + std::string(Pa_GetErrorText(err)));
    }
  }

  PaError err = paNoError;
  std::vector<int16_t> audio_queue_;
  int sample_rate_;  // Default sample rate
  int channels_;     // Default number of channels
  std::string test_file_path_;

  // Subscriber for audio data
  rclcpp::Subscription<perception_msgs::msg::PerceptionAudio>::SharedPtr audio_subscriber_;

  std::map<std::string, PaStream*> stream_dict_;
};

}  // namespace perception