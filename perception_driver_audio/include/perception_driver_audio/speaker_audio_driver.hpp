#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <map>
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

    // Load parameters from the node
    config_.name = node->get_parameter("driver.audio.SpeakerAudioDriver.name").as_string();
    config_.device_name = node->get_parameter("driver.audio.SpeakerAudioDriver.device_name").as_string();
    config_.subscribe = node->get_parameter("driver.audio.SpeakerAudioDriver.subscribe").as_bool();
    config_.topic = node->get_parameter("driver.audio.SpeakerAudioDriver.topic").as_string();
    config_.frame_id = node->get_parameter("driver.audio.SpeakerAudioDriver.frame_id").as_string();
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
    event_->info("started.");
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
        event_->error("Failed to stop speaker stream: " + std::string(Pa_GetErrorText(err)));
        throw perception_exception("Failed to stop speaker stream: " + std::string(Pa_GetErrorText(err)));
      }

      err = Pa_CloseStream(pair.second);
      if (err != paNoError)
      {
        event_->error("Failed to close speaker stream: " + std::string(Pa_GetErrorText(err)));
        throw perception_exception("Failed to close speaker stream: " + std::string(Pa_GetErrorText(err)));
      }
      pair.second = nullptr;
    }

    event_->info("stopped.");
  }

  /**
   * @brief Set the latest audio data to the driver. This function sends the latest audio data to the speaker driver.
   *
   * @param input The latest audio data to the driver in the form of `const std::vector<std::vector<int16_t>>&`.
   * @throws perception_exception if not implemented in derived classes
   */
  void setDataStream(const std::any& input) override
  {
    audio_data data;

    try
    {
      // convert std::any to audio_data
      data = std::any_cast<const perception::audio_data&>(input);
    }
    catch (const perception_exception& error)
    {
      throw perception_exception("Invalid audio data passed to SpeakerAudioDriver::setDataStream");
    }

    if (data.samples.empty())
    {
      event_->error("Received empty audio data, nothing to set.");
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
      err = Pa_OpenStream(&stream, nullptr, &outputParameters, data.sample_rate,
                          paFramesPerBufferUnspecified, paClipOff, nullptr, nullptr);

      if (err != paNoError)
      {
        event_->error("Failed to open speaker stream: " + std::string(Pa_GetErrorText(err)));
        throw perception_exception("Failed to open speaker stream: " + std::string(Pa_GetErrorText(err)));
      }

      err = Pa_StartStream(stream);
      if (err != paNoError)
      {
        event_->error("Failed to start speaker stream: " + std::string(Pa_GetErrorText(err)));
        throw perception_exception("Failed to start speaker stream: " + std::string(Pa_GetErrorText(err)));
      }

      if (!Pa_IsStreamActive(stream))
      {
        event_->error("Stream is not active after starting.");
        throw perception_exception("PortAudio stream failed to activate.");
      }

      // Store the stream in the dictionary
      stream_dict_[stream_key] = stream;
      event_->info("Opened new speaker stream: " + stream_key);
    }

    // Write data to the stream
    try
    {
      write_data(data, stream_key);
      event_->info("Audio data written to stream: " + stream_key);
    }
    catch (const perception_exception& e)
    {
      event_->error("Error writing audio data to stream: " + std::string(e.what()));
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

      event_->info("Read audio data from " + filepath.string() + " with " + std::to_string(audio_data.samples.size()) +
                   " samples, " + std::to_string(audio_data.sample_rate) + " Hz, " +
                   std::to_string(audio_data.channels) + " channels.");

      setDataStream(audio_data);
    }
    catch (const perception_exception& e)
    {
      event_->error("Error during test: " + std::string(e.what()));
    }

    event_->info("Test completed.");
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
  int sample_rate_;           // Default sample rate
  int channels_;              // Default number of channels

  // Subscriber for audio data
  rclcpp::Subscription<perception_msgs::msg::PerceptionAudio>::SharedPtr audio_subscriber_;

  std::map<std::string, PaStream*> stream_dict_;
};

}  // namespace perception