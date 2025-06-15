
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include <audio_driver/audio.hpp>
#include <audio_driver/utils.hpp>

#include <perception_events/event_client.hpp>

namespace perception
{
class AudioDriver : public rclcpp::Node
{
public:
  AudioDriver(/* args */) : Node("audio_driver_node")
  {
    // Declare parameters
    this->declare_parameter("AudioDriver.input.device_name", "default");
    this->declare_parameter("AudioDriver.input.topic", "audio/microphone");
    this->declare_parameter("AudioDriver.input.frame_id", "microphone_frame");
    this->declare_parameter("AudioDriver.input.chunk_size", 256);     // default chunk size
    this->declare_parameter("AudioDriver.input.sample_rate", 44100);  // default sample rate
    this->declare_parameter("AudioDriver.input.channels", 1);         // default number of channels

    // Load parameters from the node
    input_device_name = this->get_parameter("AudioDriver.input.device_name").as_string();
    input_device_topic = this->get_parameter("AudioDriver.input.topic").as_string();
    input_frame_id = this->get_parameter("AudioDriver.input.frame_id").as_string();
    input_chunk_size_ = this->get_parameter("AudioDriver.input.chunk_size").as_int();
    input_sample_rate_ = this->get_parameter("AudioDriver.input.sample_rate").as_int();
    input_channels_ = this->get_parameter("AudioDriver.input.channels").as_int();

    event_ = std::make_shared<EventClient>(this, "Audio Driver", "/events");

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
      input_device_id = perception::getDeviceIdByName(input_device_name);
      event_->info("Device ID for name '" + input_device_name + "' is " + std::to_string(input_device_id));
    }
    catch (const std::exception& e)
    {
      event_->error("Failed to get device ID for name '" + input_device_name + "': " + e.what());
      throw perception_exception("Failed to get device ID for name '" + input_device_name + "': " + e.what());
    }

    event_->info("Assigned driver device_name: " + input_device_name);
    event_->info("Assigned driver device_id: " + std::to_string(input_device_id));
    event_->info("Assigned driver topic: " + input_device_topic);
    event_->info("Assigned driver frame_id: " + input_frame_id);
    event_->info("Assigned driver chunk_size: " + std::to_string(input_chunk_size_));
    event_->info("Assigned driver sample_rate: " + std::to_string(input_sample_rate_));
    event_->info("Assigned driver channels: " + std::to_string(input_channels_));

    audio_publisher_ = this->create_publisher<perception_msgs::msg::PerceptionAudio>(input_device_topic, 10);

    inputParams.device = input_device_id;
    inputParams.channelCount = input_channels_;
    inputParams.sampleFormat = paInt16;
    inputParams.suggestedLatency = Pa_GetDeviceInfo(input_device_id)->defaultLowInputLatency;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&input_stream_, &inputParams, nullptr, input_sample_rate_, input_chunk_size_, paNoFlag, nullptr,
                        nullptr);
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
  }

  ~AudioDriver()
  {
    if (stream_)
    {
      Pa_StopStream(stream_);
      Pa_CloseStream(stream_);
      stream_ = nullptr;
      event_->info("stopped.");
    }

    err = Pa_Terminate();
    if (err != paNoError)
    {
      event_->error("PortAudio termination failed: " + std::string(Pa_GetErrorText(err)));
      throw perception_exception("PortAudio termination failed: " + std::string(Pa_GetErrorText(err)));
    }
  }

private:
  PaStream* input_stream_;
  PaError err = paNoError;
  PaStreamParameters inputParams;

  std::string input_device_name;
  std::string input_device_topic;
  std::string input_frame_id;
  int input_device_id;     // default device ID, will be set later
  int input_chunk_size_;   // default chunk size
  int input_sample_rate_;  // default sample rate
  int input_channels_;     // default number of channels

  PaStream* stream_ = nullptr;
  PaError err = paNoError;

  rclcpp::Publisher<perception_msgs::msg::PerceptionAudio>::SharedPtr audio_publisher_;

  /**
   * @brief client for publishing events
   */
  std::shared_ptr<EventClient> event_;
};

}  // namespace perception
