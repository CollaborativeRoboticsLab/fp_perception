#ifndef MICROPHONE_DRIVER_HPP_
#define MICROPHONE_DRIVER_HPP_

#include <portaudio.h>
#include <vector>
#include <mutex>
#include <perception/driver_base.hpp>

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
   * @brief Start the driver streaming with a ROS node and namespace. This function
   * initializes the microphone driver and starts the audio stream. It uses PortAudio
   * to open the default audio stream and starts it.
   *
   * @param node Shared pointer to the ROS node
   * @param config configuration loaded from the yaml file
   */
  void start(const rclcpp::Node::SharedPtr& node, const driver_options& config) override
  {
    initialize_base(node, config);

    Pa_Initialize();

    PaError err = Pa_OpenDefaultStream(&stream_, 1, 0, paInt16, 44100, 256, nullptr, nullptr);
    if (err != paNoError) {
      throw perception_exception("Failed to open microphone stream: " + std::string(Pa_GetErrorText(err)));
    }
    
    Pa_StartStream(stream_);

    event_->info("MicrophoneAudioDriver started.");
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
  }

  /**
   * @brief Get latest audio data from the driver. This function reads audio data from
   * the microphone stream and returns it as a vector of int16_t.
   *
   * @return std::any The latest audio data from the driver.
   * @throws perception_exception if the stream is not active
   */
  std::any getData() const override
  {
    if (!stream_ || !Pa_IsStreamActive(stream_))
      throw perception_exception("Microphone stream not active");

    std::vector<int16_t> buffer(256);
    Pa_ReadStream(stream_, buffer.data(), buffer.size());
    return buffer;
  }

protected:
  PaStream* stream_;
};

}  // namespace perception

#endif  // MICROPHONE_DRIVER_HPP_
