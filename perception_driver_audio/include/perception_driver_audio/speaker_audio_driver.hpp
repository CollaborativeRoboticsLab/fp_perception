#ifndef SPEAKER_DRIVER_HPP_
#define SPEAKER_DRIVER_HPP_

#include <portaudio.h>
#include <vector>
#include <mutex>
#include <perception/driver_base.hpp>

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
  SpeakerAudioDriver() : stream_(nullptr)
  {
  }
  ~SpeakerAudioDriver() override
  {
    stop();
    Pa_Terminate();
  }

  /**
   * @brief Start the driver streaming with a ROS node and namespace. This function
   * initializes the speaker driver and starts the audio stream. It uses PortAudio
   * to open the default audio stream and starts it.
   *
   * @param node Shared pointer to the ROS node
   * @param config configuration loaded from the yaml file
   */
  void start(const rclcpp::Node::SharedPtr& node, const driver_options& config) override
  {
    initialize_base(node, config);

    Pa_Initialize();

    PaError err = Pa_OpenDefaultStream(&stream_, 0, 1, paInt16, 44100, 256, nullptr, nullptr);
    if (err != paNoError)
    {
      throw perception_exception("Failed to open speaker stream: " + std::string(Pa_GetErrorText(err)));
    }

    Pa_StartStream(stream_);

    event_->info("SpeakerAudioDriver started.");
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
      event_->info("SpeakerAudioDriver stopped.");
    }
  }

  /**
   * @brief Set the latest data to the driver.
   *
   * This function sends the latest audio data to the speaker driver.
   *
   * @param input The latest audio data to the driver.
   * @throws perception_exception if not implemented in derived classes
   */
  void setData(std::any& input) const override
  {
    if (!stream_ || !Pa_IsStreamActive(stream_))
      throw perception_exception("Speaker stream not active");

    try
    {
      const auto& buffer = std::any_cast<const std::vector<int16_t>&>(input);
      Pa_WriteStream(stream_, buffer.data(), buffer.size());
    }
    catch (const std::bad_any_cast&)
    {
      throw perception_exception("Invalid audio data passed to SpeakerDriver::setData");
    }
  }

protected:
  PaStream* stream_;
};

}  // namespace perception

#endif  // SPEAKER_DRIVER_HPP_
