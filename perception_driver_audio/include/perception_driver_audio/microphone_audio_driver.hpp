#pragma once

#include <portaudio.h>
#include <vector>
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <perception_base/driver_base.hpp>

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

    // Log that the driver has been initialized
    event_->info("Initialized");

    // If publishing is enabled, create a publisher for the audio topic
    if (config_.publish)
    {
      audio_publisher_ = node->create_publisher<std_msgs::msg::Int16MultiArray>(config_.topic, 10);
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
   * @brief Get latest audio data from the driver. This function reads audio data from
   * the microphone stream and returns it as a std::vector<int16_t>.
   *
   * @return std::any The latest audio data from the driver.
   * @throws perception_exception if the stream is not active
   */
  std::any getData() const override
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // Check if the audio buffer is empty
    if (audio_buffer_.empty())
      throw perception_exception("No audio data available");

    // check if the audio buffer is smaller than the chunk size
    if (audio_buffer_.size() < chunk_size_)
      throw perception_exception("Audio buffer does not contain enough data for a full chunk");

    std::vector<int16_t> chunk;

    lock_guard<std::mutex> publish_lock(buffer_mutex_);

    // Pop the first chunk_size_ elements from the audio buffer
    for (size_t i = 0; i < chunk_size_; ++i)
    {
      chunk.push_back(audio_buffer_.front());
      audio_buffer_.pop_front();
    }

    return chunk;
  }

  /**
   * @brief Get latest audio data from the driver as a stream. This function reads
   * audio data from the microphone stream and returns it as a std::vector<std::vector<int16_t>>.
   *
   * @return std::any The latest audio data from the driver.
   * @throws perception_exception if the stream is not active
   */
  std::any getDataStream() const override
  {
    std::vector<std::vector<int16_t>> chunks;

    try
    {
      while (audio_buffer_.size() >= chunk_size_)
      {
        std::vector<int16_t> chunk = std::any_cast<std::vector<int16_t>>(getData());
        chunks.push_back(chunk);
      }
    }
    catch (const perception::perception_exception& error)
    {
      throw error;  // Re-throw the exception if no more data is available
    }

    return chunks;
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
      std::vector<std::vector<int16_t>> chunks;

      lock_guard<std::mutex> publish_lock(publish_mutex_);

      while (publish_buffer_.size() >= chunk_size_)
      {
        std::vector<int16_t> chunk;

        // Pop the first chunk_size_ elements from the publish buffer
        for (size_t i = 0; i < chunk_size_; ++i)
        {
          chunk.push_back(publish_buffer_.front());
          publish_buffer_.pop_front();
        }

        chunks.push_back(chunk);
      }

      if (chunks.size() > 0)
      {
        // If there are chunks to publish, create a message and publish it
        std_msgs::msg::Int16MultiArray msg;

        msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        msg.layout.dim[0].label = "samples";
        msg.layout.dim[0].size = chunks.size();
        msg.layout.dim[0].stride = chunk_size_ * chunks.size();
        msg.layout.dim[1].label = "chunk";
        msg.layout.dim[1].size = chunk_size_;
        msg.layout.dim[1].stride = chunk_size_;

        for (const auto& chunk : chunks)
        {
          msg.data.insert(msg.data.end(), chunk.begin(), chunk.end());
        }

        audio_publisher_->publish(msg);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 / config_.frequency)));
    }

    event_->info("MicrophoneAudioDriver thread stopped.");
  }

protected:
  static int paCallback(const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData)
  {
    auto* self = static_cast<MicrophoneAudioDriver*>(userData);
    const auto* in = static_cast<const int16_t*>(inputBuffer);

    if (in)
    {
      std::lock_guard<std::mutex> lock(self->buffer_mutex_);
      self->audio_buffer_.insert(self->audio_buffer_.end(), in, in + framesPerBuffer);

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
  mutable std::mutex buffer_mutex_;
  mutable std::mutex publish_mutex_;
  unsigned long chunk_size_;  // Default chunk size
  int sample_rate_;           // Default sample rate
  int channels_;              // Default number of channels

  // Publisher for audio data
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr audio_publisher_;
};

}  // namespace perception
