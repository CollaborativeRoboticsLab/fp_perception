#pragma once

#include <prompt_msgs/srv/prompt.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace perception
{
/**
 * @brief PromptToolsTranscribeDriver class for handling prompt_tools based transcriptions.
 *
 *
 */
using PromptSrv = prompt_msgs::srv::Prompt;

class PromptToolsTranscribeDriver : public DriverBase
{
public:
  PromptToolsTranscribeDriver() = default;
  ~PromptToolsTranscribeDriver() override = default;

  /**
   * @brief Initialize the driver
   *
   * This function should be overridden in derived classes to provide specific initialization.
   *
   * @param node Shared pointer to the ROS node
   */
  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    // Confirm parameters for the node
    node->declare_parameter("driver.transcription.PromptToolsTranscribeDriver.name", "PromptToolsTranscribeDriver");
    node->declare_parameter("driver.transcription.PromptToolsTranscribeDriver.model", "whisper-1");
    node->declare_parameter("driver.transcription.PromptToolsTranscribeDriver.prompt", "This audio contains human "
                                                                                       "speech.");

    config_.name = node->get_parameter("driver.transcription.PromptToolsTranscribeDriver.name").as_string();
    model_name_ = node->get_parameter("driver.transcription.PromptToolsTranscribeDriver.model").as_string();
    prompt_text_ = node->get_parameter("driver.transcription.PromptToolsTranscribeDriver.prompt").as_string();

    // Log that the driver has been initialized
    event_->info("Initialized");
  }

protected:
  /**
   * @brief Start the driver streaming
   *
   * This function should be overridden in derived classes to provide specific streaming logic.
   */
  void start() override
  {
    // Create a client for the transcription service
    transcribe_client_ = node_->create_client<PromptSrv>("prompt_bridge/transcribe");
    event_->info("Transcribe service client created");
  }

  /**
   * @brief Stop driver streaming
   *
   * This function should be overridden in derived classes to provide specific stop logic.
   */
  void stop() override
  {
    // Implement the logic to stop the transcription service
  }

  /**
   * @brief Get latest data from the driver
   *
   * This function waits for the transcription service to complete and retrieves the latest transcription data.
   *
   * @return std::any The latest transcription data in the form of std::string
   * @throws perception_exception if not implemented in derived classes
   */
  std::any getData() const override
  {
    // wait until the future is ready
    future_.wait();

    // Implement the logic to get the latest transcription data
    try
    {
      if (!future_.valid())
      {
        event_->error("Transcription service call is not valid.");
        throw perception_exception("Transcription service call is not valid.");
      }

      auto response = future.get();
      if (response->response.response)
      {
        event_->info("Transcription successful: " + response->response.response);
      }
    }
    catch (const std::exception& e)
    {
      event_->error("Transcription service call failed: " + std::string(e.what()));
    }

    return response->response.response;
  }
  /**
   * @brief Set data to the driver
   *
   * This function sends the latest audio data to the transcription service. It expects the input to be a vector of
   * audio chunks in the form of std::vector<std::vector<int16_t>>.
   *
   * @param input The latest data from the driver.
   */
  void setDataStream(const std::any& input) const override
  {
    const auto& new_audio = std::any_cast<const std::vector<std::vector<int16_t>>&>(input);

    if (new_audio.size() > 0)
    {
      // If there are chunks to publish, create a message and publish it
      std_msgs::msg::Int16MultiArray msg;

      msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      msg.layout.dim[0].label = "samples";
      msg.layout.dim[0].size = new_audio.size();
      msg.layout.dim[0].stride = new_audio[1].size() * new_audio.size();

      msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      msg.layout.dim[1].label = "chunk";
      msg.layout.dim[1].size = new_audio[0].size();
      msg.layout.dim[1].stride = new_audio[0].size();

      for (const auto& chunk : new_audio)
      {
        msg.data.insert(msg.data.end(), chunk.begin(), chunk.end());
      }

      // Implement the logic to set the transcription data
      PromptSrv::Request::SharedPtr request = std::make_shared<PromptSrv::Request>();
      request->prompt.prompt = prompt_text_;
      request->prompt.flush = true;
      request->prompt.contains_audio = true;
      request->prompt.file_type = "audio/wav";
      request->prompt.audio_buffer = msg;

      request->prompt.options.push_back(
          prompt::PromptOption{ "model", model_name_, prompt::PromptOptionType::STRING_TYPE });
      request->prompt.options.push_back(
          prompt::PromptOption{ "response_format", "json", prompt::PromptOptionType::STRING_TYPE });

      // Call the transcription service
      future_ = transcribe_client_->async_send_request(request);
    }
    else
    {
      event_->warn("No audio data provided to transcribe.");
      throw perception_exception("No audio data provided to transcribe.");
    }
  }

  rclcpp::Client<PromptSrv>::SharedPtr transcribe_client_;
  rclcpp::Client<PromptSrv>::SharedFuture future_;

  std::string model_name_;
  std::string prompt_text_ = "This audio contains human speech.";
}

}  // namespace perception
