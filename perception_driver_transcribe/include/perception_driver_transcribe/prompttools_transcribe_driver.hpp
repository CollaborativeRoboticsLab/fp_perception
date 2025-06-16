#pragma once

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <prompt_msgs/msg/model_option.hpp>
#include <prompt_msgs/srv/prompt.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_base/utils/audio/structs.hpp>
#include <perception_base/utils/audio/wav.hpp>
#include <perception_base/utils/exceptions.hpp>

namespace perception
{
/**
 * @brief PromptToolsTranscribeDriver class for handling prompt_tools based transcriptions.
 *
 *  This class is responsible for managing the transcription of audio data using the PromptTools service.
 *  It provides methods to initialize the driver, start and stop the transcription service, and retrieve the latest
 *  transcription data.
 */
using PromptSrv = prompt_msgs::srv::Prompt;

class PromptToolsTranscribeDriver : public DriverBase
{
public:
  /**
   * @brief Construct a new Prompt Tools Transcribe Driver object
   *
   */
  PromptToolsTranscribeDriver()
  {
  }

  /**
   * @brief Destroy the Prompt Tools Transcribe Driver object
   *
   */
  ~PromptToolsTranscribeDriver() override
  {
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
    // Confirm parameters for the node
    node->declare_parameter("driver.transcription.PromptToolsTranscribeDriver.name", "PromptToolsTranscribeDriver");
    node->declare_parameter("driver.transcription.PromptToolsTranscribeDriver.model", "whisper-1");
    node->declare_parameter("driver.transcription.PromptToolsTranscribeDriver.prompt", "This audio contains human "
                                                                                       "speech.");
    node->declare_parameter("driver.transcription.PromptToolsTranscribeDriver.service_name", "prompt_bridge/transcribe");
    node->declare_parameter("driver.transcription.PromptToolsTranscribeDriver.test_file_path", "test/mic.wav");
                                                                                       
    // Get parameters from the node
    config_.name = node->get_parameter("driver.transcription.PromptToolsTranscribeDriver.name").as_string();
    model_name_ = node->get_parameter("driver.transcription.PromptToolsTranscribeDriver.model").as_string();
    prompt_text_ = node->get_parameter("driver.transcription.PromptToolsTranscribeDriver.prompt").as_string();
    service_name_ = node->get_parameter("driver.transcription.PromptToolsTranscribeDriver.service_name").as_string();
    test_file_path_ = node->get_parameter("driver.transcription.PromptToolsTranscribeDriver.test_file_path").as_string();

    // Initialize the base driver
    initialize_base(node);

    // Log the parameters
    event_->info("Assigned driver Name: " + config_.name);
    event_->info("Assigned driver Model: " + model_name_);
    event_->info("Assigned driver Service Name: " + service_name_);
    event_->info("Assigned driver Prompt: " + prompt_text_);
    event_->info("Assigned driver Test Audio Path: " + test_file_path_);

    // Log that the driver has been initialized
    event_->info("Initialized");
  }

  /**
   * @brief Start the driver streaming
   *
   * This function should be overridden in derived classes to provide specific streaming logic.
   */
  void start() override
  {
    // Create a client for the transcription service
    transcribe_client_ = node_->create_client<PromptSrv>(service_name_);

    // Wait for the service to be available
    if(!transcribe_client_->wait_for_service(std::chrono::seconds(10)))
    {
      event_->error("Transcribe service not available. Please check if the service is running.");
      throw perception_exception("Transcribe service not available.");
    }

    // Log that the client has been created
    event_->info("Transcribe service client created and connected.");
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
  std::any getData() override
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

      auto response = future_.get();

      event_->info("Transcription successful: " + response->response.response);
      return response->response.response;
    }
    catch (const std::exception& e)
    {
      event_->error("Transcription service call failed: " + std::string(e.what()));
      throw perception_exception("Transcription service call failed: " + std::string(e.what()));
    }
  }

  /**
   * @brief Set data to the driver
   *
   * This function sends the latest audio data to the transcription service. It expects the input to be a vector of
   * audio chunks in the form of perception::audio_data.
   *
   * @param input The latest data from the driver.
   */
  void setDataStream(const std::any& input) override
  {
    const auto& new_audio = std::any_cast<const perception::audio_data&>(input);

    if (new_audio.samples.size() > 0)
    {
      // Implement the logic to set the transcription data
      PromptSrv::Request::SharedPtr request = std::make_shared<PromptSrv::Request>();
      request->prompt.prompt = prompt_text_;
      request->prompt.flush = true;
      request->prompt.contains_audio = true;
      request->prompt.file_type = "audio/wav";
      request->prompt.samples = new_audio.samples;
      request->prompt.sample_rate = new_audio.sample_rate;
      request->prompt.channels = new_audio.channels;
      request->prompt.chunk_size = new_audio.chunk_size;
      request->prompt.chunk_count = new_audio.chunk_count;

      prompt_msgs::msg::ModelOption model_option1;
      model_option1.key = "model";
      model_option1.value = model_name_;
      model_option1.type = prompt_msgs::msg::ModelOption::STRING_TYPE;
      request->prompt.options.push_back(model_option1);

      prompt_msgs::msg::ModelOption model_option2;
      model_option2.key = "response_format";
      model_option2.value = "json";
      model_option2.type = prompt_msgs::msg::ModelOption::STRING_TYPE;
      request->prompt.options.push_back(model_option2);

      // Call the transcription service
      future_ = transcribe_client_->async_send_request(request);
    }
    else
    {
      event_->error("No audio data provided to transcribe.");
      throw perception_exception("No audio data provided to transcribe.");
    }
  }

  /**
   * @brief Test method for the driver
   *
   * This function can be overridden in derived classes to implement specific test logic.
   */
  void test() override
  {
    // Implement test logic if needed
    event_->info("Testing with model: " + model_name_);
    event_->info("Testing by transcribing : " + test_file_path_);

    // Check if the test audio file exists
    auto filepath = check_file(test_file_path_);

    // reading test audio file
    auto data = readWavFile(filepath);

    if (data.samples.empty())
    {
      event_->error("Failed to read test audio file.");
      throw perception_exception("Failed to read test audio file.");
    }
    event_->info("Test audio file read successfully, size: " + std::to_string(data.samples.size()));

    // Convert the audio data to the expected format
    setDataStream(data);

    event_->info("Transcription service called with test audio data. waiting for response...");

    auto result = getData();

    if (result.has_value())
    {
      event_->info("Transcription result: " + std::any_cast<std::string>(result));
    }
    else
    {
      event_->error("No transcription result received.");
      throw perception_exception("No transcription result received.");
    }

    event_->info("Test completed.");
  }

protected:
  rclcpp::Client<PromptSrv>::SharedPtr transcribe_client_;
  rclcpp::Client<PromptSrv>::SharedFuture future_;

  std::string model_name_;
  std::string prompt_text_;
  std::string service_name_;
  std::string test_file_path_;
};

}  // namespace perception
