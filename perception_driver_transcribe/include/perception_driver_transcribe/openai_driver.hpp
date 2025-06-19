#pragma once

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <perception_base/rest_base.hpp>
#include <perception_base/utils/audio/structs.hpp>
#include <perception_base/utils/audio/wav.hpp>
#include <perception_base/utils/exceptions.hpp>
#include <perception_msgs/srv/perception_transcribe.hpp>

namespace perception
{

class OpenAIDriver : public RestBase
{
public:
  using Transcribe = perception_msgs::srv::PerceptionTranscribe;
  /**
   * @brief Construct a new Prompt Tools Transcribe Driver object
   *
   */
  OpenAIDriver()
  {
  }

  /**
   * @brief Destroy the Prompt Tools Transcribe Driver object
   *
   */
  ~OpenAIDriver() override
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
    node->declare_parameter("driver.transcription.OpenAIDriver.name", "OpenAIDriver");
    node->declare_parameter("driver.transcription.OpenAIDriver.model", "whisper-1");
    node->declare_parameter("driver.transcription.OpenAIDriver.test_file_path", "test/mic.wav");
    node->declare_parameter("driver.transcription.OpenAIDriver.provide_service", false);
    node->declare_parameter("driver.transcription.OpenAIDriver.service_name", "perception/transcription");

    // Get parameters from the node
    config_.name = node->get_parameter("driver.transcription.OpenAIDriver.name").as_string();
    model_name_ = node->get_parameter("driver.transcription.OpenAIDriver.model").as_string();
    test_file_path_ = node->get_parameter("driver.transcription.OpenAIDriver.test_file_path").as_string();
    config_.interface_name = node->get_parameter("driver.transcription.OpenAIDriver.service_name").as_string();
    config_.interface_enabled = node->get_parameter("driver.transcription.OpenAIDriver.provide_service").as_bool();

    // Initialize the REST base class
    initialize_rest_base(node, "driver.transcription.OpenAIDriver", "OPENAI_API_KEY");

    // Log the parameters
    event_->info("Assigned driver Name: " + config_.name);
    event_->info("Assigned driver Model: " + model_name_);
    event_->info("Assigned driver Test Audio Path: " + test_file_path_);

    // Initialize service if enabled
    if (config_.interface_enabled)
    {
      service_ = node->create_service<Transcribe>(
          config_.interface_name,
          std::bind(&OpenAIDriver::service_cb, this, std::placeholders::_1, std::placeholders::_2));

      event_->info("Service " + config_.interface_name + " created for transcription.");
    }
    else
    {
      event_->info("Transcription service not enabled.");
    }

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
    // Log that the client has been created
    event_->info("Started");
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
    return response_.response;
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

    // create perception::RESTRequest object
    perception::RESTRequest request;
    request.file_type = "audio/wav";
    request.file_stream = encodeWavToBytes(new_audio);
    request.options.clear();

    perception::RESTOption model_option1;
    model_option1.key = "model";
    model_option1.value = model_name_;
    model_option1.type = perception::RESTOptionType::STRING;
    request.options.push_back(model_option1);

    perception::RESTOption model_option2;
    model_option2.key = "response_format";
    model_option2.value = "json";
    model_option2.type = perception::RESTOptionType::STRING;
    request.options.push_back(model_option2);

    response_ = call_audio(request);
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
  /**
   * @brief Convert a prompt request to a JSON object
   *
   * This method converts the perception request to a JSON object that can be sent to the perception plugin.
   * It includes the prompt text and options for the JSON object.
   *
   * @param prompt The perception request to convert
   * @return A JSON object representing the prompt request
   */
  virtual nlohmann::json toJson(const perception::RESTRequest& prompt)
  {
    throw perception_exception("toJson() not implemented for this driver.");
  }

  /**
   * @brief Convert a JSON object to a perception response
   *
   * This method converts a JSON object received from the perception plugin into a perception response.
   * It extracts the relevant fields from the JSON object and returns a perception response object.
   *
   * @param object The JSON object to convert
   * @return A perception response object containing the response data
   */
  virtual perception::RESTResponse fromJson(const nlohmann::json& object)
  {
    perception::RESTResponse res;

    // Check if the key "text" exists and is a string
    if (object.contains("text") && object["text"].is_string())
    {
      res.response = object["text"].get<std::string>();
    }

    return res;
  }

  /**
   * @brief Service callback for transcription requests
   *
   * This method handles incoming transcription requests and processes them.
   *
   * @param request The incoming transcription request
   * @param response The response to be sent back
   */
  void service_cb(const std::shared_ptr<Transcribe::Request> request, std::shared_ptr<Transcribe::Response> response)
  {
    event_->info("Received transcription request.");
    const auto& audio_data = perception::msg_to_audio_data(request->audio);

    setDataStream(audio_data);
    auto result = getData();

    if (result.has_value())
    {
      response->transcription = std::any_cast<std::string>(result);
      response->success = true;
      event_->info("Transcription service processed request successfully.");
    }
    else
    {
      response->success = false;
      response->transcription = "No transcription result received.";
      event_->error("Transcription service failed to process request.");
    }
  }

  perception::RESTResponse response_;

  std::string model_name_;
  std::string test_file_path_;
  rclcpp::Service<Transcribe>::SharedPtr service_;
};

}  // namespace perception
