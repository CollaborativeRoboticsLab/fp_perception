#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <perception_base/rest_base.hpp>
#include <perception_base/audio/structs.hpp>
#include <perception_base/audio/wav.hpp>
#include <perception_base/exceptions.hpp>

namespace perception
{

class OpenAIDriver : public RestBase
{
public:
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
    deinitialize();
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

    // Get parameters from the node
    name_ = node->get_parameter("driver.transcription.OpenAIDriver.name").as_string();
    model_name_ = node->get_parameter("driver.transcription.OpenAIDriver.model").as_string();
    test_file_path_ = node->get_parameter("driver.transcription.OpenAIDriver.test_file_path").as_string();

    // Initialize the REST base class
    initialize_rest_base(node, "driver.transcription.OpenAIDriver", "OPENAI_API_KEY");

    // Log the parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Name: %s", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Model: %s", model_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Test Audio Path: %s", test_file_path_.c_str());

    // Log that the driver has been initialized
    RCLCPP_INFO(node_->get_logger(), "Initialized");
  }

  /**
   * @brief Deinitialize the driver
   *
   * This is required by DriverBase. For this driver, deinitialization primarily
   * means releasing references to the ROS node and clearing cached response state.
   */
  void deinitialize() override
  {
    response_ = perception::RESTResponse{};
    model_name_.clear();
    test_file_path_.clear();
    name_.clear();
    node_.reset();
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
    RCLCPP_INFO(node_->get_logger(), "Testing with model: %s by transcribing: %s", model_name_.c_str(),
                test_file_path_.c_str());

    // Check if the test audio file exists
    auto filepath = check_file(test_file_path_);

    // reading test audio file
    auto data = readWavFile(filepath);

    if (data.samples.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to read test audio file.");
      throw perception_exception("Failed to read test audio file.");
    }
    RCLCPP_INFO(node_->get_logger(), "Test audio file read successfully, size: %d", data.samples.size());

    // Convert the audio data to the expected format
    setDataStream(data);

    RCLCPP_INFO(node_->get_logger(), "Transcription service called with test audio data. waiting for response...");

    auto result = getData();

    if (result.has_value())
    {
      RCLCPP_INFO(node_->get_logger(), "Transcription result: %s", std::any_cast<std::string>(result).c_str());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "No transcription result received.");
      throw perception_exception("No transcription result received.");
    }

    RCLCPP_INFO(node_->get_logger(), "Test completed.");
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

  perception::RESTResponse response_;

  std::string model_name_;
  std::string test_file_path_;
};

}  // namespace perception
