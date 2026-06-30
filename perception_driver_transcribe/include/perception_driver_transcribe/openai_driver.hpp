#pragma once

#include <perception_base/audio/wav.hpp>
#include <perception_base/exceptions.hpp>
#include <perception_base/rest_base.hpp>
#include <perception_base/transcription/structs.hpp>
#include <perception_base/transcription/transcription_driver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace perception
{

class OpenAIDriver : public RestBase, public TranscriptionDriver
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

    if (diagnostics_enabled())
    {
      enable_diagnostics("rest-transcription-" + model_name_, name_ + " status",
                         [this](diagnostic_updater::DiagnosticStatusWrapper& status) {
                           produce_diagnostics(status);
                         });
    }

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
    disable_diagnostics();
    response_ = perception::RESTResponse{};
    model_name_.clear();
    test_file_path_.clear();
    name_.clear();
    node_.reset();
  }

  /**
   * @brief Set data to the driver
   *
   * This function sends the latest audio data to the transcription service. It expects the input to be a vector of
   * audio chunks in the form of perception::audio_data.
   *
   * @param input The latest data from the driver.
   */
  transcription_result transcribe(const transcription_request& request_data) override
  {
    const auto& new_audio = request_data.audio;
    const int sample_rate = std::max(1, new_audio.sample_rate);
    const int channels = std::max(1, new_audio.channels);
    const double audio_seconds =
      static_cast<double>(new_audio.samples.size()) / static_cast<double>(sample_rate * channels);

    RCLCPP_INFO(node_->get_logger(),
          "Starting OpenAI transcription request: samples=%zu sample_rate=%d channels=%d duration=%.3f seconds",
          new_audio.samples.size(), new_audio.sample_rate, new_audio.channels, audio_seconds);

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

    RCLCPP_INFO(node_->get_logger(),
          "Completed OpenAI transcription request: response_chars=%zu",
          response_.response.size());

    transcription_result result;
    result.text = response_.response;
    result.success = !response_.response.empty();
    if (!result.success)
      result.error = "No transcription result received.";

    last_result_ = result;
    return last_result_;
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
    RCLCPP_INFO(node_->get_logger(), "Test audio file read successfully, size: %zu", data.samples.size());

    // Convert the audio data to the expected format
    transcription_request request;
    request.audio = data;

    const auto transcription = transcribe(request);

    RCLCPP_INFO(node_->get_logger(), "Transcription service called with test audio data. waiting for response...");

    if (transcription.success)
    {
      RCLCPP_INFO(node_->get_logger(), "Transcription result: %s", transcription.text.c_str());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "%s", transcription.error.c_str());
      throw perception_exception(transcription.error.empty() ? "No transcription result received." :
                                                               transcription.error);
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
  virtual nlohmann::json toJson(const perception::RESTRequest& request)
  {
    (void)request;
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

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    std::string last_error;
    {
      std::lock_guard<std::mutex> lock(rest_status_mutex_);
      last_error = last_rest_error_;
    }

    if (rest_request_count_.load() == 0)
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Transcription driver idle");
    else if (last_rest_success_.load())
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Last transcription request succeeded");
    else
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Last transcription request failed");

    status.add("model", model_name_);
    status.add("uri", uri_);
    status.add("request_count", rest_request_count_.load());
    status.add("failure_count", rest_failure_count_.load());
    status.add("last_http_code", last_rest_http_code_.load());
    status.add("last_result_success", last_result_.success ? "true" : "false");
    status.add("last_transcription_length", last_result_.text.size());
    status.add("last_error", last_error.empty() ? std::string("none") : last_error);
  }

  perception::RESTResponse response_;
  transcription_result last_result_;

  std::string model_name_;
  std::string test_file_path_;
};

}  // namespace perception
