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
class OpenAISpeechDriver : public RestBase
{
public:
  /**
   * @brief Construct a new OpenAI Speech Driver object
   */
  OpenAISpeechDriver()
  {
  }

  /**
   * @brief Destroy the OpenAI Speech Driver object
   */
  ~OpenAISpeechDriver() override
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
    node->declare_parameter("driver.speech.OpenAISpeechDriver.name", "OpenAISpeechDriver");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.model", "gpt-4o-mini-tts");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.test_text", "Hello this is a test speech for ROS2 "
                                                                          "perception speech driver.");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.test_file_path", "test/speech.wav");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.voice", "coral");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.instructions", "Please speak clearly and slowly.");

    // Get parameters from the node
    name_ = node->get_parameter("driver.speech.OpenAISpeechDriver.name").as_string();
    model_name_ = node->get_parameter("driver.speech.OpenAISpeechDriver.model").as_string();
    test_text_ = node->get_parameter("driver.speech.OpenAISpeechDriver.test_text").as_string();
    test_file_path_ = node->get_parameter("driver.speech.OpenAISpeechDriver.test_file_path").as_string();
    voice_ = node->get_parameter("driver.speech.OpenAISpeechDriver.voice").as_string();
    instructions_ = node->get_parameter("driver.speech.OpenAISpeechDriver.instructions").as_string();

    // Initialize the REST base class
    initialize_rest_base(node, "driver.speech.OpenAISpeechDriver", "OPENAI_API_KEY");

    // log the parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver name: %s", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver model: %s", model_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver test text: %s", test_text_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver test file path: %s", test_file_path_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver voice: %s", voice_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver instructions: %s", instructions_.c_str());

    // Log the driver initialization
    RCLCPP_INFO(node_->get_logger(), "Initialized");
  }

  /**
   * @brief Deinitialize the driver
   *
   * Required by DriverBase. This driver holds no long-running resources beyond
   * cached state and the ROS node pointer.
   */
  void deinitialize() override
  {
    response_ = perception::RESTResponse{};
    model_name_.clear();
    test_text_.clear();
    voice_.clear();
    instructions_.clear();
    test_file_path_.clear();
    name_.clear();
    node_.reset();
  }

  /**
   * @brief Get latest data from the driver
   *
   * This function should be overridden in derived classes to provide specific data retrieval logic.
   *
   * @return std::any containing the audio data
   */
  std::any getData() override
  {
    audio_data data;

    if (!response_.audio_stream.empty())
    {
      data.samples = std::any_cast<std::vector<int16_t>>(response_.audio_stream);
      data.sample_rate = 24000;  // Assuming a sample rate of 24kHz
      data.channels = 1;         // Assuming mono audio
    }

    return data;
  }

  void setDataStream(const std::any& input) override
  {
    const auto& new_text = std::any_cast<const perception::text_data&>(input);

    // create perception::RESTRequest object
    perception::RESTRequest request;
    request.prompt = new_text.text;
    request.options.clear();

    perception::RESTOption model_option1;
    model_option1.key = "model";
    model_option1.value = model_name_;
    model_option1.type = perception::RESTOptionType::STRING;
    request.options.push_back(model_option1);

    perception::RESTOption model_option2;
    model_option2.key = "response_format";
    model_option2.value = "pcm";
    model_option2.type = perception::RESTOptionType::STRING;
    request.options.push_back(model_option2);

    perception::RESTOption model_option3;
    model_option3.key = "voice";
    if (new_text.voice.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Voice is empty, using default voice.");
      model_option3.value = voice_;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Using voice: %s", new_text.voice.c_str());
      model_option3.value = new_text.voice;
    }
    model_option3.type = perception::RESTOptionType::STRING;
    request.options.push_back(model_option3);

    perception::RESTOption model_option4;
    model_option4.key = "instructions";
    if (new_text.instructions.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Instructions are empty, using default instructions.");
      model_option4.value = instructions_;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Using instructions: %s", new_text.instructions.c_str());
      model_option4.value = new_text.instructions;
    }
    model_option4.type = perception::RESTOptionType::STRING;
    request.options.push_back(model_option4);

    response_ = call_tts(request);
  }

  /**
   * @brief Test method for the driver
   *
   * This function can be overridden in derived classes to implement specific test logic.
   */
  void test() override
  {
    // Implement test logic if needed
    RCLCPP_INFO(node_->get_logger(), "Testing with model: %s", model_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Testing by speeching : %s", test_text_.c_str());

    perception::text_data new_text;
    new_text.text = test_text_;
    new_text.voice = voice_;
    new_text.instructions = instructions_;

    // Convert the audio data to the expected format
    setDataStream(new_text);

    RCLCPP_INFO(node_->get_logger(), "Speech service called with test text data. waiting for response...");

    auto result = getData();

    if (result.has_value())
    {
      audio_data data = std::any_cast<audio_data>(result);
      writeWavFile(test_file_path_, data);
      RCLCPP_INFO(node_->get_logger(), "Speech synthesis result received and saved to %s", test_file_path_.c_str());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "No speech synthesis result received.");
      throw perception_exception("No speech synthesis result received.");
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
    nlohmann::json result;

    // Add options
    for (const auto& option : request.options)
    {
      result[option.key] = option.value;
    }

    // Add prompt
    result["input"] = request.prompt;

    return result;
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
    throw perception_exception("fromJson() not implemented for this driver.");
  }

  std::string model_name_;
  std::string test_text_;
  std::string voice_;
  std::string instructions_;
  std::string test_file_path_ = "test_audio.wav";  // Path to the test audio file

  perception::RESTResponse response_;
};
}  // namespace perception