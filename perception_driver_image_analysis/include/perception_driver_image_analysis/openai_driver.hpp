#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <perception_base/rest_base.hpp>
#include <perception_base/exceptions.hpp>

namespace perception
{

class OpenAIImageAnalysisDriver : public RestBase
{
public:
  /**
   * @brief Construct a new OpenAI Image Analysis Driver object
   */
  OpenAIImageAnalysisDriver()
  {
  }

  /**
   * @brief Destroy the Prompt Tools Transcribe Driver object
   *
   */
  ~OpenAIImageAnalysisDriver() override
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
    node->declare_parameter("driver.image_analysis.OpenAIImageAnalysisDriver.name", "OpenAIDriver");
    node->declare_parameter("driver.image_analysis.OpenAIImageAnalysisDriver.model", "gpt-4.1");
    node->declare_parameter("driver.image_analysis.OpenAIImageAnalysisDriver.test_file_path", "test/image.png");
    node->declare_parameter("driver.image_analysis.OpenAIImageAnalysisDriver.test_prompt", "Is there a cat in this image?");
    node->declare_parameter("driver.image_analysis.OpenAIImageAnalysisDriver.detail", "auto");

    // Get parameters from the node
    name_ = node->get_parameter("driver.image_analysis.OpenAIImageAnalysisDriver.name").as_string();
    model_name_ = node->get_parameter("driver.image_analysis.OpenAIImageAnalysisDriver.model").as_string();
    test_file_path_ = node->get_parameter("driver.image_analysis.OpenAIImageAnalysisDriver.test_file_path").as_string();
    test_prompt_ = node->get_parameter("driver.image_analysis.OpenAIImageAnalysisDriver.test_prompt").as_string();
    detail_ = node->get_parameter("driver.image_analysis.OpenAIImageAnalysisDriver.detail").as_string();

    // Initialize the REST base class
    initialize_rest_base(node, "driver.image_analysis.OpenAIImageAnalysisDriver", "OPENAI_API_KEY");

    // Log the parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Name: %s", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Model: %s", model_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Test Image Path: %s", test_file_path_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Test Prompt: %s", test_prompt_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Image Detail: %s", detail_.c_str());

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
    detail_.clear();
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
    const auto& payload = std::any_cast<const std::pair<cv::Mat, std::string>&>(input);

    const cv::Mat& frame = payload.first;
    const std::string& prompt = payload.second;

    if (frame.empty())
      throw perception_exception("OpenAIImageAnalysisDriver received an empty image frame");

    std::vector<uchar> png_bytes;
    if (!cv::imencode(".png", frame, png_bytes))
      throw perception_exception("OpenAIImageAnalysisDriver failed to encode image to PNG");

    perception::RESTRequest request;
    request.prompt = prompt.empty() ? std::string("What's in this image?") : prompt;
    request.file_type = "image/png";
    request.file_stream.assign(reinterpret_cast<const char*>(png_bytes.data()),
                               reinterpret_cast<const char*>(png_bytes.data()) + png_bytes.size());

    response_ = call(request);
  }

  /**
   * @brief Test method for the driver
   *
   * This function can be overridden in derived classes to implement specific test logic.
   */
  void test() override
  {
    RCLCPP_INFO(node_->get_logger(), "Testing image analysis with model: %s", model_name_.c_str());

    auto filepath = check_file(test_file_path_);

    cv::Mat image = cv::imread(filepath.string(), cv::IMREAD_COLOR);
    if (image.empty())
      throw perception_exception("Failed to read test image file: " + filepath.string());

    setDataStream(std::make_pair(image, test_prompt_));

    auto result = getData();
    if (result.has_value())
    {
      RCLCPP_INFO(node_->get_logger(), "Image analysis result: %s", std::any_cast<std::string>(result).c_str());
    }
    else
    {
      throw perception_exception("No image analysis result received.");
    }
  }

protected:
  static std::string base64_encode(const unsigned char* data, size_t len)
  {
    static const char* kTable =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);

    for (size_t i = 0; i < len; i += 3)
    {
      const unsigned int octet_a = i < len ? data[i] : 0;
      const unsigned int octet_b = (i + 1) < len ? data[i + 1] : 0;
      const unsigned int octet_c = (i + 2) < len ? data[i + 2] : 0;

      const unsigned int triple = (octet_a << 16) | (octet_b << 8) | octet_c;

      out.push_back(kTable[(triple >> 18) & 0x3F]);
      out.push_back(kTable[(triple >> 12) & 0x3F]);
      out.push_back((i + 1) < len ? kTable[(triple >> 6) & 0x3F] : '=');
      out.push_back((i + 2) < len ? kTable[triple & 0x3F] : '=');
    }

    return out;
  }

  static std::string extract_output_text(const nlohmann::json& object)
  {
    if (object.contains("error") && !object["error"].is_null())
    {
      try
      {
        const auto& err = object["error"];

        if (err.is_object())
        {
          if (err.contains("message"))
          {
            if (err["message"].is_string())
              return std::string("OpenAI error: ") + err["message"].get<std::string>();
            return std::string("OpenAI error: ") + err["message"].dump();
          }

          return std::string("OpenAI error: ") + err.dump();
        }

        if (err.is_string())
          return std::string("OpenAI error: ") + err.get<std::string>();
      }
      catch (...) {}
      try
      {
        return std::string("OpenAI error: ") + object["error"].dump();
      }
      catch (...) {}
      return "OpenAI error";
    }

    if (object.contains("output_text") && object["output_text"].is_string())
      return object["output_text"].get<std::string>();

    std::string text;
    if (object.contains("output") && object["output"].is_array())
    {
      for (const auto& out : object["output"])
      {
        if (!out.is_object() || !out.contains("content") || !out["content"].is_array())
          continue;

        for (const auto& content : out["content"])
        {
          if (content.is_object() && content.contains("type") && content["type"].is_string() &&
              content["type"].get<std::string>() == "output_text" && content.contains("text") &&
              content["text"].is_string())
          {
            if (!text.empty())
              text += "\n";
            text += content["text"].get<std::string>();
          }
        }
      }
    }

    return text;
  }

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
    if (request.file_stream.empty())
      throw perception_exception("Image analysis request missing image payload");

    const auto base64_image = base64_encode(
        reinterpret_cast<const unsigned char*>(request.file_stream.data()), request.file_stream.size());
    const std::string mime = request.file_type.empty() ? std::string("image/png") : request.file_type;
    const std::string data_url = "data:" + mime + ";base64," + base64_image;

    nlohmann::json body;
    body["model"] = model_name_;

    nlohmann::json content = nlohmann::json::array();
    content.push_back({ { "type", "input_text" }, { "text", request.prompt } });

    nlohmann::json image_obj = { { "type", "input_image" }, { "image_url", data_url } };
    if (!detail_.empty())
      image_obj["detail"] = detail_;
    content.push_back(image_obj);

    body["input"] = nlohmann::json::array(
        { { { "role", "user" }, { "content", content } } });

    return body;
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

    res.response = extract_output_text(object);
    const bool has_real_error = object.contains("error") && !object["error"].is_null();
    res.success = !has_real_error && !res.response.empty();

    return res;
  }

  perception::RESTResponse response_;

  std::string model_name_;
  std::string test_file_path_;
  std::string test_prompt_;
  std::string detail_;
};

}  // namespace perception
