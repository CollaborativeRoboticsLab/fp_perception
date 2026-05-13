#pragma once

#include <any>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <perception_base/rest_base.hpp>
#include <perception_base/sentiment/sentiment_analysis_driver.hpp>
#include <perception_base/sentiment/structs.hpp>

namespace perception
{
/**
 * @brief SentimentDriver class for handling prompt_tools based sentument analysis.
 *
 *  This class is responsible for managing the sentiment analysis using the prompt_tools service.
 *  It provides methods to initialize the driver, start and stop the service, and retrieve sentiment analysis data.
 *  It uses the prompt_msgs::srv::Prompt service to send text for sentiment analysis and receive the response.
 *
 */
class SentimentDriver : public RestBase, public SentimentAnalysisDriver
{
public:
  /**
   * @brief Constructor for SentimentDriver
   *
   * Initializes the sentiment analysis service client.
   */
  SentimentDriver()
  {
  }

  /**
   * @brief Destructor for SentimentDriver
   *
   * Cleans up the sentiment analysis service client.
   */
  ~SentimentDriver() override
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
    node->declare_parameter("driver.sentiment.SentimentDriver.name", "SentimentDriver");

    name_ = node->get_parameter("driver.sentiment.SentimentDriver.name").as_string();

    // Initialize the base driver
    initialize_rest_base(node, "driver.sentiment.SentimentDriver", "HUGGINGFACE_API_KEY");

    // Log the parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Name: %s", name_.c_str());

    // Log that the driver has been initialized
    RCLCPP_INFO(node_->get_logger(), "Initialized");
  }

  /**
   * @brief Deinitialize the driver
   *
   * Required by DriverBase. Clears cached response state and releases the node.
   */
  void deinitialize() override
  {
    response_ = perception::RESTResponse{};
    name_.clear();
    node_.reset();
  }

  /**
   * @brief Get latest data from the driver
   *
   * This function waits for the sentiment analysis service to complete and retrieves the latest sentiment analysis
   * data.
   *
   * @return std::any containing the latest sentiment analysis data, which is a std::pair<std::string, double>
   * @throws perception_exception if not implemented in derived classes
   */
  std::any getData() override
  {
    return last_result_;
  }
  /**
   * @brief Set data to the driver
   *
   * This function sends the latest audio data to the transcription service. It expects the input to be a vector of
   * audio chunks in the form of std::vector<std::vector<int16_t>>.
   *
   * @param input The latest data from the driver.
   */
  sentiment_result analyze(const sentiment_request& request_data) override
  {
    perception::RESTRequest request;
    request.prompt = request_data.text;

    response_ = call(request);

    sentiment_result result;
    result.label = response_.response;
    result.score = response_.confidence;
    result.success = !response_.response.empty();
    if (!result.success)
      result.error = "No response received from sentiment analysis service";

    last_result_ = result;
    return last_result_;
  }

  void setDataStream(const std::any& input) override
  {
    analyze(std::any_cast<const sentiment_request&>(input));
  }

  /**
   * @brief Test method for the driver
   *
   * This function can be overridden in derived classes to implement specific test logic.
   */
  void test() override
  {
    RCLCPP_INFO(node_->get_logger(), "Testing SentimentDriver with model: %s", uri_.c_str());

    // Example test input
    sentiment_request request;
    request.text = "I love programming!";

    // Wait for the service to process the request
    RCLCPP_INFO(node_->get_logger(), "Initiated Sentiment analysis for text: %s", request.text.c_str());

    const auto sentiment = analyze(request);

    if (sentiment.success)
    {
      RCLCPP_INFO(node_->get_logger(), "Analysis results with sentiment: %s and confidence: %f",
                  sentiment.label.c_str(), sentiment.score);
    }
    else
    {
      throw perception_exception(sentiment.error.empty() ? "No response received from sentiment analysis service"
                                                         : sentiment.error);
    }

    RCLCPP_INFO(node_->get_logger(), "Test completed.");
  }

protected:
  /**
   * @brief Convert a prompt request to a JSON object
   *
   * This method converts the sentiment request to a JSON object that can be sent to the sentiment analysis service.
   * It includes the prompt text and options for the JSON object.
   *
   * @param prompt The sentiment request to convert
   * @return A JSON object representing the prompt request
   */
  nlohmann::json toJson(const perception::RESTRequest& request) override
  {
    nlohmann::json result;

    // Add options
    for (const auto& option : request.options)
    {
      result[option.key] = option.value;
    }

    // Add prompt
    result["inputs"] = request.prompt;

    return result;
  }

  /**
   * @brief Convert a JSON object to a RESTResponse
   *
   * This method converts the JSON response from the sentiment analysis service to a RESTResponse object.
   *
   * @param object The JSON object to convert
   * @return A RESTResponse object representing the sentiment analysis response
   */
  perception::RESTResponse fromJson(const nlohmann::json& object) override
  {
    perception::RESTResponse res;

    // Hugging Face returns a double-nested array: [[{label, score}, {label, score}]]
    if (object.is_array() && !object.empty() && object[0].is_array() && !object[0].empty())
    {
      const auto& firstResult = object[0][0];
      if (firstResult.contains("label") && firstResult.contains("score"))
      {
        res.response = firstResult["label"].get<std::string>();
        res.confidence = firstResult["score"].get<double>();
      }
    }
    else
      throw perception_exception("Unexpected sentiment JSON structure received");

    return res;
  }

  perception::RESTResponse response_;
  sentiment_result last_result_;
};

}  // namespace perception
