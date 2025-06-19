#pragma once

#include <any>
#include <perception_base/rest_base.hpp>
#include <perception_msgs/srv/perception_sentiment.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

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
class SentimentDriver : public RestBase
{
public:
  using Sentiment = perception_msgs::srv::PerceptionSentiment;
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
    node->declare_parameter("driver.sentiment.SentimentDriver.service_name", "perception/sentiment_analysis");
    node->declare_parameter("driver.sentiment.SentimentDriver.provide_service", false);

    config_.name = node->get_parameter("driver.sentiment.SentimentDriver.name").as_string();
    config_.interface_name = node->get_parameter("driver.sentiment.SentimentDriver.service_name").as_string();
    config_.interface_enabled = node->get_parameter("driver.sentiment.SentimentDriver.provide_service").as_bool();

    // Initialize the base driver
    initialize_rest_base(node, "driver.sentiment.SentimentDriver", "HUGGINGFACE_API_KEY");

    // Log the parameters
    event_->info("Assigned driver Name: " + config_.name);
    event_->info("Assigned driver Service Name: " + config_.interface_name);
    event_->info("Assigned driver Provide Service: " + std::string(config_.interface_enabled ? "true" : "false"));

    // If the service is enabled, create the service
    if (config_.interface_enabled)
    {
      sentiment_service_ = node->create_service<Sentiment>(
          config_.interface_name,
          std::bind(&SentimentDriver::service_cb, this, std::placeholders::_1, std::placeholders::_2));

      event_->info("Sentiment service created: " + config_.interface_name);
    }
    else
    {
      event_->info("Sentiment service not enabled.");
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
   * This function waits for the sentiment analysis service to complete and retrieves the latest sentiment analysis
   * data.
   *
   * @return std::any containing the latest sentiment analysis data, which is a std::pair<std::string, double>
   * @throws perception_exception if not implemented in derived classes
   */
  std::any getData() override
  {
    std::pair<std::string, double> sentiment_result;

    if (response_.response.empty())
    {
      throw perception_exception("No response received from sentiment analysis service");
    }
    else
    {
      sentiment_result.first = response_.response;     // The sentiment label
      sentiment_result.second = response_.confidence;  // The confidence score
    }

    return sentiment_result;
  }
  /**
   * @brief Set data to the driver
   *
   * This function sends the latest audio data to the transcription service. It expects the input to be a vector of
   * audio chunks in the form of std::vector<std::vector<int16_t>>.
   *
   * @param input The latest data from the driver.
   */
  void setDataStream(const std::any& input) override
  {
    const auto& text = std::any_cast<const std::string>(input);

    perception::RESTRequest request;
    request.prompt = text;

    response_ = call(request);
  }

  /**
   * @brief Test method for the driver
   *
   * This function can be overridden in derived classes to implement specific test logic.
   */
  void test() override
  {
    event_->info("Testing SentimentDriver with model: " + uri_);

    // Example test input
    std::string test_text = "I love programming!";

    // Wait for the service to process the request
    event_->info("Initiated Sentiment analysis for text: " + test_text);

    setDataStream(test_text);

    auto result = getData();

    if (result.has_value())
    {
      auto sentiment_result = std::any_cast<std::pair<std::string, double>>(result);

      event_->info("Analysis results with sentiment: " + sentiment_result.first +
                   " and confidence: " + std::to_string(sentiment_result.second));
    }

    event_->info("Test completed.");
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

  /**
   * @brief Callback function for the sentiment analysis service
   *
   * This function is called when a request is received by the sentiment analysis service.
   * It processes the request and sends back a response with the sentiment analysis results.
   *
   * @param request The request received from the client
   * @param response The response to be sent back to the client
   */
  void service_cb(const std::shared_ptr<Sentiment::Request> request, std::shared_ptr<Sentiment::Response> response)
  {
    event_->info("Received sentiment analysis request");

    setDataStream(request->text);

    auto result = getData();

    if (result.has_value())
    {
      auto sentiment_result = std::any_cast<std::pair<std::string, double>>(result);

      // Set the response data
      response->label = sentiment_result.first;   // Example response
      response->score = sentiment_result.second;  // Example confidence score
    }
    else
    {
      event_->error("No sentiment analysis result available");
      response->label = "Error: No sentiment analysis result available";
      response->score = 0.0;
    }
  }

  perception::RESTResponse response_;
  rclcpp::Service<Sentiment>::SharedPtr sentiment_service_;
};

}  // namespace perception
