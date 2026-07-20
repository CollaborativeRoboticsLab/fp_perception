#pragma once

#include <fp_perception_base/rest_base.hpp>
#include <fp_perception_base/sentiment/sentiment_analysis_driver.hpp>
#include <fp_perception_base/sentiment/structs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace fp_perception
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

    if (diagnostics_enabled())
    {
      enable_diagnostics("rest-sentiment-" + name_, name_ + " status",
                         [this](diagnostic_updater::DiagnosticStatusWrapper& status) { produce_diagnostics(status); });
    }

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
    disable_diagnostics();
    response_ = fp_perception::RESTResponse{};
    name_.clear();
    node_.reset();
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
    fp_perception::RESTRequest request;
    request.prompt = request_data.text;

    response_ = call(request);

    sentiment_result result;
    result.label = response_.response;
    result.score = response_.confidence;
    result.analyzed_text = request_data.text;
    result.success = !response_.response.empty();
    if (!result.success)
      result.error = "No response received from sentiment analysis service";

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
      throw fp_perception_exception(sentiment.error.empty() ? "No response received from sentiment analysis service" :
                                                              sentiment.error);
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
  nlohmann::json toJson(const fp_perception::RESTRequest& request) override
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
  fp_perception::RESTResponse fromJson(const nlohmann::json& object) override
  {
    fp_perception::RESTResponse res;

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
      throw fp_perception_exception("Unexpected sentiment JSON structure received");

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
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Sentiment driver idle");
    else if (last_rest_success_.load())
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Last sentiment request succeeded");
    else
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Last sentiment request failed");

    status.add("uri", uri_);
    status.add("request_count", rest_request_count_.load());
    status.add("failure_count", rest_failure_count_.load());
    status.add("last_http_code", last_rest_http_code_.load());
    status.add("last_result_success", last_result_.success ? "true" : "false");
    status.add("last_label", last_result_.label.empty() ? std::string("none") : last_result_.label);
    status.add("last_score", last_result_.score);
    status.add("last_error", last_error.empty() ? std::string("none") : last_error);
  }

  fp_perception::RESTResponse response_;
  sentiment_result last_result_;
};

}  // namespace fp_perception
