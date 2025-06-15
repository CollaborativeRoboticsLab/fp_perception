#pragma once

#include <prompt_msgs/srv/prompt.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>
#include <any>
#include <perception_base/driver_base.hpp>

namespace perception
{
/**
 * @brief PromptToolsSentimentDriver class for handling prompt_tools based sentument analysis.
 *
 *
 */
using PromptSrv = prompt_msgs::srv::Prompt;

class PromptToolsSentimentDriver : public DriverBase
{
public:
  PromptToolsSentimentDriver() = default;
  ~PromptToolsSentimentDriver() override = default;

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
    node->declare_parameter("driver.sentiment.PromptToolsSentimentDriver.name", "PromptToolsSentimentDriver");

    config_.name = node->get_parameter("driver.sentiment.PromptToolsSentimentDriver.name").as_string();

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
    sentiment_client_ = node_->create_client<PromptSrv>("prompt_bridge/sentiment");
    event_->info("Sentiment Analysis service client created");
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
  std::any getData() const override
  {
    // wait until the future is ready
    future_.wait();

    // Implement the logic to get the latest sentiment analysis data
    try
    {
      if (!future_.valid())
      {
        event_->error("Sent service call is not valid.");
        throw perception_exception("Sentiment analysis service call is not valid.");
      }

      auto response = future_.get();
      event_->info("Analysis successful: " + response->response.response +
                   " with confidence: " + std::to_string(response->response.confidence));

      return std::pair(response->response.response, response->response.confidence);
    }
    catch (const std::exception& e)
    {
      event_->error("Analysis service call failed: " + std::string(e.what()));
    }
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
    const auto& text = std::any_cast<const std::string>(input);

    // Implement the logic to set the transcription data
    PromptSrv::Request::SharedPtr request = std::make_shared<PromptSrv::Request>();
    request->prompt.prompt = text;
    request->prompt.flush = true;
    request->prompt.contains_audio = false;

    // Call the transcription service
    future_ = sentiment_client_->async_send_request(request);
  }

  /**
   * @brief Test method for the driver
   *
   * This function can be overridden in derived classes to implement specific test logic.
   */
  void test() override
  {
    event_->info("Testing PromptToolsSentimentDriver with model: " + config_.name);

    // Example test input
    std::string test_text = "I love programming!";
    setDataStream(test_text);

    event_->info("Sentiment analysis service called with test text. waiting for response...");

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
  rclcpp::Client<PromptSrv>::SharedPtr sentiment_client_;
  mutable rclcpp::Client<PromptSrv>::SharedFuture future_;
};

}  // namespace perception
