#pragma once

#include <prompt_msgs/srv/prompt.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

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

protected:
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

      auto response = future.get();
      if (response->response.response)
      {
        event_->info("Analysis successful: " + response->response.response +
                     " with confidence: " + std::to_string(response->response.confidence));
      }
    }
    catch (const std::exception& e)
    {
      event_->error("Analysis service call failed: " + std::string(e.what()));
    }

    return std::pair(response->response.response, response->response.confidence);
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
    const auto& text = std::any_cast<const std::string>& > (input);

    // Implement the logic to set the transcription data
    PromptSrv::Request::SharedPtr request = std::make_shared<PromptSrv::Request>();
    request->prompt.prompt = text;
    request->prompt.flush = true;
    request->prompt.contains_audio = false;

    // Call the transcription service
    future_ = transcribe_client_->async_send_request(request);
  }

  rclcpp::Client<PromptSrv>::SharedPtr sentiment_client_;
  rclcpp::Client<PromptSrv>::SharedFuture future_;
}

}  // namespace perception
