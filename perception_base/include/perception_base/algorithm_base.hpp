#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <perception_base/options.hpp>
#include <perception_base/exceptions.hpp>
#include <perception_base/driver_base.hpp>

namespace perception
{

  class AlgorithmBase
  {
  public:
    virtual ~AlgorithmBase() = default;

    /**
     * @brief Initialize the driver with a ROS node and namespace
     *
     * @param node Shared pointer to the ROS node
     */
    virtual void initialize(const rclcpp::Node::SharedPtr &node)
    {
      initialize_base(node);
    }

    /**
     * @brief Start the algorithm
     *
     */
    virtual void start() = 0;

    /**
     * @brief Stop the algorithm
     *
     */
    virtual void stop() = 0;

    /**
     * @brief Get the name of the algorithm
     *
     * This function returns the name of the algorithm. It can be overridden
     * by derived classes to provide specific implementations.
     *
     * @return Name of the algorithm
     */
    virtual std::string getName() const
    {
      return config_.name;
    }

    /**
     * @brief Set the vision driver shared pointer
     * 
     * @param vision_driver 
     */
    void set_vision_driver(const std::shared_ptr<perception::DriverBase> &vision_driver)
    {
      vision_driver_ = vision_driver;
    }
    
    /**
     * @brief Set the audio input driver shared pointer
     * 
     * @param audio_driver 
     */
    void set_audio_input_driver(const std::shared_ptr<perception::DriverBase> &audio_driver)
    {
      audio_input_driver_ = audio_driver;
    }

    /**
     * @brief Set the transcription driver shared pointer
     * 
     * @param transcription_driver 
     */
    void set_transcription_driver(const std::shared_ptr<perception::DriverBase> &transcription_driver)
    {
      transcription_driver_ = transcription_driver;
    }

    /**
     * @brief Set the sentiment driver shared pointer
     * 
     * @param sentiment_driver 
     */
    void set_sentiment_driver(const std::shared_ptr<perception::DriverBase> &sentiment_driver)
    {
      sentiment_driver_ = sentiment_driver;
    }

  protected:
    /**
     * @brief Initializer base driver in place of constructor due to plugin semantics
     *
     * @param node shared pointer to the ROS node.
     */
    void initialize_base(const rclcpp::Node::SharedPtr &node)
    {
      node_ = node;
    }

    /**
     * @brief ROS node for the driver
     */
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief algorithm options
     */
    algorithm_options config_;

    /**
     * @brief driver for vision
     */
    std::shared_ptr<perception::DriverBase> vision_driver_;

    /**
     * @brief driver for audio input
     */
    std::shared_ptr<perception::DriverBase> audio_input_driver_;

    /**
     * @brief driver for transcription
     */
    std::shared_ptr<perception::DriverBase> transcription_driver_;

    /**
     * @brief driver for sentiment analysis
     */
    std::shared_ptr<perception::DriverBase> sentiment_driver_;
  };

} // namespace perception
