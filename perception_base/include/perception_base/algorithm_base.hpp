#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <perception_base/utils/options.hpp>
#include <perception_base/utils/exceptions.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_events/event_client.hpp>

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
     * @brief Set the audio driver shared pointer
     * 
     * @param audio_driver 
     */
    void set_audio_driver(const std::shared_ptr<perception::DriverBase> &audio_driver)
    {
      audio_driver_ = audio_driver;
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
      event_ = std::make_shared<EventClient>(node_, config_.name, "/events");
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
     * @brief client for publishing events
     */
    std::shared_ptr<EventClient> event_;

    /**
     * @brief driver for vision
     */
    std::shared_ptr<perception::DriverBase> vision_driver_;

    /**
     * @brief driver for audio
     */
    std::shared_ptr<perception::DriverBase> audio_driver_;
  };

} // namespace perception
