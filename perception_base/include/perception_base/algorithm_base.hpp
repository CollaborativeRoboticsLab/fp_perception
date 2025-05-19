#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <perception_base/utils/options.hpp>
#include <perception_base/utils/exceptions.hpp>
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
  virtual void initialize(const rclcpp::Node::SharedPtr& node, const algorithm_options& config) = 0;

  /**
   * @brief Initializer base driver in place of constructor due to plugin semantics
   *
   * @param node shared pointer to the ROS node.
   * @param run_config configuration loaded from the yaml file
   */
  void initialize_base(const rclcpp::Node::SharedPtr& node, const algorithm_options& config)
  {
    node_ = node;
    config_ = config;
    event_ = std::make_shared<EventClient>(node_, config.name_space + "/" + config.name, "/events");
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
  virtual std::string getName() const = 0;

private:
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
};

}  // namespace perception
