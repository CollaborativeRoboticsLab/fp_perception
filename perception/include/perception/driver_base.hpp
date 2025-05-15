#ifndef DRIVER_BASE_HPP_
#define DRIVER_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <perception_events/event_client.hpp>
#include <perception/options.hpp>
#include <perception/exceptions.hpp>

namespace perception
{

class DriverBase
{
public:
  virtual ~DriverBase() = default;

  /**
   * @brief Initialize the driver with a ROS node and namespace
   *
   * @param node Shared pointer to the ROS node
   * @param config configuration loaded from the yaml file
   */
  virtual void initialize(const rclcpp::Node::SharedPtr& node, const driver_options& config) = 0;

  /**
   * @brief Initializer base driver in place of constructor due to plugin semantics
   *
   * @param node shared pointer to the ROS node.
   * @param run_config configuration loaded from the yaml file
   */
  void initialize_base(const rclcpp::Node::SharedPtr& node, const driver_options& config)
  {
    node_ = node;
    config_ = config;
    event_ = std::make_shared<EventClient>(node_, config.name_space + "/" + config.name, "/events");
  }

  /**
   * @brief Start driver streaming
   *
   */
  virtual void start() = 0;

  /**
   * @brief Stop driver streaming
   *
   */
  virtual void stop() = 0;

  /**
   * @brief Get the name of the driver
   *
   * This function returns the name of the driver. It can be overridden
   * by derived classes to provide specific implementations.
   *
   * @return Name of the driver
   */
  virtual std::string getName() const = 0;

private:
  /**
   * @brief ROS node for the driver
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief driver options
   */
  driver_options config_;

  /**
   * @brief client for publishing events
   */
  std::shared_ptr<EventClient> event_;
};
}  // namespace perception

#endif  // DRIVER_BASE_HPP_
