#ifndef DRIVER_BASE_HPP_
#define DRIVER_BASE_HPP_

#include <string>
#include <any>

#include <rclcpp/rclcpp.hpp>

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
   * @brief Start the driver streaming with a ROS node and namespace
   *
   * @param node Shared pointer to the ROS node
   * @param config configuration loaded from the yaml file
   */
  virtual void start(const rclcpp::Node::SharedPtr& node, const driver_options& config) = 0;

  /**
   * @brief Stop driver streaming
   *
   */
  virtual void stop() = 0;

  /**
   * @brief Get latest data from the driver
   *
   * This function should be overridden in derived classes to provide specific data.
   *
   * @return std::any The latest data from the driver.
   * @throws perception_exception if not implemented in derived classes
   */
  virtual std::any getData() const
  {
    throw perception_exception("getData() not implemented for this driver.");
  }

  /**
   * @brief Get latest data from the driver as a stream
   *
   * This function should be overridden in derived classes to provide specific data.
   *
   * @return std::any The latest data from the driver.
   * @throws perception_exception if not implemented in derived classes
   */
  virtual std::any getDataStream() const
  {
    throw perception_exception("getDataStream() not implemented for this driver.");
  }

protected:
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
    event_ = std::make_shared<EventClient>(node_, config.name, "/events");
  }

  virtual std::string getName() const
  {
    return config_.name;
  }

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
