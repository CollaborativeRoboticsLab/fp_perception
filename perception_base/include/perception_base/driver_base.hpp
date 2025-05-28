#pragma once

#include <string>
#include <any>

#include <rclcpp/rclcpp.hpp>

#include <perception_events/event_client.hpp>
#include <perception_base/utils/options.hpp>
#include <perception_base/utils/exceptions.hpp>

namespace perception
{

  class DriverBase
  {
  public:
    virtual ~DriverBase() = default;

    /**
     * @brief Initialize the driver
     *
     * This function should be overridden in derived classes to provide specific initialization.
     *
     * @param node Shared pointer to the ROS node
     */
    virtual void initialize(const rclcpp::Node::SharedPtr &node)
    {
      initialize_base(node);
    }

    /**
     * @brief Start the driver streaming
     *
     */
    virtual void start() = 0;

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
     * @brief Set data to the driver
     *
     * This function should be overridden in derived classes to provide specific data.
     *
     * @param  input The latest data from the driver.
     * @throws perception_exception if not implemented in derived classes
     */
    virtual void setData(std::any &input) const
    {
      throw perception_exception("setData() not implemented for this driver.");
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

    /**
     * @brief Set data to the driver as a stream
     *
     * This function should be overridden in derived classes to provide specific data.
     *
     * @return std::any The latest data from the driver.
     * @throws perception_exception if not implemented in derived classes
     */
    virtual void setDataStream(std::any &input) const
    {
      throw perception_exception("setDataStream() not implemented for this driver.");
    }

    /**
     * @brief Get the name of the driver
     *
     * @return std::string The name of the driver
     */
    virtual std::string getName() const
    {
      return config_.name;
    }

    /**
     * @brief Test the driver
     * 
     */
    virtual void test()
    {
      event_->info("Driver test function called");
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
     * @brief driver options
     */
    driver_options config_;

    /**
     * @brief client for publishing events
     */
    std::shared_ptr<EventClient> event_;
  };
} // namespace perception
