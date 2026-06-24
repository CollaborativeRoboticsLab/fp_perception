#pragma once

#include <memory>
#include <string>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <perception_base/driver_base.hpp>
#include <perception_base/exceptions.hpp>

namespace perception
{

class DriverManager
{
public:
  DriverManager(const rclcpp::Node::SharedPtr& node, pluginlib::ClassLoader<perception::DriverBase>& loader)
    : node_(node), loader_(loader)
  {
  }

  template<typename DriverType>
  std::shared_ptr<DriverType> loadDriver(const std::string& parameter_name, const std::string& default_plugin_name,
                                         const std::string& driver_description,
                                         const std::string& interface_description) const
  {
    node_->declare_parameter(parameter_name, default_plugin_name);
    const auto plugin_name = node_->get_parameter(parameter_name).as_string();

    RCLCPP_INFO(node_->get_logger(), "Loading %s plugin: %s", driver_description.c_str(), plugin_name.c_str());

    auto driver_base = loader_.createSharedInstance(plugin_name);
    driver_base->initialize(node_);

    auto typed_driver = std::dynamic_pointer_cast<DriverType>(driver_base);
    if (!typed_driver)
    {
      throw perception_exception("Loaded " + driver_description + " does not implement " + interface_description +
                                 ": " + plugin_name);
    }

    RCLCPP_INFO(node_->get_logger(), "Started %s plugin: %s", driver_description.c_str(), plugin_name.c_str());

    return typed_driver;
  }

  template<typename DriverType>
  static void testDriver(const rclcpp::Logger& logger, const std::shared_ptr<DriverType>& driver,
                         const std::string& driver_description)
  {
    if (!driver)
      return;

    RCLCPP_INFO(logger, "Testing %s...", driver_description.c_str());
    driver->test();
  }

private:
  rclcpp::Node::SharedPtr node_;
  pluginlib::ClassLoader<perception::DriverBase>& loader_;
};

}  // namespace perception