#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <perception_msgs/msg/perception_event.hpp>

class EventListener : public rclcpp::Node
{
public:
  using Event = perception_msgs::msg::PerceptionEvent;

  /**
   * @brief Construct a new Event Listener object
   *
   * @param options Node options for the event listener
   */
  EventListener(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("events_listener", options)
  {
    // Create a subscription to the "topic" topic
    RCLCPP_INFO(this->get_logger(), "Creating subscription to topic");

    subscription_ =
        this->create_subscription<Event>("/events", 10, std::bind(&EventListener::topic_callback, this, std::placeholders::_1));
  }

private:
  /**
   * @brief Callback function for the subscription
   *
   * @param msg The received message
   */
  void topic_callback(const Event& msg) const
  {
    std::string text;

    if (msg.thread_id >= 0)
    {
      text = "[" + msg.origin_node + "]" + "[" + std::to_string(msg.thread_id) + "] " + msg.content;
    }
    else
    {
      text = "[" + msg.origin_node + "] " + msg.content;
    }

    if (msg.type == Event::ERROR)
      RCLCPP_ERROR(get_logger(), text.c_str());
    else if (msg.type == Event::DEBUG)
      RCLCPP_DEBUG(get_logger(), text.c_str());
    else
      RCLCPP_INFO(get_logger(), text.c_str());
  }

  rclcpp::Subscription<Event>::SharedPtr subscription_;
};