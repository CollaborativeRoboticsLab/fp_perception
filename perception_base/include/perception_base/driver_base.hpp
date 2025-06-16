#pragma once

#include <string>
#include <any>
#include <mutex>
#include <thread>
#include <filesystem>
#include <atomic>

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
  virtual void initialize(const rclcpp::Node::SharedPtr& node)
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
  virtual std::any getData()
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
  virtual void setData(const std::any& input)
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
  virtual std::any getDataStream()
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
  virtual void setDataStream(const std::any& input)
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
   * @brief Driver thread function. need to be overridden in derived classesif used
   */
  virtual void driver_thread()
  {
    // This is a placeholder for the driver thread logic.
    while (rclcpp::ok())
    {
      // Here you would typically gather data from the device and publish it.
      std::lock_guard<std::mutex> lock(buffer_mutex_);

      // Simulate data gathering
      // In a real implementation, you would gather data from the device here.
      event_->info("Driver thread started for " + config_.name + " without implementation");

      // If publishing is enabled, you would publish the data to a topic.
      std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 / config_.frequency)));
    }
  }

  /**
   * @brief Test the driver
   *
   */
  virtual void test()
  {
    event_->info("Driver test function called");
  }

  /**
   * @brief Check if the test directory exists, create it if not
   *
   * This function checks if a directory named "test" exists in the current working directory.
   * If it does not exist, it creates the directory.
   *
   * @param folder_name The name of the folder to check or create.
   */
  void check_test_directory(std::string folder_name)
  {
    // Create "test" directory if it doesn't exist
    const std::filesystem::path dir(folder_name);
    if (!std::filesystem::exists(dir))
    {
      std::filesystem::create_directory(dir);
    }
  }

protected:
  /**
   * @brief Initializer base driver in place of constructor due to plugin semantics
   *
   * @param node shared pointer to the ROS node.
   */
  void initialize_base(const rclcpp::Node::SharedPtr& node)
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

  /**
   * @brief Mutex to protect access to the camera data
   *
   */
  std::mutex buffer_mutex_;

  /**
   * @brief Condition variable to notify when new data is available
   *
   */
  std::condition_variable buffer_cv_;

  /**
   * @brief Thread for gathering data from the device for publishing
   *
   */
  std::thread driver_thread_;  // Thread for capturing images

  /**
   * @brief Flag to indicate if the driver thread is running
   *
   */
  std::atomic<bool> is_running_{ false };
};
}  // namespace perception
