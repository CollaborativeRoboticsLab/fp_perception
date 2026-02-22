#pragma once

#include <string>
#include <any>
#include <mutex>
#include <thread>
#include <filesystem>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <perception_base/exceptions.hpp>

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
   * @brief Deinitialize the driver
   *
   */
  virtual void deinitialize() = 0;

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
    return name_;
  }

  /**
   * @brief Test the driver
   *
   */
  virtual void test()
  {
    RCLCPP_INFO(node_->get_logger(), "Driver test function called for driver: %s", name_.c_str());
  }

  /**
   * @brief Check if the test directory exists, create it if not
   *
   * This function checks if a directory named "test" exists in the current working directory.
   * If it does not exist, it creates the directory.
   *
   * @param folder_name The name of the folder to check or create.
   */
  void check_directory(std::string folder_name)
  {
    // Create "test" directory if it doesn't exist
    const std::filesystem::path dir(folder_name);
    if (!std::filesystem::exists(dir))
    {
      std::filesystem::create_directory(dir);
    }
  }

  const std::filesystem::path check_file(const std::string& file_name)
  {
    // Check if the file exists
    const std::filesystem::path file_path(file_name);
    if (!std::filesystem::exists(file_path))
    {
      throw perception_exception("File does not exist: " + file_name);
    }
    return file_path;
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
  }

  /**
   * @brief ROS node for the driver
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief Name of the driver
   */
  std::string name_;

  /**
   * @brief Name of the device for the driver
   */
  std::string device_name_;

  /**
   * @brief Mutex to protect access to the driver data
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
