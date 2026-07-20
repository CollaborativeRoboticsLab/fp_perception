#pragma once

#include <string>
#include <chrono>
#include <functional>
#include <mutex>
#include <memory>
#include <filesystem>
#include <atomic>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fp_perception_base/exceptions.hpp>

namespace fp_perception
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
      throw fp_perception_exception("File does not exist: " + file_name);
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

    if (!node_->has_parameter("use_diagnostics"))
      node_->declare_parameter("use_diagnostics", false);

    diagnostics_enabled_ = node_->get_parameter("use_diagnostics").as_bool();
  }

  bool diagnostics_enabled() const
  {
    return diagnostics_enabled_;
  }

  void enable_diagnostics(const std::string& hardware_id, const std::string& task_name,
                          std::function<void(diagnostic_updater::DiagnosticStatusWrapper&)> task,
                          std::chrono::milliseconds period = std::chrono::seconds(1))
  {
    if (!diagnostics_enabled_ || !node_)
      return;

    diagnostics_timer_.reset();
    diagnostics_updater_.reset();

    diagnostics_updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
    diagnostics_updater_->setHardwareID(hardware_id.empty() ? name_ : hardware_id);
    diagnostics_updater_->add(task_name, std::move(task));
    diagnostics_timer_ = node_->create_wall_timer(period, [this]() {
      if (diagnostics_updater_)
        diagnostics_updater_->force_update();
    });
  }

  void disable_diagnostics()
  {
    diagnostics_timer_.reset();
    diagnostics_updater_.reset();
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
   * @brief Flag to indicate if the driver thread is running
   *
   */
  std::atomic<bool> is_running_{ false };

  /**
   * @brief Flag to enable standard ROS diagnostics publishing.
   */
  bool diagnostics_enabled_{ false };

  std::unique_ptr<diagnostic_updater::Updater> diagnostics_updater_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};
}  // namespace fp_perception
