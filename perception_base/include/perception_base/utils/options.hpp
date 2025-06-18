#pragma once
#include <string>

namespace perception
{

/**
 * @brief driver options
 *
 * Contains the options required to start and maintain a consistent driver.
 * normally loaded from the yaml file
 *
 */
struct driver_options
{
  std::string name;
  std::string interface_name;
  std::string device_name;  // name of the device, used to find the device ID
  int device_id;
  std::string frame_id;
  bool interface_enabled;  // whether the interface is enabled or not
  double frequency;        // frequency of data capture in Hz
};

/**
 * @brief algorithm options
 *
 * Contains the options required to start and maintain a consistent algorithm.
 *
 */
struct algorithm_options
{
  std::string name;
  std::string started_by;
  std::string pid;
};

}  // namespace perception
