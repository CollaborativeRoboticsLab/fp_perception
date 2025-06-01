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
  std::string topic;
  int device_id;
  std::string frame_id;
  bool publish = false;  // whether to publish the data to a topic
  bool subscribe = false;  // whether to subscribe to the data from a topic
  double frequency;  // frequency of data capture in Hz
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
