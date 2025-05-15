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
  std::string type;
  std::string name_space; 
  std::string address;
  std::string started_by;
  std::string pid;
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
  std::string name_space;
  std::string started_by;
  std::string pid;
};

}  // namespace perception
