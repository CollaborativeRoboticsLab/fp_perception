#pragma once

#include <stdexcept>
#include <string>


namespace perception
{
/**
 * @brief Base class for driver exceptions
 *
 */
struct perception_exception : public std::runtime_error
{
  using std::runtime_error::runtime_error;

  perception_exception(const std::string& what) : std::runtime_error(what)
  {
  }

  virtual const char* what() const noexcept override
  {
    return std::runtime_error::what();
  }
};
}