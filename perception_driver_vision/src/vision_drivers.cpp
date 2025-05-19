#include <pluginlib/class_list_macros.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_driver_vision/default_vision_driver.hpp>
#include <perception_driver_vision/opencv_vision_driver.hpp>

PLUGINLIB_EXPORT_CLASS(perception::DefaultDriver, perception::DriverBase);
PLUGINLIB_EXPORT_CLASS(perception::OpenCVDriver, perception::DriverBase);