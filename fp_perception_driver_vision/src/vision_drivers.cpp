#include <pluginlib/class_list_macros.hpp>
#include <fp_perception_base/driver_base.hpp>
#include <fp_perception_driver_vision/default_vision_driver.hpp>
#include <fp_perception_driver_vision/opencv_vision_driver.hpp>

PLUGINLIB_EXPORT_CLASS(fp_perception::DefaultDriver, fp_perception::DriverBase);
PLUGINLIB_EXPORT_CLASS(fp_perception::OpenCVDriver, fp_perception::DriverBase);