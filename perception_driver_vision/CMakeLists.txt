cmake_minimum_required(VERSION 3.8)
project(perception_driver_vision)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(perception_base REQUIRED)
find_package(perception_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/vision_drivers.cpp
)

# Link OpenCV explicitly
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  pluginlib
  cv_bridge
  image_transport
  sensor_msgs
  perception_base
  perception_msgs
)

pluginlib_export_plugin_description_file(perception plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()