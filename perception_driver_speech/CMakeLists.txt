cmake_minimum_required(VERSION 3.8)
project(perception_driver_speech)

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
find_package(perception_base REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/speech_drivers.cpp
)

target_link_libraries(${PROJECT_NAME}
  CURL::libcurl
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  pluginlib
  perception_base
  perception_msgs
  nlohmann_json
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
