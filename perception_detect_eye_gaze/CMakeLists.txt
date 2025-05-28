cmake_minimum_required(VERSION 3.8)
project(perception_detect_eye_gaze)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(Torch_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pytorch_vendor/external/libtorch/share/cmake/Torch")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(perception_base REQUIRED)
find_package(perception_events REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(pytorch_vendor REQUIRED)
find_package(Torch REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/eye_gaze_algorithm.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  pluginlib
  cv_bridge
  perception_base
  perception_events
  perception_msgs
  pytorch_vendor
)

# Link OpenCV explicitly
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC ${TORCH_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME} 
  PROPERTIES 
  CXX_STANDARD 17 
  CXX_STANDARD_REQUIRED YES
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

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()