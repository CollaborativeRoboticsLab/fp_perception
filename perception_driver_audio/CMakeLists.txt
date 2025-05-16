cmake_minimum_required(VERSION 3.8)
project(perception_driver_audio)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(perception REQUIRED)
find_package(perception_events REQUIRED)
find_package(perception_msgs REQUIRED)

# PortAudio
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

include_directories(
  include
  ${PORTAUDIO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/audio_drivers.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  perception
  perception_events
  perception_msgs
)

# Link PortAudio
target_link_libraries(${PROJECT_NAME}
  ${PORTAUDIO_LIBRARIES}
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