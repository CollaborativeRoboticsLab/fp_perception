cmake_minimum_required(VERSION 3.8)
project(perception)

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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

############################################################################
# perception server executable
############################################################################

add_executable(${PROJECT_NAME}_node
  src/server_node.cpp
)

# Link OpenCV explicitly
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  pluginlib
  perception_base
  perception_msgs
  cv_bridge
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

############################################################################
# perception server component
############################################################################

add_library(${PROJECT_NAME}_comp SHARED
  src/server_component.cpp
)

ament_target_dependencies(${PROJECT_NAME}_comp
  rclcpp
  rclcpp_components
  pluginlib
  perception_base
  perception_msgs
  cv_bridge
)

# register component
rclcpp_components_register_node(${PROJECT_NAME}_comp
  PLUGIN "perception::PerceptionServer"
  EXECUTABLE perception_server_component
)

ament_export_targets(perception_server_component)

install(TARGETS ${PROJECT_NAME}_comp
  EXPORT perception_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

############################################################################
# Miscellaneous installations
############################################################################

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_comp)
ament_package()
