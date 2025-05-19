#include <perception/perception_server.hpp>

int main(int argc, char* argv[])
{
  // Initialize the ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // Create a shared pointer to the CapabilitiesFabricClient
  auto node = std::make_shared<perception::PerceptionServer>();
  
  // Initialize the node
  node->initialize();  // Call initialize after construction

  // Spin the node to process callbacks
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}