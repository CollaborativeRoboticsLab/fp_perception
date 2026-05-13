#include <perception/perception_server.hpp>

#include <cstdio>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<perception::PerceptionServer>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::fprintf(stderr, "PerceptionServer startup failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
}