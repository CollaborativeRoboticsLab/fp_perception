#include <audo_driver/driver_node.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AudioDriver>());
  rclcpp::shutdown();
  return 0;
}
