#include "lekiwi_node/base_controller.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lekiwi_node::BaseController>());
  rclcpp::shutdown();
  return 0;
}
