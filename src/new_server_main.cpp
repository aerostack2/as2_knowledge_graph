#include "new_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NewNode>());
  rclcpp::shutdown();
  return 0;
}