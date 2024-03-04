#include "NewNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Service<my_graph::srv::CreateNode>::SharedPtr client_create_node;
  rclcpp::spin(std::make_shared<NewNode>());
  rclcpp::shutdown();
  return 0;
}