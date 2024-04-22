#include "as2_knowledge_graph_service.hpp"
#include "as2_knowledge_graph_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <stdexcept>

#include "gtest/gtest.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode()
  : rclcpp::Node("add_node")
  {
    service = this->create_service<as2_knowledge_graph_msgs::srv::CreateNode>(
      "create_node", std::bind(
        &ServerNode::add, this, _1,
        _2));

    RCLCPP_INFO(this->get_logger(), "Ready ");
  }

  void add(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request,
    std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response)
  {
    response->resultado = true;
    RCLCPP_INFO(
      this->get_logger(), "Incoming request\n node name: %s node class: %s ",
      request->node.node_name.c_str(), request->node.node_class.c_str());
    RCLCPP_INFO(this->get_logger(), "sending back response: %d ", response->resultado);
  }

public:
  rclcpp::Service<as2_knowledge_graph_msgs::srv::CreateNode>::SharedPtr service;
};

class ClientNode : public rclcpp::Node
{
public:
  ClientNode()
  : rclcpp::Node("create_node")
  {
    client = this->create_client<as2_knowledge_graph_msgs::srv::CreateNode>("create_node");

    RCLCPP_INFO(this->get_logger(), "Ready to send request.");
  }

public:
  rclcpp::Client<as2_knowledge_graph_msgs::srv::CreateNode>::SharedPtr client;
};


/* Test fixture */
class MyTest : public testing::Test
{
protected:
  std::shared_ptr<ServerNode> server_node;
  std::shared_ptr<ClientNode> client_node;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp()
  {
    server_node = std::make_shared<ServerNode>();
    client_node = std::make_shared<ClientNode>();

    executor.add_node(server_node);
    executor.add_node(client_node);
  }

  void TearDown()
  {
    executor.cancel();
    executor.remove_node(client_node);
    executor.remove_node(server_node);
    client_node.reset();
    server_node.reset();
  }
};


/* Test cases */
TEST_F(MyTest, CallService) {
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
  request->node.node_name = "pedro";
  request->node.node_class = "persona";

  auto result_future = client_node->client->async_send_request(request);
  executor.spin_some();

  auto result = executor.spin_until_future_complete(result_future);
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  //ASSERT_EQ(result_future.get()->sum, 5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
