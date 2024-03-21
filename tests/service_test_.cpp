#include "as2_knowledge_graph_service.hpp"
#include "as2_knowledge_graph_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <stdexcept>

#include "gtest/gtest.h"

using namespace std::chrono_literals;

class NodeTest : public ServiceClient
{
public:

bool testCreate(const knowledge_graph_msgs::msg::Node &test){
    return createNode(test);
}

};

knowledge_graph_msgs::msg::Node get_name_test(){
    knowledge_graph_msgs::msg::Node ret_node;
    ret_node.node_name="Paco";
    ret_node.node_class="Persona";
    return ret_node;
 };

/* Test fixture */
class MyTest : public testing::Test {
public:
  std::shared_ptr<NewNode> server_node;
  std::shared_ptr<NodeTest> client_node;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp() {
    server_node = std::make_shared<NewNode>();
    client_node = std::make_shared<NodeTest>();

    executor.add_node(server_node);
    executor.add_node(client_node);
  }

  void TearDown() {
    executor.cancel();
    executor.remove_node(client_node);
    executor.remove_node(server_node);
    client_node.reset();
    server_node.reset();
  }
};
TEST(service_test_gtest, get_name){
  std::shared_ptr<MyTest> my_test;
  knowledge_graph_msgs::msg::Node name1 = get_name_test();
    //auto start = node_test->now();
  auto result_future = my_test->client_node->testCreate(name1);
  my_test->executor.spin_some();
  printf("%d",result_future);
  
  //Esto da error
  // auto result = my_test->executor.spin_until_future_complete(result_future);
  // ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);



  // rclcpp::executors::SingleThreadedExecutor executor;
  //  std::shared_ptr<NewNode> server_node;
  //  server_node = std::make_shared<NewNode>();
  //  executor.add_node(server_node);
  // knowledge_graph_msgs::msg::Node name1 = get_name_test();
  //  auto res1 = node_test->testCreate(name1); 

  //     executor.spin_some();

  //     rate.sleep();
  //   executor.cancel();
  //   executor.remove_node(server_node);
  //   executor.remove_node(node_test);
  //   client_node.reset();
  //   node_test.reset();
  //     // while (rclcpp::ok() && (node_test->now() - start) < 1s) { // }
  //   ASSERT_EQ(res1,true);


//    ASSERT_TRUE(res1);

}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}