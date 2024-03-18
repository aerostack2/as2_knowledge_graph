#include "new_node.hpp"
#include "service_client.hpp"
#include <stdexcept>

#include "gtest/gtest.h"

using namespace std::chrono_literals;

class NodeTest : public ServiceClient
{
public:

bool testCreate(const knowledge_graph_msgs::msg::Node &test){
    createNode(test);
    return true; 
}

};

knowledge_graph_msgs::msg::Node get_name_test(){
    knowledge_graph_msgs::msg::Node ret_node;
    ret_node.node_name="Paco";
    ret_node.node_class="Persona";
    return ret_node;
 };


TEST(service_test_gtest, get_name){
    auto knowledge_graph_server = std::make_shared<NewNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(knowledge_graph_server);
     

    auto node_test = std::make_shared<NodeTest>();
    knowledge_graph_msgs::msg::Node name1 = get_name_test();

     rclcpp::Rate rate(30);
     // Test for scan test #1
    auto start = node_test->now();
    

    bool res = false;
    while (rclcpp::ok() && (node_test->now() - start) < 1s) {
        executor.spin_some();
        auto res1 = node_test->testCreate(name1);
        res=res1;
    rate.sleep();
  }
  ASSERT_EQ(res,true);

auto res1 = node_test->testCreate(name1);
   ASSERT_TRUE(res1);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}