#include "as2_knowledge_graph_service.hpp"
#include "as2_knowledge_graph_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <stdexcept>

#include "gtest/gtest.h"
using namespace std::chrono_literals;

/* Test fixture */
class MyTest : public testing::Test {
public:
    //Construtor
    MyTest() {
        server_node = std::make_shared<NewNode>();
        client_node = std::make_shared<ServiceClient>();

        executor.add_node(server_node);
        executor.add_node(client_node);
    }

    ~MyTest() {
        executor.cancel();
        executor.remove_node(client_node);
        executor.remove_node(server_node);
        client_node.reset();
        server_node.reset();
    }  
    
    std::shared_ptr<NewNode> server_node;
    std::shared_ptr<ServiceClient> client_node;
    rclcpp::executors::SingleThreadedExecutor executor;
};

knowledge_graph_msgs::msg::Node get_name_test(){
    knowledge_graph_msgs::msg::Node ret_node;
    ret_node.node_name="Paco";
    ret_node.node_class="Persona";
    return ret_node;
 };

TEST_F(MyTest, CallService) {
    std::shared_ptr<MyTest> my_test;
    knowledge_graph_msgs::msg::Node name1 = get_name_test();
    my_test->executor.spin_some();

    auto result_future = my_test->client_node->createNode(name1);
}