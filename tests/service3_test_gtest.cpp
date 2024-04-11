#include "as2_knowledge_graph_service.hpp"
#include "as2_knowledge_graph_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <stdexcept>

#include "gtest/gtest.h"

//global items
std::atomic<bool> running=true;
std::shared_ptr<NewNode> server_node;
std::shared_ptr<ServiceClient> client_node;

//Example 1
knowledge_graph_msgs::msg::Node get_name_test(){
    knowledge_graph_msgs::msg::Node ret_node;
    ret_node.node_name="Paco";
    ret_node.node_class="Persona";
    return ret_node;
 };

//Example 2
knowledge_graph_msgs::msg::Node get_name_test2(){
    knowledge_graph_msgs::msg::Node ret_node;
    ret_node.node_name="Sara";
    ret_node.node_class="Persona";
    return ret_node;
 };


//Spin the service
void spin_node(std::shared_ptr<NewNode> server_node){
    rclcpp::Rate rate(10);
    rclcpp::executors::SingleThreadedExecutor executor;
    std::cout << "node spin"<< std::endl;
    executor.add_node(server_node);
    while(rclcpp::ok() && running){
        executor.spin_some();
        rate.sleep();
    }
    //delete de executor
    executor.cancel();
    executor.remove_node(server_node);
    
    
};


// // //Create 1 node
// TEST(MyTest, serviceCall){
//     running = true;
//    auto thread = std::thread(spin_node,server_node);
//     client_node = std::make_shared<ServiceClient>();

//     //Add node name
//     bool flag;
//     flag = client_node->createNode(get_name_test());
//     ASSERT_EQ(flag,true);
//     ASSERT_EQ(flag,false);

//    //stop during 10s
//    std::this_thread::sleep_for(10s);

//     //Response received
//     client_node.reset();
//     running = false;
//     thread.join();
    // std::cout<< "Server killed" << std::endl;

    // std::this_thread::sleep_for(1s);
    // std::cout<< "exiting" << std::endl;
    

    

// }


TEST(MyTest, twonodes){
    running = true;
   auto thread = std::thread(spin_node,server_node);
    client_node = std::make_shared<ServiceClient>();

    // Add node name
    bool flag;
    //bool flag_1, flag_2;
    flag = client_node->createNode(get_name_test());
    // flag_1 = client_node->createNode(get_name_test());
    // flag_2 = client_node->createNode(get_name_test2());
    // ASSERT_EQ(flag,true);
    // ASSERT_EQ(flag,false);
    // client_node->shutdown();

   //stop during 10s
   std::this_thread::sleep_for(1s);

    //Response received
    
    running = false;
    thread.join();
    std::cout<< "Server killed" << std::endl;

    std::this_thread::sleep_for(1s);
    std::cout<< "exiting" << std::endl;
    

}

int main(int argc, char * argv[])
{ 

  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv); 

  //add node before de init
  server_node = std::make_shared<NewNode>();
  
  
  auto result = RUN_ALL_TESTS();
  
  //delete the node
    client_node.reset();
    server_node.reset();
  //server_node->shutdown();

  rclcpp::shutdown();
  return result;
}