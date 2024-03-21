#include "as2_knowledge_graph_service.hpp"
#include "as2_knowledge_graph_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <stdexcept>

#include "gtest/gtest.h"

//global items
std::atomic<bool> running=true;
std::shared_ptr<NewNode> server_node;
std::shared_ptr<ServiceClient> client_node;

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
    executor.cancel();
    executor.remove_node(server_node);
};



TEST(MyTest, serviceCall){
    
   auto thread = std::thread(spin_node,server_node);
   std::this_thread::sleep_for(10s);

    // //Response received
    running = false;
    thread.join();
    

}
int main(int argc, char * argv[])
{ 

  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv); 

  //add node before de init
  server_node = std::make_shared<NewNode>();
  auto result = RUN_ALL_TESTS();
  server_node.reset();
  rclcpp::shutdown();
  return result;
}