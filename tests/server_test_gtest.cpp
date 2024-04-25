#include "as2_knowledge_graph_service.hpp"
#include "as2_knowledge_graph_client.hpp"
#include "utils/as2_knowledge_graph_graph_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

//global items
std::atomic<bool> running = true;
std::shared_ptr<KnowledgeGraphServer> server_node;
std::shared_ptr<KnowledgeGraphClient> client_node;


//Example node graph1
knowledge_graph_msgs::msg::Node get_name_test()
{
  knowledge_graph_msgs::msg::Node ret_node;
  ret_node.node_name = "Paco";
  ret_node.node_class = "Persona";
  return ret_node;
}

//Example node graph2
knowledge_graph_msgs::msg::Node get_name_test2()
{
  knowledge_graph_msgs::msg::Node ret_node;
  ret_node.node_name = "Sara";
  ret_node.node_class = "Persona";
  return ret_node;
}

// Example node graph3
knowledge_graph_msgs::msg::Node get_name_test3()
{
  knowledge_graph_msgs::msg::Node ret_node;
  ret_node.node_name = "Ana";
  ret_node.node_class = "Persona";
  return ret_node;
}

// Example edge graph
knowledge_graph_msgs::msg::Edge get_edge_test1()
{
  knowledge_graph_msgs::msg::Edge ret_edge;
  ret_edge.edge_class = "sees";
  ret_edge.source_node = "Sara";
  ret_edge.target_node = "Ana";
  return ret_edge;
}

// Example property node
knowledge_graph_msgs::msg::Node get_node_property()
{
  knowledge_graph_msgs::msg::Node ret_node;
  ret_node.node_name = "Julia";
  ret_node.node_class = "Persona";


  knowledge_graph::add_property(ret_node.properties, "apellido", std::string("gonzalez"));
  knowledge_graph::add_property(ret_node.properties, "edad", 21);
  return ret_node;
}

// Access to my graph nodes
void readMyGraph()
{
  std::vector<std::string> node_names;
  std::vector<knowledge_graph_msgs::msg::Edge> edge_names;
  node_names = server_node->getKnowledgeGraph()->get_node_names();
  if (server_node->getKnowledgeGraph()->get_node_names().empty()) {
    std::cout << "there graph is empty" << std::endl;
  } else {
    std::cout << "Inside my graph there are the nodes:" << std::endl;
    for (size_t i = 0; i < server_node->getKnowledgeGraph()->get_node_names().size(); i++) {

      std::cout << node_names.at(i) << std::endl;
    }
  }
}

void readMyProperties(const knowledge_graph_msgs::msg::Node & name_node)
{
  knowledge_graph_msgs::msg::Node node;
  node = name_node;
  for (auto & node_name : server_node->getKnowledgeGraph()->get_node_names()) {
    if (node.node_name == node_name) {
      std::cout << "The node " << node_name << "has the properties: " << std::endl;

      for (auto & prop : node.properties) {
        std::cout <<
          knowledge_graph::to_string(prop.value).c_str() << " of type " <<
          prop.key <<
          std::endl;
      }
    } else {
      std::cout << "This node do not exists" << std::endl;
    }

  }
}


//Spin the service
void spin_node(std::shared_ptr<KnowledgeGraphServer> server_node)
{
  rclcpp::Rate rate(10);
  rclcpp::executors::SingleThreadedExecutor executor;
  std::cout << "node spin" << std::endl;
  executor.add_node(server_node);
  while (rclcpp::ok() && running) {
    executor.spin_some();
    rate.sleep();
  }
  //delete de executor
  executor.cancel();
  executor.remove_node(server_node);


}


//Create 1 node
TEST(MyTest, oneNode) {
  running = true;
  auto thread = std::thread(spin_node, server_node);
  client_node = std::make_shared<KnowledgeGraphClient>();

  //Add node name
  bool flag;
  flag = client_node->createNode(get_name_test());
  ASSERT_EQ(flag, true);

  //ASSERT_EQ(flag,false);

  //Show service

  //stop during 1s
  std::this_thread::sleep_for(1s);

  //Response received
  client_node.reset();
  running = false;
  thread.join();
  std::cout << "Server killed" << std::endl;

  std::this_thread::sleep_for(1s);
  std::cout << "exiting" << std::endl;
}

//Create an edge
TEST(MyTest, twonodes) {
  running = true;
  auto thread = std::thread(spin_node, server_node);
  client_node = std::make_shared<KnowledgeGraphClient>();
  // client_edge = std::make_shared<KnowledgeGraphClient>();

  // Add node name
  bool flag_2, flag_3, flag_edge_1;

  flag_2 = client_node->createNode(get_name_test2());
  flag_3 = client_node->createNode(get_name_test3());
  ASSERT_EQ(flag_2, true);
  ASSERT_EQ(flag_3, true);

  std::this_thread::sleep_for(1s);

  readMyGraph();

  flag_edge_1 = client_node->createEdge(get_edge_test1());
  ASSERT_EQ(flag_edge_1, true);

  //stop during 1s
  std::this_thread::sleep_for(1s);

  std::vector<knowledge_graph_msgs::msg::Edge> edge_name;
  edge_name = server_node->getKnowledgeGraph()->get_edges("Sara", "Ana");
  std::cout << edge_name.at(0).source_node << std::endl;
  std::cout << edge_name.at(0).edge_class << std::endl;
  std::cout << edge_name.at(0).target_node << std::endl;

  if (true == client_node->removeEdge(get_edge_test1())) {
    std::cout << "successfully remove edge" << std::endl;
  }


  //Response received

  running = false;
  thread.join();
  std::cout << "Server killed" << std::endl;

  std::this_thread::sleep_for(1s);
  std::cout << "exiting" << std::endl;
}

//Delete a node
TEST(MyTest, deleteNode) {
  running = true;
  auto thread = std::thread(spin_node, server_node);
  client_node = std::make_shared<KnowledgeGraphClient>();
  bool flag_delete, flag_delete_1;
  flag_delete = client_node->createNode(get_name_test());
  ASSERT_EQ(flag_delete, true);
  readMyGraph();
  flag_delete_1 = client_node->removeNode(get_name_test());
  ASSERT_EQ(flag_delete_1, true);
  readMyGraph();
  //Response received

  running = false;
  thread.join();
  std::cout << "Server killed" << std::endl;
  std::this_thread::sleep_for(1s);
  std::cout << "exiting" << std::endl;

}

// Test create a property
TEST(MyTest, oneNodeProperty) {
  running = true;
  auto thread = std::thread(spin_node, server_node);
  client_node = std::make_shared<KnowledgeGraphClient>();


  //Add node property

  bool flag, flag_name;
  auto node = get_node_property();
  flag_name = client_node->createNode(node);
  flag = client_node->addPropertyNode(node);
  readMyProperties(node);
  ASSERT_EQ(flag, true);


  //stop during 1s
  std::this_thread::sleep_for(1s);

  //Response received
  client_node.reset();
  running = false;
  thread.join();
  std::cout << "Server killed" << std::endl;

  std::this_thread::sleep_for(1s);
  std::cout << "exiting" << std::endl;
}


int main(int argc, char * argv[])
{

  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  //add node before de init
  server_node = std::make_shared<KnowledgeGraphServer>();


  auto result = RUN_ALL_TESTS();
  readMyGraph();

  std::cout << "delete nodes" << std::endl;
  //delete the node
  client_node.reset();
  // client_edge.reset();
  server_node.reset();
  //server_node->shutdown();

  rclcpp::shutdown();
  return result;
}
