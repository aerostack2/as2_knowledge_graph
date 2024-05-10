#include "as2_knowledge_graph_service.hpp"
#include "as2_knowledge_graph_client.hpp"
#include "utils/as2_knowledge_graph_graph_utils.hpp"
#include "rclcpp/rclcpp.hpp"


#include "gtest/gtest.h"

// global items
std::atomic<bool> running = true;
std::shared_ptr<KnowledgeGraphServer> server_node;
std::shared_ptr<KnowledgeGraphClient> client_node;

// Spin the service
void spin_node(std::shared_ptr<KnowledgeGraphServer> server_node1)
{
  rclcpp::Rate rate(10);
  rclcpp::executors::SingleThreadedExecutor executor;
  std::cout << "node spin" << std::endl;
  executor.add_node(server_node1);
  while (rclcpp::ok() && running) {
    executor.spin_some();
    rate.sleep();
  }
// Delete de executor
  executor.cancel();
  executor.remove_node(server_node1);
}


// Node graph1
knowledge_graph_msgs::msg::Node get_name_test1()
{
  knowledge_graph_msgs::msg::Node ret_node;
  ret_node.node_name = "Paco";
  ret_node.node_class = "Persona";
  return ret_node;
}

// Node graph2
knowledge_graph_msgs::msg::Node get_name_test2()
{
  knowledge_graph_msgs::msg::Node ret_node;
  ret_node.node_name = "Sara";
  ret_node.node_class = "Persona";
  return ret_node;
}

// Node graph3
knowledge_graph_msgs::msg::Node get_name_test3()
{
  knowledge_graph_msgs::msg::Node ret_node;
  ret_node.node_name = "Ana";
  ret_node.node_class = "Persona";
  return ret_node;
}

// Edge graph1
knowledge_graph_msgs::msg::Edge get_edge_test1(
  knowledge_graph_msgs::msg::Node node1,
  knowledge_graph_msgs::msg::Node node2)
{
  knowledge_graph_msgs::msg::Edge ret_edge;
  ret_edge.edge_class = "son familia";
  ret_edge.source_node = node1.node_name;
  ret_edge.target_node = node2.node_name;
  return ret_edge;
}

// Add a node property1
knowledge_graph_msgs::msg::Node get_node_property1(knowledge_graph_msgs::msg::Node node)
{
  knowledge_graph_msgs::msg::Node ret_node;
  ret_node = node;
  knowledge_graph::add_property(ret_node, "apellido", std::string("Gonzalez"));
  knowledge_graph::add_property(ret_node, "edad", 21);
  return ret_node;
}

// Add an edge property
knowledge_graph_msgs::msg::Edge get_edge_property1(knowledge_graph_msgs::msg::Edge edge)
{
  knowledge_graph_msgs::msg::Edge ret_edge;
  ret_edge = edge;
  knowledge_graph::add_property(ret_edge, "relacion", std::string("padre e hijo"));
  return ret_edge;
}

TEST(MyTest, oneNode) {
  running = true;
  auto thread = std::thread(spin_node, server_node);
  client_node = std::make_shared<KnowledgeGraphClient>();

  bool flag_1, flag_2, flag_3, flag_4, flag_5, flag_6, flag_7, flag_8, flag_9, flag_10, flag_11,
    flag_12, flag_13, flag_14;

  flag_1 = client_node->createNode(get_name_test1());
  EXPECT_EQ(flag_1, true);
  std::this_thread::sleep_for(1s);


  // Create another node graph
  flag_2 = client_node->createNode(get_name_test2());
  EXPECT_EQ(flag_2, true);
  std::this_thread::sleep_for(1s);

  // Create an edge between de two nodes
  flag_5 = client_node->createEdge(get_edge_test1(get_name_test1(), get_name_test2()));
  EXPECT_EQ(flag_5, true);
  std::this_thread::sleep_for(1s);

  // Read whole graph
  flag_3 = client_node->readGraph();
  EXPECT_EQ(flag_3, true);
  std::this_thread::sleep_for(1s);

  // Read the edge source and target
  flag_6 = client_node->readEdgeSourceTargetGraph("son familia");
  EXPECT_EQ(flag_6, true);
  std::this_thread::sleep_for(1s);

  // Read the edge class
  flag_7 = client_node->readEdgeClassGraph(get_name_test1(), get_name_test2());
  EXPECT_EQ(flag_7, true);
  std::this_thread::sleep_for(1s);

  // Add a property to node1
  knowledge_graph_msgs::msg::Node node_aux;
  node_aux = get_node_property1(get_name_test1());
  flag_8 = client_node->addPropertyNode(node_aux);
  EXPECT_EQ(flag_8, true);
  std::this_thread::sleep_for(1s);

  // Add a property to the edge1
  flag_14 =
    client_node->addPropertyEdge(
    get_edge_property1(
      get_edge_test1(
        get_name_test1(),
        get_name_test2())));
  EXPECT_EQ(flag_14, true);
  std::this_thread::sleep_for(1s);

  // Read nodes of a specific node class
  flag_13 = client_node->readNodeGraph("Persona");
  EXPECT_EQ(flag_13, true);
  std::this_thread::sleep_for(1s);

  // Read only the node asked
  flag_4 = client_node->readGraph("Paco");
  EXPECT_EQ(flag_4, true);
  std::this_thread::sleep_for(1s);

  // Remove the edge
  flag_9 = client_node->removeEdge(get_edge_test1(get_name_test1(), get_name_test2()));
  EXPECT_EQ(flag_9, true);
  std::this_thread::sleep_for(1s);

  // Remove the nodes
  flag_10 = client_node->removeNode(get_name_test1());
  EXPECT_EQ(flag_10, true);
  std::this_thread::sleep_for(1s);
  flag_11 = client_node->removeNode(get_name_test2());
  EXPECT_EQ(flag_11, true);
  std::this_thread::sleep_for(1s);
  flag_12 = client_node->readGraph();
  EXPECT_EQ(flag_12, true);
  std::this_thread::sleep_for(1s);


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

  // Add node before de init
  server_node = std::make_shared<KnowledgeGraphServer>();
  auto result = RUN_ALL_TESTS();
  std::cout << "delete server and client" << std::endl;

  // Delete the server and the client
  client_node.reset();
  server_node.reset();
  rclcpp::shutdown();
  return result;
}
