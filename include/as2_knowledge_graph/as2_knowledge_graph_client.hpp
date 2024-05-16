#ifndef Service_Client_hpp
#define Service_Client_hpp

#include "as2_core/node.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_knowledge_graph_msgs/srv/create_edge.hpp"
#include "as2_knowledge_graph_msgs/srv/create_node.hpp"
#include "as2_knowledge_graph_msgs/srv/read_graph.hpp"
#include "as2_knowledge_graph_msgs/srv/read_edge_graph.hpp"
#include "as2_knowledge_graph_msgs/srv/read_property.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class KnowledgeGraphClient : public as2::Node
{
public:
  KnowledgeGraphClient()
  : as2::Node("as2_knowledge_graph_client")
  {
    client_create_node_ = this->create_client<as2_knowledge_graph_msgs::srv::CreateNode>(
      "create_node");
    client_remove_node_ = this->create_client<as2_knowledge_graph_msgs::srv::CreateNode>(
      "remove_node");
    client_create_edge_ = this->create_client<as2_knowledge_graph_msgs::srv::CreateEdge>(
      "create_edge");
    client_remove_edge_ = this->create_client<as2_knowledge_graph_msgs::srv::CreateEdge>(
      "remove_edge");
    client_add_property_node_ = this->create_client<as2_knowledge_graph_msgs::srv::CreateNode>(
      "add_property_node_");
    client_add_property_edge_ = this->create_client<as2_knowledge_graph_msgs::srv::CreateEdge>(
      "add_property_edge");
    client_read_graph_ =
      this->create_client<as2_knowledge_graph_msgs::srv::ReadGraph>("read_graph");
    client_read_node_graph_ = this->create_client<as2_knowledge_graph_msgs::srv::ReadGraph>(
      "read_node_graph");
    client_read_edge_class_graph_ =
      this->create_client<as2_knowledge_graph_msgs::srv::ReadEdgeGraph>(
      "read_edge_class_graph");
    client_read_edge_source_target_graph_ =
      this->create_client<as2_knowledge_graph_msgs::srv::ReadEdgeGraph>(
      "read_edge_source_target_graph");
    service_read_node_property_graph_ =
      this->create_client<as2_knowledge_graph_msgs::srv::ReadProperty>(
      "read_node_property_graph");
    service_read_edge_property_graph_ =
      this->create_client<as2_knowledge_graph_msgs::srv::ReadProperty>(
      "read_edge_property_graph");

    timer_ = this->create_wall_timer(20ms, std::bind(&KnowledgeGraphClient::timerCallback, this));
  }

  bool is_service_done() const
  {
    return this->service_done_;
  }

public:
  bool service_done_ = false;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::CreateNode>::SharedPtr client_create_node_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::CreateNode>::SharedPtr client_remove_node_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::CreateEdge>::SharedPtr client_create_edge_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::CreateEdge>::SharedPtr client_remove_edge_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::CreateNode>::SharedPtr client_add_property_node_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::CreateEdge>::SharedPtr client_add_property_edge_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::ReadGraph>::SharedPtr client_read_graph_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::ReadGraph>::SharedPtr client_read_node_graph_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::ReadEdgeGraph>::SharedPtr
    client_read_edge_class_graph_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::ReadEdgeGraph>::SharedPtr
    client_read_edge_source_target_graph_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::ReadProperty>::SharedPtr
    service_read_node_property_graph_;
  rclcpp::Client<as2_knowledge_graph_msgs::srv::ReadProperty>::SharedPtr
    service_read_edge_property_graph_;

  void timerCallback();
  bool createNode(const knowledge_graph_msgs::msg::Node & client_);
  bool createEdge(const knowledge_graph_msgs::msg::Edge & client_);
  bool removeNode(const knowledge_graph_msgs::msg::Node & client_);
  bool removeEdge(const knowledge_graph_msgs::msg::Edge & client_);
  bool addPropertyNode(
    const knowledge_graph_msgs::msg::Node & client_);
  bool addPropertyEdge(const knowledge_graph_msgs::msg::Edge & client_);
  bool readGraph(const std::string & node_name_client_);
  bool readGraph();
  bool readNodeGraph(const std::string & node_class_client_);
  bool readEdgeClassGraph(
    const knowledge_graph_msgs::msg::Node & source_client_,
    const knowledge_graph_msgs::msg::Node & target_client_);
  bool readEdgeSourceTargetGraph(const std::string & edge_class_client_);

  bool readNodePropertyGraph(const std::string & node_name_client_);
  bool readEdgePropertyGraph(const std::string & edge_class_client_);
};


#endif
