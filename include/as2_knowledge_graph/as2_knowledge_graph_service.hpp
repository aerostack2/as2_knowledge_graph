#ifndef New_Node_hpp
#define New_Node_hpp

#include <memory>
#include <optional>
#include <rclcpp/node.hpp>
#include <string>
#include <vector>

#include "as2_knowledge_graph_msgs/srv/create_node.hpp"
#include "as2_knowledge_graph_msgs/srv/create_edge.hpp"
#include "as2_knowledge_graph_msgs/srv/read_graph.hpp"
#include "as2_knowledge_graph_msgs/srv/read_edge_graph.hpp"
#include "knowledge_graph/graph_utils.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "knowledge_graph_msgs/msg/content.hpp"
#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class KnowledgeGraphServer : public rclcpp::Node
{
public:
  KnowledgeGraphServer()
  : rclcpp::Node("as2_knowledge_graph_server")
  {
    service_create_node_ = this->create_service<as2_knowledge_graph_msgs::srv::CreateNode>(
      "create_node", std::bind(&KnowledgeGraphServer::createNode, this, _1, _2));
    service_create_edge_ = this->create_service<as2_knowledge_graph_msgs::srv::CreateEdge>(
      "create_edge", std::bind(&KnowledgeGraphServer::createEdge, this, _1, _2));

    service_remove_node_ = this->create_service<as2_knowledge_graph_msgs::srv::CreateNode>(
      "remove_node", std::bind(&KnowledgeGraphServer::removeNode, this, _1, _2));
    service_remove_edge_ = this->create_service<as2_knowledge_graph_msgs::srv::CreateEdge>(
      "remove_edge", std::bind(&KnowledgeGraphServer::removeEdge, this, _1, _2));

    add_property_node_ = this->create_service<as2_knowledge_graph_msgs::srv::CreateNode>(
      "add_property_node", std::bind(&KnowledgeGraphServer::addPropertyNode, this, _1, _2));
    add_property_edge_ = this->create_service<as2_knowledge_graph_msgs::srv::CreateEdge>(
      "add_property_edge", std::bind(&KnowledgeGraphServer::addPropertyEdge, this, _1, _2));

    service_read_graph_ = this->create_service<as2_knowledge_graph_msgs::srv::ReadGraph>(
      "read_graph", std::bind(&KnowledgeGraphServer::readGraph, this, _1, _2));
    service_read_node_graph_ = this->create_service<as2_knowledge_graph_msgs::srv::ReadGraph>(
      "read_node_graph", std::bind(&KnowledgeGraphServer::readNodeGraph, this, _1, _2));
    service_read_edge_class_graph_ =
      this->create_service<as2_knowledge_graph_msgs::srv::ReadEdgeGraph>(
      "read_edge_class_graph", std::bind(&KnowledgeGraphServer::readEdgeClassGraph, this, _1, _2));
    service_read_edge_source_target_graph_ =
      this->create_service<as2_knowledge_graph_msgs::srv::ReadEdgeGraph>(
      "read_edge_source_target_graph",
      std::bind(&KnowledgeGraphServer::readEdgeSourceTargetGraph, this, _1, _2));
    service_read_node_property_graph_ =
      this->create_service<as2_knowledge_graph_msgs::srv::ReadGraph>(
      "read_node_property_graph",
      std::bind(&KnowledgeGraphServer::readNodePropertyGraph, this, _1, _2));

    timer_ = this->create_wall_timer(20ms, std::bind(&KnowledgeGraphServer::timerCallback, this));

    static auto setup = this->create_wall_timer(
      20ms, [this]() {
        if (!knowledge_graph_ptr_) {
          knowledge_graph_ptr_ = std::make_shared<knowledge_graph::KnowledgeGraph>(
            static_cast<std::shared_ptr<rclcpp::Node>>(
              this->shared_from_this()));
        }
      });
  }
  ~KnowledgeGraphServer()
  {

    service_create_node_.reset();
    service_create_edge_.reset();
    service_remove_node_.reset();
    service_remove_edge_.reset();
    add_property_node_.reset();
    add_property_edge_.reset();
    service_read_graph_.reset();
    service_read_node_graph_.reset();
    service_read_edge_class_graph_.reset();
    service_read_edge_source_target_graph_.reset();
    service_read_node_property_graph_.reset();
    knowledge_graph_ptr_.reset();
    timer_.reset();
  }

  std::shared_ptr<knowledge_graph::KnowledgeGraph> getKnowledgeGraph()
  {
    if (!knowledge_graph_ptr_) {
      knowledge_graph_ptr_ = std::make_shared<knowledge_graph::KnowledgeGraph>(
        static_cast<std::shared_ptr<rclcpp::Node>>(this->shared_from_this()));
    }
    return knowledge_graph_ptr_;
  }

protected:
  std::shared_ptr<knowledge_graph::KnowledgeGraph> knowledge_graph_ptr_;


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::CreateNode>::SharedPtr service_create_node_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::CreateEdge>::SharedPtr add_property_edge_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::CreateEdge>::SharedPtr service_create_edge_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::CreateNode>::SharedPtr service_remove_node_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::CreateEdge>::SharedPtr service_remove_edge_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::CreateNode>::SharedPtr add_property_node_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::ReadGraph>::SharedPtr service_read_graph_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::ReadGraph>::SharedPtr service_read_node_graph_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::ReadEdgeGraph>::SharedPtr
    service_read_edge_class_graph_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::ReadEdgeGraph>::SharedPtr
    service_read_edge_source_target_graph_;
  rclcpp::Service<as2_knowledge_graph_msgs::srv::ReadGraph>::SharedPtr
    service_read_node_property_graph_;
  size_t count_;

  std::optional<knowledge_graph_msgs::msg::Node> get_node_from_class(
    const std::string node_class);


  void createNode(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response);
  void createEdge(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response);
  void removeNode(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response);
  void removeEdge(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response);
  void addPropertyNode(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response);
  void addPropertyEdge(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response);
  void timerCallback();
  void readGraph(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response);
  void readNodeGraph(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response);
  void readEdgeClassGraph(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Response> response);
  void readEdgeSourceTargetGraph(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Response> response);
  void readNodePropertyGraph(
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
    const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response);
};

#endif
