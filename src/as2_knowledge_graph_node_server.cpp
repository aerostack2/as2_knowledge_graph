#include "as2_knowledge_graph_service.hpp"
#include <memory>
#include <rclcpp/node.hpp>
#include "utils/as2_knowledge_graph_graph_utils.hpp"

void KnowledgeGraphServer::timerCallback()
{
}

void KnowledgeGraphServer::createNode(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response)
{

  RCLCPP_INFO(this->get_logger(), "service create node");
  knowledge_graph_msgs::msg::Node my_node;
  request_name_received = true;
  my_node.node_name = request->node.node_name;
  my_node.node_class = request->node.node_class;

  RCLCPP_INFO(this->get_logger(), "successfully built node");
  if (this->knowledge_graph_ptr_->update_node(my_node, 1) == true) {
    RCLCPP_INFO(this->get_logger(), " successfully update");
    response->resultado = true;
  }

}

void KnowledgeGraphServer::createEdge(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response)
{
  knowledge_graph_msgs::msg::Edge my_edge;
  my_edge.edge_class = request->edge.edge_class;
  my_edge.source_node = request->edge.source_node;
  my_edge.target_node = request->edge.target_node;

  request_edge_received = true;
  response->resultado = request_edge_received;
  RCLCPP_INFO(this->get_logger(), "successfullly received edge");
  if (this->knowledge_graph_ptr_->update_edge(my_edge, 1) == true) {
    RCLCPP_INFO(this->get_logger(), "successfully update");
    this->knowledge_graph_ptr_->get_edges(my_edge.source_node, my_edge.target_node);
    this->knowledge_graph_ptr_->get_edges(my_edge.edge_class);
    this->knowledge_graph_ptr_->get_out_edges(my_edge.source_node);
    this->knowledge_graph_ptr_->get_in_edges(my_edge.target_node);
  }
}

void KnowledgeGraphServer::removeNode(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response)
{
  knowledge_graph_msgs::msg::Node my_node;
  request_remove_node_received = true;
  my_node.node_name = request->node.node_name;
  my_node.node_class = request->node.node_class;
  response->resultado = request_remove_node_received;
  if (this->knowledge_graph_ptr_->remove_node(my_node.node_name) == true) {
    RCLCPP_INFO(this->get_logger(), " successfully remove %s", my_node.node_name.c_str());
  }
}

void KnowledgeGraphServer::removeEdge(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response)
{
  knowledge_graph_msgs::msg::Edge my_edge;
  my_edge.edge_class = request->edge.edge_class;
  my_edge.source_node = request->edge.source_node;
  my_edge.target_node = request->edge.target_node;request_remove_edge_received = true;
  response->resultado = request_remove_edge_received;
  if (this->knowledge_graph_ptr_->remove_edge(my_edge, 1) == true) {
    RCLCPP_INFO(
      this->get_logger(), "%s, %s, %s ",
      my_edge.edge_class.c_str(), my_edge.source_node.c_str(), my_edge.target_node.c_str());
  }
}


void KnowledgeGraphServer::addPropertyNode(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Adding a node property");
  knowledge_graph_msgs::msg::Node my_node;
  my_node = request->node;
  for (auto & property: my_node.properties) {
    RCLCPP_INFO(this->get_logger(), "the key: %s", property.key.c_str());
    RCLCPP_INFO(
      this->get_logger(), "the content: %s",
      knowledge_graph::to_string(property.value).c_str());
    response->resultado = 1;

  }
}

void KnowledgeGraphServer::addPropertyEdge(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response)
{
  knowledge_graph_msgs::msg::Edge my_edge;
  my_edge = request->edge;
  for (auto & property:my_edge.properties) {
    if (knowledge_graph::add_property(my_edge, property.key, property.value) == true) {
      response->resultado = 1;
    }
  }
  RCLCPP_INFO(this->get_logger(), "Edge property service");
}