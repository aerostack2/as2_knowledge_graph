#include "as2_knowledge_graph_service.hpp"
#include <memory>
#include <rclcpp/node.hpp>
#include "utils/as2_knowledge_graph_graph_utils.hpp"

void KnowledgeGraphServer::timerCallback()
{
}
std::optional<knowledge_graph_msgs::msg::Node> KnowledgeGraphServer::get_node_from_class(
  const std::string node_class)
{
  for (auto it:KnowledgeGraphServer::getKnowledgeGraph()->get_nodes()) {
    knowledge_graph_msgs::msg::Node ret;
    ret.node_class = it.node_class;
    if (ret.node_class == node_class) {
      return ret = it;
    }
  }
  return {};
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
void KnowledgeGraphServer::readGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response)
{
  if (KnowledgeGraphServer::getKnowledgeGraph()->get_node_names().empty()) {
    RCLCPP_INFO(this->get_logger(), "the graph is empty");
  } else {
    RCLCPP_INFO(this->get_logger(), "Inside the graph there are the nodes:");
    for (auto & node_names:KnowledgeGraphServer::getKnowledgeGraph()->get_node_names()) {
      knowledge_graph_msgs::msg::Node node;
      node.node_name = node_names;
      response->nodes.emplace_back(node);
    }
  }

}

void KnowledgeGraphServer::readNodeGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response)
{
  if (KnowledgeGraphServer::getKnowledgeGraph()->get_nodes().empty()) {
    RCLCPP_INFO(this->get_logger(), "the graph is empty");
  } else {
    std::optional<knowledge_graph_msgs::msg::Node> node;
    node = get_node_from_class(request->node_class);
    if (node) {
      response->nodes.emplace_back(node.value());
      for (auto & it:response->nodes) {
        RCLCPP_INFO(this->get_logger(), "the node name is: %s", it.node_name.c_str());
      }
    }
  }
}

void KnowledgeGraphServer::readEdgeClassGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Response> response)
{
  if (KnowledgeGraphServer::getKnowledgeGraph()->get_edges(
      request->source_node,
      request->target_node).empty())
  {
    RCLCPP_INFO(this->get_logger(), "the edge does not exist");
  } else {
    for (auto & the_class:
      KnowledgeGraphServer::getKnowledgeGraph()->get_edges(
        request->source_node,
        request->target_node))
    {
      knowledge_graph_msgs::msg::Edge edge;
      edge.edge_class = the_class.edge_class;
      response->edge.emplace_back(edge);
    }
  }
}

void KnowledgeGraphServer::readEdgeSourceTargetGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Response> response)
{
  if (KnowledgeGraphServer::getKnowledgeGraph()->get_edges(
      request->edge_class).empty())
  {
    RCLCPP_INFO(this->get_logger(), "the edge class does not exist");
  } else {
    for (auto & the_class:
      KnowledgeGraphServer::getKnowledgeGraph()->get_edges(
        request->edge_class))
    {
      knowledge_graph_msgs::msg::Edge edge;
      edge.source_node = the_class.source_node;
      edge.target_node = the_class.target_node;
      response->edge.emplace_back(edge);
    }
  }
}

//la idea es que le pase una clase y me diga todos los nodos de esa clase, y si no existe, que me diga todos los nodos que hay en el grafo
// void KnowledgeGraphServer::readGraph(
//   const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
//   const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response)
// {
//   if (KnowledgeGraphServer::getKnowledgeGraph()->get_node_names().empty()) {
//     RCLCPP_INFO(this->get_logger(), "the graph is empty");
//   } else {
//     // for (auto & name:KnowledgeGraphServer::getKnowledgeGraph()->get_node_names()) {
//     knowledge_graph_msgs::msg::Node node;
//     //node.node_name =;
//     //esto esta mal, tengo que ver como hacer para pasarles los nodos de mi grafo para poder sacarles la clase
//     // node.node_class = get_node_class(node);

//     // if (std::string(name) == request->node_class) {
//     // } else {
//     //   RCLCPP_INFO(this->get_logger(), "Inside the graph there are the nodes:");
//     //   for (auto & node_names:KnowledgeGraphServer::getKnowledgeGraph()->get_node_names()) {
//     //     knowledge_graph_msgs::msg::Node node;
//     //     node.node_name = node_names;
//     //     response->nodes.emplace_back(node);
//     //   }
//     //}
//     //}
//   }
// }
