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
  if (KnowledgeGraphServer::getKnowledgeGraph()->get_nodes().empty()) {
    return {};
  } else {
    for (auto & it : KnowledgeGraphServer::getKnowledgeGraph()->get_nodes()) {
      if (it.node_class == node_class) {
        return it;
      }
    }
    return {};
  }
}

void KnowledgeGraphServer::createNode(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "service create node");
  knowledge_graph_msgs::msg::Node my_node;
  my_node = request->node;
  if (this->knowledge_graph_ptr_->update_node(my_node, 1) == true) {
    RCLCPP_INFO(this->get_logger(), " successfully update the node");
    response->resultado = true;
  }
}

void KnowledgeGraphServer::createEdge(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "service create edge");
  knowledge_graph_msgs::msg::Edge my_edge;
  my_edge = request->edge;
  if (this->knowledge_graph_ptr_->update_edge(my_edge, 1) == true) {
    response->resultado = true;
    RCLCPP_INFO(this->get_logger(), "successfully update the edge");
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
  RCLCPP_INFO(this->get_logger(), "Service remove node");
  knowledge_graph_msgs::msg::Node my_node;
  my_node = request->node;
  if (this->knowledge_graph_ptr_->remove_node(my_node.node_name) == true) {
    response->resultado = true;
    RCLCPP_INFO(this->get_logger(), "Successfully remove %s", my_node.node_name.c_str());
  }
}

void KnowledgeGraphServer::removeEdge(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service remove edge");
  knowledge_graph_msgs::msg::Edge my_edge;
  my_edge = request->edge;
  if (this->knowledge_graph_ptr_->remove_edge(my_edge, 1) == true) {
    response->resultado = true;
    RCLCPP_INFO(
      this->get_logger(), "Remove edge: %s between %s and %s",
      my_edge.edge_class.c_str(), my_edge.source_node.c_str(), my_edge.target_node.c_str());
  }
}


void KnowledgeGraphServer::addPropertyNode(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Adding a node property service");
  if (KnowledgeGraphServer::getKnowledgeGraph()->exist_node(request->node.node_name)) {


    std::optional<knowledge_graph_msgs::msg::Node> my_node;
    my_node = KnowledgeGraphServer::getKnowledgeGraph()->get_node(request->node.node_name);

    // if (knowledge_graph::add_property(my_node.value(), "edad", 15)) {
    //   std::cout << "ok" << std::endl;
    // }
    for (auto & property : my_node.value().properties) {
      if (knowledge_graph::add_property(
          my_node.value(), std::string(property.key),
          property.value))
      {
        RCLCPP_INFO(this->get_logger(), "the key: %s", property.key.c_str());
        RCLCPP_INFO(
          this->get_logger(), "the content: %s",
          knowledge_graph::to_string(property.value).c_str());
        response->resultado = 1;
      } else {
        RCLCPP_INFO(this->get_logger(), "The service fail");
      }
    }
  }
}


void KnowledgeGraphServer::addPropertyEdge(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Adding a edge property service");
  knowledge_graph_msgs::msg::Edge my_edge;
  my_edge = request->edge;
  for (auto & property : my_edge.properties) {
    if (knowledge_graph::add_property(my_edge, property.key, property.value) == true) {
      response->resultado = 1;
      RCLCPP_INFO(this->get_logger(), "ok");
    } else {
      RCLCPP_INFO(this->get_logger(), "no ok");
    }
  }
  RCLCPP_INFO(this->get_logger(), "Edge property service");
}
void KnowledgeGraphServer::readGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Access to the graph service");
  if (KnowledgeGraphServer::getKnowledgeGraph()->get_node_names().empty()) {
    RCLCPP_INFO(this->get_logger(), "the graph is empty");
  } else {
    if (KnowledgeGraphServer::getKnowledgeGraph()->exist_node(request->node_name)) {
      knowledge_graph_msgs::msg::Node node;
      node.node_name = request->node_name;
      response->nodes.emplace_back(node);
      RCLCPP_INFO(this->get_logger(), "The node exist");
    } else {
      RCLCPP_INFO(this->get_logger(), "Inside the graph there are the nodes:");
      for (auto & node_names : KnowledgeGraphServer::getKnowledgeGraph()->get_node_names()) {
        knowledge_graph_msgs::msg::Node node;
        node.node_name = node_names;
        response->nodes.emplace_back(node);

      }
    }
  }
}

void KnowledgeGraphServer::readNodeGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response)
{
  if (KnowledgeGraphServer::getKnowledgeGraph()->get_node_names().empty()) {
    RCLCPP_INFO(this->get_logger(), "the graph is empty");
  } else {
    if (KnowledgeGraphServer::get_node_from_class(request->node_class).has_value()) {
      knowledge_graph_msgs::msg::Node node;
      node = KnowledgeGraphServer::get_node_from_class(request->node_class).value();
      response->nodes.emplace_back(node);
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
void KnowledgeGraphServer::readNodePropertyGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response)
{
//   if (KnowledgeGraphServer::getKnowledgeGraph()->exist_node(request->node_name)) {
//     knowledge_graph_msgs::msg::Node the_node;
//     the_node = KnowledgeGraphServer::getKnowledgeGraph()->get_node(request->node_name);
// response->nodes.

//   }
}
