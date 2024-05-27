// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "as2_knowledge_graph_service.hpp"
#include <memory>
#include <rclcpp/node.hpp>

void KnowledgeGraphServer::timerCallback()
{
}

std::vector<knowledge_graph_msgs::msg::Node> KnowledgeGraphServer::get_nodes_from_class(
  const std::string node_class)
{
  std::vector<knowledge_graph_msgs::msg::Node> aux_nodes;
  for (auto & it : knowledge_graph_ptr_->get_nodes()) {
    if (it.node_class == node_class) {
      aux_nodes.emplace_back(it);
    }
  }
  return aux_nodes;
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
  my_edge.edge_class = request->edge.edge_class;
  my_edge.source_node = request->edge.source_node;
  my_edge.target_node = request->edge.target_node;
  if (this->knowledge_graph_ptr_->update_edge(my_edge, 1) == true) {
    response->resultado = true;
    RCLCPP_INFO(this->get_logger(), "successfully update the edge");

    // Esto creo que es inecesario
    // this->knowledge_graph_ptr_->get_edges(my_edge.source_node, my_edge.target_node);
    // this->knowledge_graph_ptr_->get_edges(my_edge.edge_class);
    // this->knowledge_graph_ptr_->get_out_edges(my_edge.source_node);
    // this->knowledge_graph_ptr_->get_in_edges(my_edge.target_node);
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
  std::optional<knowledge_graph_msgs::msg::Node> my_node;
  // knowledge_graph_ptr_->update_node(request->node);
  my_node = knowledge_graph_ptr_->get_node(request->node.node_name);
  if (!my_node.has_value()) {
    response->resultado = false;
    RCLCPP_INFO(this->get_logger(), "The node %s does not exist", request->node.node_name.c_str());
  }
  knowledge_graph_msgs::msg::Node & new_node = request->node;
  if (!knowledge_graph::add_property(my_node.value(), new_node.properties)) {
    RCLCPP_INFO(this->get_logger(), "The service fail");
    response->resultado = false;
  }
  RCLCPP_INFO(this->get_logger(), "correctly update de properties");
  knowledge_graph_ptr_->update_node(my_node.value());
  response->resultado = true;
}


void KnowledgeGraphServer::addPropertyEdge(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Adding an edge property service");
  std::vector<knowledge_graph_msgs::msg::Edge> old_my_edge;
  old_my_edge = knowledge_graph_ptr_->get_edges(request->edge.edge_class);
  if (old_my_edge.empty()) {
    response->resultado = false;
    RCLCPP_INFO(this->get_logger(), "The edge %s does not exist", request->edge.edge_class.c_str());
  }
  knowledge_graph_msgs::msg::Edge & new_edge = request->edge;
  for (auto & edge : old_my_edge) {
    if (!knowledge_graph::add_property(edge, new_edge.properties)) {
      RCLCPP_INFO(this->get_logger(), "The service fail");
      response->resultado = false;
    }
    RCLCPP_INFO(this->get_logger(), "correctly update de properties");
    response->resultado = true;
    knowledge_graph_ptr_->update_edge(edge);
  }
}


void KnowledgeGraphServer::readGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Access to the graph service");
  if (knowledge_graph_ptr_->get_node_names().empty()) {
    RCLCPP_INFO(this->get_logger(), "the graph is empty");
  } else {
    if (knowledge_graph_ptr_->exist_node(request->node_name)) {
      std::optional<knowledge_graph_msgs::msg::Node> node;
      node = knowledge_graph_ptr_->get_node(request->node_name);
      response->nodes.emplace_back(node.value());
      RCLCPP_INFO(this->get_logger(), "The node exist");
    } else {
      RCLCPP_INFO(this->get_logger(), "Inside the graph there are the nodes:");
      for (auto & node_aux : knowledge_graph_ptr_->get_nodes()) {
        // knowledge_graph_msgs::msg::Node node;
        // node.node_name = node_names;
        response->nodes.emplace_back(node_aux);
        // RCLCPP_INFO(
        //   this->get_logger(), "%s %s",
        //   node_aux.node_name.c_str(), node_aux.node_class.c_str());
      }
    }
  }
}

void KnowledgeGraphServer::readNodeGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadGraph::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Reading the nodes from class %s", request->node_class.c_str());
  if (knowledge_graph_ptr_->get_node_names().empty()) {
    RCLCPP_INFO(this->get_logger(), "the graph is empty");
  } else {
    if (KnowledgeGraphServer::get_nodes_from_class(request->node_class).empty()) {
      RCLCPP_INFO(
        this->get_logger(), "The are not nodes with the class name: %s",
        request->node_class.c_str());
    } else {
      for (auto & node : KnowledgeGraphServer::get_nodes_from_class(request->node_class)) {
        response->nodes.emplace_back(node);
        RCLCPP_INFO(this->get_logger(), "%s", node.node_name.c_str());
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
void KnowledgeGraphServer::readNodePropertyGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadProperty::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadProperty::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Access to Node properties service");
  if (knowledge_graph_ptr_->get_nodes().empty()) {
    RCLCPP_INFO(this->get_logger(), "the graph is empty");
  }
  if (!knowledge_graph_ptr_->exist_node(request->node_name)) {
    RCLCPP_INFO(
      this->get_logger(), "the node %s does not belong to this graph", request->node_name.c_str());
  }
  std::optional<knowledge_graph_msgs::msg::Node> node;
  node = knowledge_graph_ptr_->get_node(request->node_name);
  if (node.value().properties.empty()) {
    RCLCPP_INFO(
      this->get_logger(), "the node %s does not have properties", request->node_name.c_str());
  }
  for (auto & prop : node.value().properties) {
    response->properties.emplace_back(prop);
  }
}
void KnowledgeGraphServer::readEdgePropertyGraph(
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadProperty::Request> request,
  const std::shared_ptr<as2_knowledge_graph_msgs::srv::ReadProperty::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Reading an edge property service");

  std::vector<knowledge_graph_msgs::msg::Edge> my_edge;
  my_edge = knowledge_graph_ptr_->get_edges(request->edge_class);
  if (my_edge.empty()) {
    RCLCPP_INFO(this->get_logger(), "The edge %s does not exist", request->edge_class.c_str());
  }
  for (auto & edge : my_edge) {
    if (edge.properties.empty()) {
      RCLCPP_INFO(
        this->get_logger(), "the edge %s does not have properties", request->edge_class.c_str());
    }
    for (auto & prop : edge.properties) {
      response->properties.emplace_back(prop);
    }
  }
}
