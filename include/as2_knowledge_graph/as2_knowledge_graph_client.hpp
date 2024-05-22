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


#ifndef AS2_KNOWLEDGE_GRAPH__AS2_KNOWLEDGE_GRAPH_CLIENT_HPP_
#define AS2_KNOWLEDGE_GRAPH__AS2_KNOWLEDGE_GRAPH_CLIENT_HPP_

#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <chrono>

#include "as2_core/node.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_knowledge_graph_msgs/srv/create_edge.hpp"
#include "as2_knowledge_graph_msgs/srv/create_node.hpp"
#include "as2_knowledge_graph_msgs/srv/read_graph.hpp"
#include "as2_knowledge_graph_msgs/srv/read_edge_graph.hpp"
#include "as2_knowledge_graph_msgs/srv/read_property.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/rclcpp.hpp"


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


#endif  // AS2_KNOWLEDGE_GRAPH__AS2_KNOWLEDGE_GRAPH_CLIENT_HPP_
