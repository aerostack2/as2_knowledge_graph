#include "as2_knowledge_graph_client.hpp"
#include "knowledge_graph/graph_utils.hpp"
#include "utils/as2_knowledge_graph_graph_utils.hpp"


void KnowledgeGraphClient::timerCallback()
{
}

bool KnowledgeGraphClient::createNode(const knowledge_graph_msgs::msg::Node & client_)
{
  RCLCPP_INFO(this->get_logger(), "Setting node name");
  // New node SynchronousServiceClient object
  // as2::Node *node;
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::CreateNode>(
    "create_node", this);
  // Set request
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Response>();
  request->node.node_name = client_.node_name;
  request->node.node_class = client_.node_class;

  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    RCLCPP_INFO(
      this->get_logger(), " Create:\n Node Name: %s\n, Node Class: %s\n",
      request->node.node_name.c_str(), request->node.node_class.c_str());
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "it was not able to creted de node");
    return false;
  }
//     //knowledge_graph_msgs::msg::Node my_node;
//     // auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
//     // request->node.node_name = request_n.node.node_name;
//     // request->node.node_name = request_n.node.node_name;
//     // request->node.node_name = my_node.node_name;
//     // request->node.node_class = my_node.node_class;
//     //auto result = client_create_node_->async_send_request(request);

//     // RCLCPP_INFO(this->get_logger(), " Create:\n Node Name: %s\n, Node Class: %s\n",request->node.node_name.c_str(), request->node.node_class.c_str());
}

// bool KnowledgeGraphClient::createNode(const knowledge_graph_msgs::msg::Node &client_){
//     RCLCPP_INFO(this->get_logger(),"Setting node name");
//     auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
//     request->node.node_name = client_.node_name;
//     request->node.node_class = client_.node_class;
//     auto result = client_create_node_->async_send_request(request);

//         RCLCPP_INFO(this->get_logger(), " Create:\n Node Name: %s\n, Node Class: %s\n",request->node.node_name.c_str(), request->node.node_class.c_str());
//         return true;
//     // }else{
//     //     RCLCPP_INFO(this->get_logger(),"it was not able to creted de node");
//     //   return false;
//     // }
// };

//Asynchronous create edge request
// void KnowledgeGraphClient::createEdge(const knowledge_graph_msgs::msg::Edge &client_){
//     knowledge_graph_msgs::msg::Edge my_edge;
//     auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateEdge::Request>();
//     request->edge.edge_class=client_.edge_class;
//     request->edge.source_node=client_.source_node;
//     request->edge.target_node=client_.target_node;
//     // request->edge.edge_class = my_edge.edge_class ;
//     // request->edge.source_node = my_edge.source_node;
//     // request->edge.target_node = my_edge.target_node;
//     auto result = client_create_edge_->async_send_request(request);
//     RCLCPP_INFO(this->get_logger(),"Edge class: %s\n Source node: %s\n Target node: %s\n",request->edge.edge_class.c_str(),request->edge.source_node.c_str(),request->edge.target_node.c_str());
// };

// Synchronous create edge request
bool KnowledgeGraphClient::createEdge(const knowledge_graph_msgs::msg::Edge & client_)
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::CreateEdge>(
    "create_edge", this);
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateEdge::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::CreateEdge::Response>();
  request->edge.edge_class = client_.edge_class;
  request->edge.source_node = client_.source_node;
  request->edge.target_node = client_.target_node;
  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    RCLCPP_INFO(
      this->get_logger(), "Edge class: %s\n Source node: %s\n Target node: %s\n",
      request->edge.edge_class.c_str(),
      request->edge.source_node.c_str(), request->edge.target_node.c_str());
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "it was not able to creted de node");
    return false;
  }
}

// Asynchronous delete Node
// void KnowledgeGraphClient::removeNode(){
//     knowledge_graph_msgs::msg::Node my_node;
//     auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
//     request->node.node_name = my_node.node_name;
//     request->node.node_class = my_node.node_class;
//     auto result = client_remove_node_->async_send_request(request);
//     RCLCPP_INFO(this->get_logger(), "Remove node: %s",request->node.node_name.c_str());

// };

// Synchronous delete Node
bool KnowledgeGraphClient::removeNode(const knowledge_graph_msgs::msg::Node & client_)
{
  RCLCPP_INFO(this->get_logger(), "Deleting node name");
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::CreateNode>(
    "remove_node", this);
  // Set request
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Response>();
  request->node.node_name = client_.node_name;
  request->node.node_class = client_.node_class;

  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    RCLCPP_INFO(this->get_logger(), "Remove node: %s", request->node.node_name.c_str());
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "it was not able to creted de node");
    return false;
  }
}

// Asynchronous removeEdge
// void KnowledgeGraphClient::removeEdge()
// {
//   knowledge_graph_msgs::msg::Edge my_edge;
//   auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateEdge::Request>();
//   request->edge.edge_class = my_edge.edge_class;
//   request->edge.source_node = my_edge.source_node;
//   request->edge.target_node = my_edge.target_node;
//   auto result = client_remove_edge_->async_send_request(request);
//   RCLCPP_INFO(this->get_logger(), "Remove edge: %s", request->edge.edge_class.c_str());
// }

// Syncronous removeEdge
bool KnowledgeGraphClient::removeEdge(const knowledge_graph_msgs::msg::Edge & client_)
{
  RCLCPP_INFO(this->get_logger(), "Deleting edge");
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::CreateEdge>(
    "remove_edge", this);
  // Set request
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateEdge::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::CreateEdge::Response>();
  request->edge = client_;

  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    RCLCPP_INFO(
      this->get_logger(), "Remove edge: %s between %s and %s ",
      request->edge.edge_class.c_str(),
      request->edge.source_node.c_str(), request->edge.target_node.c_str());
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "it was not able to removed the edge");
    return false;
  }
}

// Asynchronous addPropertyNode
// void KnowledgeGraphClient::addPropertyNode(){
//     auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
//     auto result = client_add_property_node_->async_send_request(request);
//     for(size_t i=0; i<request->node.properties.size(); i++){
//        RCLCPP_INFO(this->get_logger(),"Add property: %d",request->node.properties.at(i).value.int_value);
//     }
// };

// Synchronous addProperty
bool KnowledgeGraphClient::addPropertyNode(const knowledge_graph_msgs::msg::Node & client_)
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::CreateNode>(
    "add_property_node", this);

  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Response>();
  request->node = client_;
  bool out = set_cli.sendRequest(request, response);
  RCLCPP_INFO(
    this->get_logger(), "The node %s request to add the property ",
    request->node.node_name.c_str());
  if (out && response) {
    for (auto & prop : client_.properties) {
      RCLCPP_INFO(this->get_logger(), "the propety with the key: %s", prop.key.c_str());
      RCLCPP_INFO(
        this->get_logger(), "the property with the content: %s",
        knowledge_graph::to_string(prop.value).c_str());
    }
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "it was not able to add the property");
    return false;
  }
}

void KnowledgeGraphClient::addPropertyEdge()
{
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateEdge::Request>();
  auto result = client_add_property_edge_->async_send_request(request);
  for (size_t i = 0; i < request->edge.properties.size(); i++) {
    RCLCPP_INFO(
      this->get_logger(), "Add property: %d", request->edge.properties.at(
        i).value.int_value);
  }
}

bool KnowledgeGraphClient::readGraph(const std::string & node_name_client_)
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::ReadGraph>(
    "read_graph", this);
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::ReadGraph::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::ReadGraph::Response>();
  request->node_name = node_name_client_;
  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    for (auto & nodes : response->nodes) {
      RCLCPP_INFO(this->get_logger(), "The node: %s exists in the graph", nodes.node_name.c_str());
    }
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "The client fail");
    return false;
  }
}

bool KnowledgeGraphClient::readGraph()
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::ReadGraph>(
    "read_graph", this);
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::ReadGraph::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::ReadGraph::Response>();
  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    for (auto & nodes : response->nodes) {
      RCLCPP_INFO(this->get_logger(), "The node: %s is inside the graph", nodes.node_name.c_str());
    }
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "The client fail");
    return false;
  }
}

bool KnowledgeGraphClient::readNodeGraph(const std::string & node_class_client_)
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::ReadGraph>(
    "read_node_graph", this);
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::ReadGraph::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::ReadGraph::Response>();
  request->node_name = node_class_client_;
  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    for (auto & nodes : response->nodes) {
      RCLCPP_INFO(this->get_logger(), nodes.node_name.c_str());
    }
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "The client fail");
    return false;
  }
}

bool KnowledgeGraphClient::readEdgeClassGraph(
  const knowledge_graph_msgs::msg::Node & source_client_,
  const knowledge_graph_msgs::msg::Node & target_client_)
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::ReadEdgeGraph>(
    "read_edge_class_graph", this);
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Response>();
  request->source_node = source_client_.node_name;
  request->target_node = target_client_.node_name;
  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    for (auto & edges : response->edge) {
      RCLCPP_INFO(
        this->get_logger(), "There is the edge %s between %s and %s",
        edges.edge_class.c_str(), edges.source_node.c_str(), edges.target_node.c_str());

    }
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "The client fail");
    return false;
  }
}
