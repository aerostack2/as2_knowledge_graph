#include "as2_knowledge_graph_client.hpp"
#include "knowledge_graph/graph_utils.hpp"

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
}


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

// Synchronous delete Node
bool KnowledgeGraphClient::removeNode(const knowledge_graph_msgs::msg::Node & client_)
{
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

// Syncronous removeEdge
bool KnowledgeGraphClient::removeEdge(const knowledge_graph_msgs::msg::Edge & client_)
{
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

// Synchronous addProperty
bool KnowledgeGraphClient::addPropertyNode(
  const knowledge_graph_msgs::msg::Node & client_)
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::CreateNode>(
    "add_property_node", this);

  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Response>();
  request->node = client_;

  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    for (auto & prop : client_.properties) {
      RCLCPP_INFO(
        this->get_logger(),
        "The node %s request to add the property with the key: %s and the content %s:",
        request->node.node_name.c_str(), prop.key.c_str(), knowledge_graph::to_string(
          prop.value).c_str());
    }
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "it was not able to add the property");
    return false;
  }
}

bool KnowledgeGraphClient::addPropertyEdge(const knowledge_graph_msgs::msg::Edge & client_)
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::CreateEdge>(
    "add_property_edge", this);
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateEdge::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::CreateEdge::Response>();
  request->edge = client_;
  bool out = set_cli.sendRequest(request, response);
  RCLCPP_INFO(
    this->get_logger(), "The edge %s request to add a property ",
    request->edge.edge_class.c_str());
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
  request->node_class = node_class_client_;
  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    for (auto & nodes : response->nodes) {
      RCLCPP_INFO(
        this->get_logger(), "The node %s is of type %s",
        nodes.node_name.c_str(), node_class_client_.c_str());
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
        edges.edge_class.c_str(), request->source_node.c_str(), request->target_node.c_str());
    }
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "The client fail");
    return false;
  }
}
bool KnowledgeGraphClient::readEdgeSourceTargetGraph(
  const std::string & edge_class_client_)
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::ReadEdgeGraph>(
    "read_edge_source_target_graph", this);
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::ReadEdgeGraph::Response>();
  request->edge_class = edge_class_client_;
  bool out = set_cli.sendRequest(request, response);
  if (out && response) {
    for (auto & edges : response->edge) {
      RCLCPP_INFO(
        this->get_logger(), "The edge of class %s exists between %s and %s",
        edge_class_client_.c_str(), edges.source_node.c_str(), edges.target_node.c_str());
    }
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "The client fail");
    return false;
  }
}
bool KnowledgeGraphClient::readNodePropertyGraph(const std::string & node_name_client_)
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::ReadProperty>(
    "read_node_property_graph", this);
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::ReadProperty::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::ReadProperty::Response>();
  request->node_name = node_name_client_;
  bool out = set_cli.sendRequest(request, response);
  if (!(out && response)) {
    RCLCPP_INFO(this->get_logger(), "The client fail");
    return false;
  }
  for (auto & prop : response->properties) {
    RCLCPP_INFO(
      this->get_logger(), "The node %s has the property %s with the key %s",
      node_name_client_.c_str(), knowledge_graph::to_string(
        prop.value).c_str(), prop.key.c_str());
  }
  return true;
}

bool KnowledgeGraphClient::readEdgePropertyGraph(const std::string & edge_class_client_)
{
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::ReadProperty>(
    "read_edge_property_graph", this);
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::ReadProperty::Request>();
  auto response = std::make_shared<as2_knowledge_graph_msgs::srv::ReadProperty::Response>();
  request->edge_class = edge_class_client_;
  bool out = set_cli.sendRequest(request, response);
  if (!(out && response)) {
    RCLCPP_INFO(this->get_logger(), "The client fail");
    return false;
  }
  for (auto & prop : response->properties) {
    RCLCPP_INFO(
      this->get_logger(), "The edge %s has the property %s with the key %s",
      edge_class_client_.c_str(), knowledge_graph::to_string(
        prop.value).c_str(), prop.key.c_str());
  }
  return true;
}
