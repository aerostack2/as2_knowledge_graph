#include "as2_knowledge_graph_client.hpp"
#include "knowledge_graph/graph_utils.hpp"
#include "as2_knowledge_graph_graph_utils.hpp"


void KnowledgeGraphClient::timerCallback()
{
}

bool KnowledgeGraphClient::createNode(const knowledge_graph_msgs::msg::Node & client_)
{
  RCLCPP_INFO(this->get_logger(), "Setting node name");
  //New node SynchronousServiceClient object
  // as2::Node *node;
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::CreateNode>(
    "create_node", this);
  //Set request
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

//     //RCLCPP_INFO(this->get_logger(), " Create:\n Node Name: %s\n, Node Class: %s\n",request->node.node_name.c_str(), request->node.node_class.c_str());
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

//Synchronous create edge request
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

//Asynchronous delete Node
// void KnowledgeGraphClient::removeNode(){
//     knowledge_graph_msgs::msg::Node my_node;
//     auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
//     request->node.node_name = my_node.node_name;
//     request->node.node_class = my_node.node_class;
//     auto result = client_remove_node_->async_send_request(request);
//     RCLCPP_INFO(this->get_logger(), "Remove node: %s",request->node.node_name.c_str());

// };

//Synchronous delete Node
bool KnowledgeGraphClient::removeNode(const knowledge_graph_msgs::msg::Node & client_)
{
  RCLCPP_INFO(this->get_logger(), "Deleting node name");
  auto set_cli = as2::SynchronousServiceClient<as2_knowledge_graph_msgs::srv::CreateNode>(
    "remove_node", this);
  //Set request
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

void KnowledgeGraphClient::removeEdge()
{
  knowledge_graph_msgs::msg::Edge my_edge;
  auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateEdge::Request>();
  request->edge.edge_class = my_edge.edge_class;
  request->edge.source_node = my_edge.source_node;
  request->edge.target_node = my_edge.target_node;
  auto result = client_remove_edge_->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "Remove edge: %s", request->edge.edge_class.c_str());
}

//Asynchronous addPropertyNode
// void KnowledgeGraphClient::addPropertyNode(){
//     auto request = std::make_shared<as2_knowledge_graph_msgs::srv::CreateNode::Request>();
//     auto result = client_add_property_node_->async_send_request(request);
//     for(size_t i=0; i<request->node.properties.size(); i++){
//        RCLCPP_INFO(this->get_logger(),"Add property: %d",request->node.properties.at(i).value.int_value);
//     }
// };

//Synchronous addProperty
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
    for (auto & prop: client_.properties) {
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
