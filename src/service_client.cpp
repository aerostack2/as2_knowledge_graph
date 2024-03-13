#include "service_client.hpp"

void ServiceClient::timerCallback(){
};

void ServiceClient::createNode(){
    knowledge_graph_msgs::msg::Node my_node;
    auto request = std::make_shared<my_graph::srv::CreateNode::Request>();
    request->node.node_name = my_node.node_name;
    request->node.node_class = my_node.node_class;
    auto result = client_create_node_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), " Create:\n Node Name: %s\n, Node Class: %s\n",request->node.node_name.c_str(), request->node.node_class.c_str());
};

void ServiceClient::createEdge(){
    knowledge_graph_msgs::msg::Edge my_edge;
    auto request = std::make_shared<my_graph::srv::CreateEdge::Request>();
    request->edge.edge_class = my_edge.edge_class ;
    request->edge.source_node = my_edge.source_node;
    request->edge.target_node = my_edge.target_node;
    auto result = client_create_edge_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(),"Edge class: %s\n Source node: %s\n Target node: %s\n",request->edge.edge_class.c_str(),request->edge.source_node.c_str(),request->edge.target_node.c_str());
};
void ServiceClient::removeNode(){
    knowledge_graph_msgs::msg::Node my_node;
    auto request = std::make_shared<my_graph::srv::CreateNode::Request>();
    request->node.node_name = my_node.node_name;
    request->node.node_class = my_node.node_class;
    auto result = client_remove_node_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Remove node: %s",request->node.node_name.c_str());

};

void ServiceClient::removeEdge(){
    knowledge_graph_msgs::msg::Edge my_edge;
    auto request = std::make_shared<my_graph::srv::CreateEdge::Request>();
    request->edge.edge_class = my_edge.edge_class ;
    request->edge.source_node = my_edge.source_node;
    request->edge.target_node = my_edge.target_node;
    auto result = client_remove_edge_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(),"Remove edge: %s",request->edge.edge_class.c_str());
};

void ServiceClient::addPropertyNode(){
    auto request = std::make_shared<my_graph::srv::CreateNode::Request>();
    auto result = client_add_property_node_->async_send_request(request);
    for(size_t i=0; i<request->node.properties.size(); i++){
       RCLCPP_INFO(this->get_logger(),"Add property: %d",request->node.properties.at(i).value.int_value); 
    }
};

void ServiceClient::addPropertyEdge(){
    auto request = std::make_shared<my_graph::srv::CreateEdge::Request>();
    auto result = client_add_property_edge_->async_send_request(request);
    for(size_t i=0; i<request->edge.properties.size(); i++){
       RCLCPP_INFO(this->get_logger(),"Add property: %d",request->edge.properties.at(i).value.int_value); 
    }

};