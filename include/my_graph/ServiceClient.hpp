#ifndef ServiceClient_hpp
#define ServiceClient_hpp

#include <memory>
#include <optional>
#include <cstdlib>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "my_graph/srv/create_node.hpp"
#include "my_graph/srv/create_edge.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ServiceClient : public rclcpp::Node{
    public:
        ServiceClient() : rclcpp::Node("my_client_node"){
            client_create_node_=this->create_client<my_graph::srv::CreateNode>("create_node");
            client_remove_node_=this->create_client<my_graph::srv::CreateNode>("remove_node");
            client_create_edge_=this->create_client<my_graph::srv::CreateEdge>("create_edge");
            client_remove_edge_=this->create_client<my_graph::srv::CreateEdge>("remove_edge");
            client_add_property_node_=this->create_client<my_graph::srv::CreateNode>("add_property_node");
            client_add_property_edge_=this->create_client<my_graph::srv::CreateEdge>("add_property_edge");

            timer_ = this->create_wall_timer(20ms, std::bind(&ServiceClient::timerCallback, this));
        }

        bool is_service_done() const {
            return this->service_done_;
        }

    private:

        bool service_done_ = false;

        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Client<my_graph::srv::CreateNode>::SharedPtr client_create_node_;
        rclcpp::Client<my_graph::srv::CreateNode>::SharedPtr client_remove_node_;
        rclcpp::Client<my_graph::srv::CreateEdge>::SharedPtr client_create_edge_;
        rclcpp::Client<my_graph::srv::CreateEdge>::SharedPtr client_remove_edge_;
        rclcpp::Client<my_graph::srv::CreateNode>::SharedPtr client_add_property_node_;
        rclcpp::Client<my_graph::srv::CreateEdge>::SharedPtr client_add_property_edge_;
        
         void timerCallback();
         void createNode();
         void createEdge();
         void removeNode();
         void removeEdge();
         void addPropertyNode();
         void addPropertyEdge();



};
void ServiceClient::timerCallback(){
    removeNode();
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

#endif