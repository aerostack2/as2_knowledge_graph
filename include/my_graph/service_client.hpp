#ifndef Service_Client_hpp
#define Service_Client_hpp

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


#endif