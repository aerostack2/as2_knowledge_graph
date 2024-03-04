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

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ServiceClient : public rclcpp::Node{
    public:
        ServiceClient() : rclcpp::Node("my_client_node"){
            client_create_node_=this->create_client<my_graph::srv::CreateNode>("create_node");
            timer_ = this->create_wall_timer(20ms, std::bind(&ServiceClient::timerCallback, this));
        }
        
        bool is_service_done() const {
            return this->service_done_;
        }

    private:
         rclcpp::TimerBase::SharedPtr timer_; 
         rclcpp::Client<my_graph::srv::CreateNode>::SharedPtr client_create_node_;

         bool service_done_ = false;
        knowledge_graph_msgs::msg::Node node_;

        //  auto request = std::make_shared<my_graph::srv::CreateNode::Request>();

         void timerCallback();
         void responseCallback( rclcpp::Client<my_graph::srv::CreateNode>::SharedFuture future);


};

void ServiceClient::timerCallback(){

    // if(client_create_node_->wait_for_service(1s)){
    //     if(rclcpp::ok()){
    //     RCLCPP_ERROR(
    //         this->get_logger(),"Client interrupted while waiting for service. Terminating...");
    //     return;
    //   }
    //     RCLCPP_INFO(this->get_logger(),"Service Unavailable. Waiting for Service...");
    // }
    auto request = std::make_shared<my_graph::srv::CreateNode::Request>();
    request->node_name = node_.node_name;
    request->node_class = node_.node_class;
    service_done_ = false;

    auto result_future = client_create_node_->async_send_request(request, std::bind(&ServiceClient::responseCallback, this,_1));
    RCLCPP_INFO(this->get_logger(),"ok");
};

void ServiceClient::responseCallback( rclcpp::Client<my_graph::srv::CreateNode>::SharedFuture future){
    RCLCPP_INFO(this->get_logger(),"dentro");
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {

      RCLCPP_INFO(this->get_logger(), "Result: success");
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
};


#endif