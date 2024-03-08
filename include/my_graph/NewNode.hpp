#ifndef NewNode_hpp
#define NewNode_hpp

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "knowledge_graph/graph_utils.hpp"
#include "knowledge_graph_msgs/msg/content.hpp"
#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"
#include "my_graph/srv/create_node.hpp"
#include "my_graph/srv/create_edge.hpp"
#include "my_graph/srv/add_property.hpp"



using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class NewNode : public rclcpp::Node {
public:
  NewNode() : rclcpp::Node("my_knowledge_graph"){
    service_create_node_=this->create_service<my_graph::srv::CreateNode>("create_node", std::bind(&NewNode::createNode, this, _1,_2));
    service_create_edge_=this->create_service<my_graph::srv::CreateEdge>("create_edge", std::bind(&NewNode::createEdge, this, _1,_2));

    service_remove_node_=this->create_service<my_graph::srv::CreateNode>("remove_node", std::bind(&NewNode::removeNode, this, _1,_2));
    service_remove_edge_=this->create_service<my_graph::srv::CreateEdge>("remove_edge", std::bind(&NewNode::removeEdge, this, _1,_2));

    add_property_node_=this->create_service<my_graph::srv::CreateNode>("add_property_node", std::bind(&NewNode::addPropertyNode, this, _1,_2));
    
    timer_ = this->create_wall_timer(20ms, std::bind(&NewNode::timerCallback, this));
  }

private:
    std::shared_ptr<knowledge_graph::KnowledgeGraph> graph_;
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Service<my_graph::srv::CreateNode>::SharedPtr service_create_node_;
    rclcpp::Service<my_graph::srv::CreateEdge>::SharedPtr service_create_edge_;
    rclcpp::Service<my_graph::srv::CreateNode>::SharedPtr service_remove_node_;
    rclcpp::Service<my_graph::srv::CreateEdge>::SharedPtr service_remove_edge_;
    rclcpp::Service<my_graph::srv::CreateNode>::SharedPtr add_property_node_;

    
    bool request_name_received;   
    bool request_edge_received; 
    bool request_remove_node_received;   
    bool request_remove_edge_received; 
    size_t count_;  
  
    void createNode(const std::shared_ptr<my_graph::srv::CreateNode::Request> request ,const std::shared_ptr<my_graph::srv::CreateNode::Response> response );
    void createEdge(const std::shared_ptr<my_graph::srv::CreateEdge::Request> request,const std::shared_ptr<my_graph::srv::CreateEdge::Response> response );
    void removeNode(const std::shared_ptr<my_graph::srv::CreateNode::Request> request ,const std::shared_ptr<my_graph::srv::CreateNode::Response> response );
    void removeEdge(const std::shared_ptr<my_graph::srv::CreateEdge::Request> request,const std::shared_ptr<my_graph::srv::CreateEdge::Response> response );
    void addPropertyNode(const std::shared_ptr<my_graph::srv::CreateNode::Request> request ,const std::shared_ptr<my_graph::srv::CreateNode::Response> response );
    void timerCallback();
};

void NewNode::timerCallback(){

};

void NewNode::createNode(const std::shared_ptr<my_graph::srv::CreateNode::Request> request,const std::shared_ptr<my_graph::srv::CreateNode::Response> response ){
    
    knowledge_graph_msgs::msg::Node my_node;
    request_name_received=true;
    my_node.node_name = request->node.node_name;
    my_node.node_class = request->node.node_class;

    if((response->resultado=request_name_received)==1){
      RCLCPP_INFO(this->get_logger(),"successfullly received node");
    }; 

    this->graph_ = knowledge_graph::KnowledgeGraph::get_instance(shared_from_this());
    RCLCPP_INFO(this->get_logger(),"successfully built node");
     if(this->graph_->update_node(my_node,1)==true){
    RCLCPP_INFO(this->get_logger()," successfully update"); 
    };
};

void NewNode::createEdge(const std::shared_ptr<my_graph::srv::CreateEdge::Request> request,const std::shared_ptr<my_graph::srv::CreateEdge::Response> response){
  knowledge_graph_msgs::msg::Edge my_edge;
  my_edge.edge_class = request->edge.edge_class;
  my_edge.source_node = request->edge.source_node;
  my_edge.target_node = request->edge.target_node;

  request_edge_received=true;
  response->resultado=request_edge_received;
  RCLCPP_INFO(this->get_logger(),"successfullly received edge");
    if(this->graph_->update_edge(my_edge,1)==true){
    RCLCPP_INFO(this->get_logger(),"successfully update");
  this->graph_->get_edges(my_edge.source_node,my_edge.target_node);
  this->graph_->get_edges(my_edge.edge_class);
  this->graph_->get_out_edges(my_edge.source_node);
  this->graph_->get_in_edges(my_edge.target_node);
    }
};

void NewNode::removeNode(const std::shared_ptr<my_graph::srv::CreateNode::Request> request ,const std::shared_ptr<my_graph::srv::CreateNode::Response> response ){
    knowledge_graph_msgs::msg::Node my_node;
    request_remove_node_received=true;
    my_node.node_name = request->node.node_name;
    my_node.node_class = request->node.node_class;
    response->resultado=request_remove_node_received;
    if(this->graph_->remove_node(my_node.node_name)==true){
      RCLCPP_INFO(this->get_logger()," successfully remove %s",my_node.node_name.c_str());
    }
};

void NewNode::removeEdge(const std::shared_ptr<my_graph::srv::CreateEdge::Request> request,const std::shared_ptr<my_graph::srv::CreateEdge::Response> response ){
  knowledge_graph_msgs::msg::Edge my_edge;
  my_edge.edge_class = request->edge.edge_class;
  my_edge.source_node = request->edge.source_node;
  my_edge.target_node = request->edge.target_node;request_remove_edge_received=true;
  response->resultado=request_remove_edge_received;
    if(this->graph_->remove_edge(my_edge,1)==true){
    RCLCPP_INFO(this->get_logger(),"%s, %s, %s ", my_edge.edge_class.c_str(), my_edge.source_node.c_str(), my_edge.target_node.c_str());
  }
};

    
  void NewNode::addPropertyNode(const std::shared_ptr<my_graph::srv::CreateNode::Request> request ,const std::shared_ptr<my_graph::srv::CreateNode::Response> response ){
     knowledge_graph_msgs::msg::Node my_node;
     my_node = request->node;
    //  for (auto& property:my_node.properties){
     for (size_t i=0; i< my_node.properties.size(); i++ ){
      auto& property = my_node.properties[i];

     if(knowledge_graph::add_property(my_node,property.key,property.value)==true){
      response->resultado = 1;
     }
     }
  };

#endif