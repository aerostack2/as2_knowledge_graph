#include "as2_knowledge_graph_service.hpp"

void NewNode::timerCallback(){
 
};

void NewNode::createNode(const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request,const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response ){
    RCLCPP_INFO(this->get_logger(),"service create node");
    knowledge_graph_msgs::msg::Node my_node;
    request_name_received=true;
    my_node.node_name = request->node.node_name;
    my_node.node_class = request->node.node_class;

    this->graph_ = knowledge_graph::KnowledgeGraph::get_instance(shared_from_this());
    RCLCPP_INFO(this->get_logger(),"graph in create node %ld",graph_.use_count());
    RCLCPP_INFO(this->get_logger(),"successfully built node");
     if(this->graph_->update_node(my_node,1)==true){
    RCLCPP_INFO(this->get_logger()," successfully update");
    response->resultado=true;
    };
        RCLCPP_INFO(get_logger(),"graph in create node %ld",graph_.use_count());
};

void NewNode::createEdge(const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response){
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

void NewNode::removeNode(const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request ,const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response ){
    knowledge_graph_msgs::msg::Node my_node;
    request_remove_node_received=true;
    my_node.node_name = request->node.node_name;
    my_node.node_class = request->node.node_class;
    response->resultado=request_remove_node_received;
    if(this->graph_->remove_node(my_node.node_name)==true){
      RCLCPP_INFO(this->get_logger()," successfully remove %s",my_node.node_name.c_str());
    }
};

void NewNode::removeEdge(const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request,const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response ){
  knowledge_graph_msgs::msg::Edge my_edge;
  my_edge.edge_class = request->edge.edge_class;
  my_edge.source_node = request->edge.source_node;
  my_edge.target_node = request->edge.target_node;request_remove_edge_received=true;
  response->resultado=request_remove_edge_received;
    if(this->graph_->remove_edge(my_edge,1)==true){
    RCLCPP_INFO(this->get_logger(),"%s, %s, %s ", my_edge.edge_class.c_str(), my_edge.source_node.c_str(), my_edge.target_node.c_str());
  }
};

    
  void NewNode::addPropertyNode(const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Request> request ,const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateNode::Response> response ){
     knowledge_graph_msgs::msg::Node my_node;
     my_node = request->node;
    //  for (auto& property:my_node.properties){
     for (size_t i=0; i< my_node.properties.size(); i++ ){
      auto& property = my_node.properties[i];

     if(knowledge_graph::add_property(my_node,property.key,property.value)==true){
      response->resultado = 1;
     }
     }
    RCLCPP_INFO(this->get_logger(),"Node property service");
  };

  void NewNode::addPropertyEdge(const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Request> request ,const std::shared_ptr<as2_knowledge_graph_msgs::srv::CreateEdge::Response> response ){
     knowledge_graph_msgs::msg::Edge my_edge;
     my_edge = request->edge;
    for (auto& property:my_edge.properties){
     if(knowledge_graph::add_property(my_edge,property.key,property.value)==true){
      response->resultado = 1;
     }
     }
    RCLCPP_INFO(this->get_logger(),"Edge property service");
  };