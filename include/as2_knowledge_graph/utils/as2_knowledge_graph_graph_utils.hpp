#ifndef Graph_Utils_HPP_
#define Graph_Utils_HPP_

#include <iostream>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>
#include <rclcpp/node.hpp>
#include "rclcpp/rclcpp.hpp"


#include "knowledge_graph/knowledge_graph.hpp"
#include "knowledge_graph/graph_utils.hpp"
#include "knowledge_graph_msgs/msg/content.hpp"
#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

knowledge_graph_msgs::msg::Content whichType(const knowledge_graph_msgs::msg::Property & prop);
int whichType2(const knowledge_graph_msgs::msg::Property & prop);


#endif
