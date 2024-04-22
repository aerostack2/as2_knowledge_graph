#include "as2_knowledge_graph_graph_utils.hpp"

knowledge_graph_msgs::msg::Content whichType(const knowledge_graph_msgs::msg::Property & prop)
{
  knowledge_graph_msgs::msg::Content content;
  content.type = prop.value.type;

// switch (prop.value.type)
//     {
//         case knowledge_graph_msgs::msg::Content::BOOL:
//             content.type = 0;
//             break;
//         case knowledge_graph_msgs::msg::Content::INT:
//             content.type = 1;
//             break;
//         case knowledge_graph_msgs::msg::Content::FLOAT:
//             content.type = 2;
//             break;
//         case knowledge_graph_msgs::msg::Content::STRING:
//             content.type = 4;
//             break;

//         default:
//         RCLCPP_ERROR(rclcpp::get_logger("my_knowledge_graph"),"content type not recognized");
//             break;
//         }
  return content;
}

int whichType2(const knowledge_graph_msgs::msg::Property & prop)
{
  int i;

  switch (prop.value.type) {
    case knowledge_graph_msgs::msg::Content::BOOL:
      std::cout << "tipo bool" << std::endl;
      i = 0;
      break;
    case knowledge_graph_msgs::msg::Content::INT:
      std::cout << "tipo int" << std::endl;
      i = 1;
      break;
    case knowledge_graph_msgs::msg::Content::FLOAT:
      std::cout << "tipo float" << std::endl;
      i = 2;
      break;
    case knowledge_graph_msgs::msg::Content::STRING:
      std::cout << "tipo string" << std::endl;
      i = 4;
      break;

    default:
      RCLCPP_ERROR(rclcpp::get_logger("my_knowledge_graph"), "content type not recognized");
      break;
  }
  return i;
}
