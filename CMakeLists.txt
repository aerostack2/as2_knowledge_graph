cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME my_graph)
project(${PROJECT_NAME})

set(CMAKE_BUILD_TYPE RelWithDebInfo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  backward_ros
  knowledge_graph
  knowledge_graph_msgs
  rosidl_default_generators
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include/${PROJECT_NAME}
  
)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/CreateNode.srv
  srv/CreateEdge.srv
  srv/RemoveNode.srv
  srv/RemoveEdge.srv 
  DEPENDENCIES knowledge_graph_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_export_dependencies(rosidl_default_runtime)

add_executable(new_node_server src/new_node_server.cpp)
ament_target_dependencies(new_node_server ${PROJECT_DEPENDENCIES})
target_link_libraries(new_node_server "${cpp_typesupport_target}")

add_executable(new_edge_server src/new_node_server.cpp)
ament_target_dependencies(new_edge_server ${PROJECT_DEPENDENCIES})
target_link_libraries(new_edge_server "${cpp_typesupport_target}")

add_executable(remove_node_server src/remove_node_server.cpp)
ament_target_dependencies(remove_node_server ${PROJECT_DEPENDENCIES})
target_link_libraries(remove_node_server "${cpp_typesupport_target}")

add_executable(remove_edge_server src/remove_edge_server.cpp)
ament_target_dependencies(remove_edge_server ${PROJECT_DEPENDENCIES})
target_link_libraries(remove_edge_server "${cpp_typesupport_target}")

install(  TARGETS 
  new_node_server
  new_edge_server
  remove_node_server
  remove_edge_server
  DESTINATION lib/${PROJECT_NAME}
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()