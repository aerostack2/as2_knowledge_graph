cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME my_graph)
project(${PROJECT_NAME})

set(CMAKE_BUILD_TYPE Debug)

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
  DEPENDENCIES knowledge_graph_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_export_dependencies(rosidl_default_runtime)

add_executable(new_server src/new_server.cpp)
ament_target_dependencies(new_server ${PROJECT_DEPENDENCIES})
target_link_libraries(new_server "${cpp_typesupport_target}")

add_executable(new_client src/new_client.cpp)
ament_target_dependencies(new_client ${PROJECT_DEPENDENCIES})
target_link_libraries(new_client "${cpp_typesupport_target}")



install(  TARGETS 
  new_server
  new_client

  DESTINATION lib/${PROJECT_NAME}
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()