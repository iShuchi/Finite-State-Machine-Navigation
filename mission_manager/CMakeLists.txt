cmake_minimum_required(VERSION 3.8)
project(mission_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)  
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_executable(mission_manager_node
  src/mission_manager_node.cpp)

ament_target_dependencies(mission_manager_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp)

target_link_libraries(mission_manager_node yaml-cpp)

install(TARGETS mission_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
