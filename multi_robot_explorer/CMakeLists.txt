cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_explorer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  actionlib
  move_base_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs actionlib move_base_msgs tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/multi_robot_explorer_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(frontier_cells_test test/frontier_cells_test.cpp)
  add_test(NAME frontier_cells_test COMMAND frontier_cells_test)
endif()
