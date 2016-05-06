cmake_minimum_required(VERSION 2.8.3)
project(costmap_layers)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
  std_msgs
  sensor_msgs
  genmsg
  geometry_msgs 
)

add_message_files(FILES Wall.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(costmap_layers src/virtual_wall_layer.cpp)
add_dependencies(costmap_layers ${${PROJECT_NAME}_EXPORTED_TARGETS})

