cmake_minimum_required(VERSION 2.8.3)
project(detection_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs sensor_msgs genmsg geometry_msgs visualization_msgs nodelet tf image_geometry cv_bridge
)

add_message_files(FILES Detection.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs visualization_msgs)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(include
	${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(overlay_visualizer src/overlay_visualizer.cpp) 
target_link_libraries(overlay_visualizer ${catkin_LIBRARIES})

add_executable(patch_visualizer src/patch_visualizer.cpp) 
target_link_libraries(patch_visualizer ${catkin_LIBRARIES})


