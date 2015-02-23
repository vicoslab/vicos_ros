cmake_minimum_required(VERSION 2.8.3)
project(zbar_ros)

find_package(catkin REQUIRED COMPONENTS image_transport cv_bridge std_msgs genmsg)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(DIRECTORY msg FILES Marker.msg)

generate_messages(DEPENDENCIES std_msgs)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_objects
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(barcode_detector src/main.cpp) 

add_dependencies(barcode_detector zbar_ros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(barcode_detector ${catkin_LIBRARIES} zbar)

#############
## Install ##
#############

install(TARGETS barcode_detector
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


