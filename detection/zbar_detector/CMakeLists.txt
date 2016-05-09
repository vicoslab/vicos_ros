cmake_minimum_required(VERSION 2.8.3)
project(zbar_detector)

find_package(catkin REQUIRED COMPONENTS image_transport cv_bridge std_msgs genmsg)

add_message_files(DIRECTORY msg FILES Marker.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp message_runtime std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(barcode_detector src/main.cpp) 

add_dependencies(barcode_detector ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(barcode_detector ${catkin_LIBRARIES} zbar)

install(TARGETS barcode_detector
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


