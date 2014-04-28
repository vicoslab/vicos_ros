cmake_minimum_required(VERSION 2.8.3)
project(facedetector)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs sensor_msgs image_transport cv_bridge genmsg geometry_msgs visualization_msgs tf
)

catkin_python_setup()

add_message_files(FILES Detection.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs visualization_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

install(PROGRAMS src/detector src/visualizer src/mapper
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

