cmake_minimum_required(VERSION 2.8.3)
project(facedetector)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs sensor_msgs image_transport cv_bridge genmsg
)

catkin_python_setup()

add_message_files(FILES Detection.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

install(PROGRAMS src/detector src/visualizer
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


