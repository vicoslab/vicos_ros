cmake_minimum_required(VERSION 2.8.3)
project(localizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs sensor_msgs genmsg geometry_msgs visualization_msgs nodelet tf image_geometry eigen_conversions
)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

add_service_files(FILES Localize.srv)

generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs visualization_msgs)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(include
	${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_library(localizer src/localizer.cpp) 

add_executable(localizer_node src/localizer_node.cpp) 

ADD_DEPENDENCIES(localizer localizer_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(localizer ${catkin_LIBRARIES})

target_link_libraries(localizer_node ${catkin_LIBRARIES})

install(TARGETS localizer
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS localizer_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
