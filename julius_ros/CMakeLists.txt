cmake_minimum_required(VERSION 2.8.3)
project(julius_ros)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(recognizer src/recognizer.cpp) 

## Specify libraries to link a library or executable target against
target_link_libraries(recognizer ${catkin_LIBRARIES} julius sent pthread dl z pulse pulse-simple asound)

#############
## Install ##
#############

install(TARGETS recognizer
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


