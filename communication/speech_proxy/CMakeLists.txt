cmake_minimum_required(VERSION 2.8.3)
project(speech_proxy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

find_package(Boost COMPONENTS thread iostreams REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories (${Boost_LIBRARY_DIRS})

add_executable(speech_proxy src/speech_server.cpp)

target_link_libraries(speech_proxy ${catkin_LIBRARIES} ${Boost_LIBRARIES})

