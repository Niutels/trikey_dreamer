cmake_minimum_required(VERSION 2.8.3)
project(trikey_dreamer)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS xacro
										roscpp
										gazebo_ros)


find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS xacro roscpp gazebo_ros
)


add_library(Trikey_Plugin_Test src/trikey_plugin.cpp)
target_link_libraries(Trikey_Plugin_Test ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(TrikeyDreamer_Plugin_Test src/trikey_dreamer_plugin.cpp)
target_link_libraries(TrikeyDreamer_Plugin_Test ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

