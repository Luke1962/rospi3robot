cmake_minimum_required(VERSION 2.8.3)
project(rm_robot)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp geometry_msgs tf)
find_library(
  espeak REQUIRED
  wiringPi REQUIRED
  rt REQUIRED
  pthread REQUIRED
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS std_msgs roscpp espeak geometry_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${espeak_INCLUDE_DIRS}
)

add_executable(rm_robot_node src/rm_robot_node.cpp)

target_link_libraries(rm_robot_node
  ${catkin_LIBRARIES}
  espeak wiringPi rt pthread
)
