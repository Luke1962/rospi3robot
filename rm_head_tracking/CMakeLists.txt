cmake_minimum_required(VERSION 2.8.3)
project(rm_head_tracking)

find_package(
  catkin REQUIRED COMPONENTS roscpp sensor_msgs message_generation
)
find_package(
  OpenCV REQUIRED
)

add_message_files(
  FILES
  Heads.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include/
  /usr/local/include/opencv2
  /usr/local/include/raspicam
  /usr/local/share/OpenCV
  /home/pi/opencv-3.0.0/include/opencv2

)

add_executable(rm_head_tracking_node 
  src/rm_head_tracking_node.cpp 
)


target_link_libraries(
	rm_head_tracking_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  raspicam
  raspicam_cv
  wiringPi
)
