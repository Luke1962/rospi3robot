cmake_minimum_required(VERSION 2.8.3)
project(rm_scan)

find_package(
  catkin REQUIRED COMPONENTS roscpp sensor_msgs message_generation cv_bridge image_transport
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
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime cv_bridge image_transport
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

add_executable(rm_scan_node 
  src/rm_scan_node.cpp 
)


target_link_libraries(
	rm_scan_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  raspicam
  raspicam_cv
  wiringPi
)
