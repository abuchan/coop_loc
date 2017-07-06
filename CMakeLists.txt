cmake_minimum_required(VERSION 2.8.3)
project(coop_loc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
	FILES
  IMUwithMoving.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(DEPENDS roscpp std_msgs geometry_msgs genmsg
    LIBRARIES ${PROJECT_NAME})

include_directories(
  ${catkin_INCLUDE_DIRS}
)