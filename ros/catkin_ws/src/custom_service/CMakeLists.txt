cmake_minimum_required(VERSION 3.5.1)
project(custom_service)

find_package(catkin REQUIRED
  message_generation
  std_msgs
)

add_service_files(
  FILES
  CustomService.srv
)

add_message_files(
  FILES
  CustomMessage.msg
)

generate_messages(
   DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS rospy
)
