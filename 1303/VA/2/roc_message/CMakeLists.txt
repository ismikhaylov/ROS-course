cmake_minimum_required(VERSION 2.8.3)
project(roc_message)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(
   FILES
   Messager.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sputnik_message
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
