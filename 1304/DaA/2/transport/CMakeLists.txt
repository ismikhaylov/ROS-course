cmake_minimum_required(VERSION 2.8.3)
project(lab2_transport)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

add_message_files(
   FILES
   Message.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
CATKIN_DEPENDS message_runtime
)