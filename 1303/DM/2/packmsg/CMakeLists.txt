cmake_minimum_required(VERSION 2.8.3)
project(packmsg)


find_package(catkin REQUIRED COMPONENTS 
             std_msgs
             message_generation
           )


add_message_files(
   FILES
   Secret_coords.msg

 )



generate_messages(
  #DEPENDENCIES
  # std_msgs  # Or other packages containing msgs
 )


catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES packmsg
#  CATKIN_DEPENDS other_catkin_pkg
  CATKIN_DEPENDS message_runtime 
#  DEPENDS system_lib
)





