cmake_minimum_required(VERSION 2.8.3)
project(gn_tokugava)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	message_generation
)

add_message_files(
  FILES
  ReportMsg.msg
)

generate_messages()

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gn_tokugava src/main.cpp src/tokugava.cpp)
target_link_libraries(gn_tokugava ${catkin_LIBRARIES})