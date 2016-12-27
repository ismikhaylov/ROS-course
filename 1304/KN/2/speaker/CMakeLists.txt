cmake_minimum_required(VERSION 2.8.3)
project(speaker)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
)

add_message_files(
  FILES
  Speach.msg
)

generate_messages()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(speaker src/speaker.cpp)
target_link_libraries(speaker ${catkin_LIBRARIES})


