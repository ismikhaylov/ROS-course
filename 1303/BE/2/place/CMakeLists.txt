cmake_minimum_required(VERSION 2.8.3)
project(place)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
)

add_message_files(
  FILES
  Phrase.msg
)

generate_messages()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(place src/place.cpp)
target_link_libraries(place ${catkin_LIBRARIES})