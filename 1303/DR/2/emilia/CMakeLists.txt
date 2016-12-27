cmake_minimum_required(VERSION 2.8.3)
project(emilia)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	gn_tokugava
)

generate_messages()

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(emilia src/main.cpp src/emilia.cpp)
target_link_libraries(emilia ${catkin_LIBRARIES})