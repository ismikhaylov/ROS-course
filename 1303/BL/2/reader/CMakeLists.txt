cmake_minimum_required(VERSION 2.8.3)
project(reader)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 	
	message 
	std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(reader src/reader.cpp)
target_link_libraries(reader ${catkin_LIBRARIES})
add_dependencies(reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
