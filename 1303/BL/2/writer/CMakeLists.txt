cmake_minimum_required(VERSION 2.8.3)
project(writer)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	message 
	std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(writer src/writer.cpp)
target_link_libraries(writer ${catkin_LIBRARIES})
add_dependencies(writer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
