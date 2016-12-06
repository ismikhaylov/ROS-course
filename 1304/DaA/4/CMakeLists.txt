cmake_minimum_required(VERSION 2.8.3)
project(my_robot)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
)

catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(my_robot src/my_robot.cpp)

add_dependencies(my_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(my_robot
  ${catkin_LIBRARIES}
)