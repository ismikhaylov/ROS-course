cmake_minimum_required(VERSION 2.8.3)
project(lost_robot)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    visualization_msgs
    tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lost_robot src/lost_robot.cpp)
add_dependencies(lost_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lost_robot ${catkin_LIBRARIES})
