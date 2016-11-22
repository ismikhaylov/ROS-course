cmake_minimum_required(VERSION 2.8.3)
project(multibot_commander)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(multibot_commander src/commander.cpp)

add_dependencies(multibot_commander ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(multibot_commander
  ${catkin_LIBRARIES}
)
