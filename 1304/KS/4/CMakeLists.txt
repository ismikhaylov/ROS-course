cmake_minimum_required(VERSION 2.8.3)
project(pathplanner)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(pathplanner src/pathplanner.cpp)

add_dependencies(pathplanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pathplanner
  ${catkin_LIBRARIES}
)