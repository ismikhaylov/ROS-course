cmake_minimum_required(VERSION 3.6)
project(lab5)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp roscpp tf gazebo_ros)

include_directories(include ${catkin_INCLUDE_DIRS})



set(SOURCE_FILES1 source/r1_node.cpp source/robot/RobotBase.cpp heder/robot/RobotBase.h source/robot/RobotLost.cpp heder/robot/RobotLost.h source/robot/RobotHelper.cpp heder/robot/RobotHelper.h)
add_executable(r1 ${SOURCE_FILES1})
target_link_libraries(r1 ${catkin_LIBRARIES})


set(SOURCE_FILES2 source/r2_node.cpp source/robot/RobotBase.cpp heder/robot/RobotBase.h source/robot/RobotLost.cpp heder/robot/RobotLost.h source/robot/RobotHelper.cpp heder/robot/RobotHelper.h)
add_executable(r2 ${SOURCE_FILES2})
target_link_libraries(r2 ${catkin_LIBRARIES})
