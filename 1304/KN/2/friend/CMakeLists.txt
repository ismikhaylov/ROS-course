cmake_minimum_required(VERSION 2.8.3)
project(friend)

find_package(catkin REQUIRED COMPONENTS
  speaker
  roscpp
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(friend src/friend.cpp)
target_link_libraries(friend ${catkin_LIBRARIES})



