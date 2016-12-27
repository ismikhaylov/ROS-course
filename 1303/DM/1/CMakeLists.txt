cmake_minimum_required(VERSION 2.8.3)
project(pubpack)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pubpack
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(publisher src/publisher_tsim.cpp)
target_link_libraries(publisher  ${catkin_LIBRARIES})





