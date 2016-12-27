cmake_minimum_required(VERSION 2.8.3)
project(packpub)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS packmsg roscpp )




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES packpub
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(pSat src/pSat.cpp )
add_dependencies(pSat packmsg_gencpp) # ??
target_link_libraries(pSat ${catkin_LIBRARIES})






