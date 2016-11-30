cmake_minimum_required(VERSION 2.8.3)
project(irrlicht)

find_package(catkin REQUIRED COMPONENTS
  move_base
  roscpp
  tf
  roslib
  nav_msgs
  sensor_msgs
)

find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(Irrlicht REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${GLU_INCLUDE_DIRS}
  ${Irrlicht_INCLUDE_DIRS}
)

add_library(irrlicht STATIC IMPORTED)

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(sauron src/sauron.cpp)
target_link_libraries(sauron ${catkin_LIBRARIES})

add_executable(scene src/scene.cpp)
target_link_libraries(scene ${catkin_LIBRARIES} ${OpenGL_LIBRARIES} ${GLU_LIBRARIES} Irrlicht)
