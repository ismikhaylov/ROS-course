cmake_minimum_required(VERSION 3.6)
project(RobotsBattle)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp roscpp tf gazebo_ros
        std_msgs message_generation)

find_package(Boost COMPONENTS chrono thread filesystem REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})




set(SOURCE_FILES1
        heder/robot/warior/RobotGuardian.h
        source/robot/warior/RobotGuardian.cpp
        source/robot/warior/RobotCommander.cpp
        heder/robot/warior/RobotCommander.h
        heder/message/CommandInformation.h
        source/r1_node.cpp
        source/robot/base/RobotBase.cpp heder/robot/base/RobotBase.h
        source/robot/warior/RobotArcher.cpp heder/robot/warior/RobotArcher.h
        source/robot/factory/RobotFactory.cpp
        heder/robot/factory/RobotFactory.h
        source/robot/item/Item.cpp heder/robot/item/Item.h source/robot/base/Service.cpp heder/robot/base/Service.h source/robot/item/weapon/Weapon.cpp heder/robot/item/weapon/Weapon.h source/robot/item/weapon/Bow.cpp heder/robot/item/weapon/Bow.h
        source/robot/item/projectile/Arrow.cpp heder/robot/item/projectile/Arrow.h
        source/robot/item/projectile/Projectile.cpp heder/robot/item/projectile/Projectile.h source/thread/ThreadPool.cpp heder/thread/ThreadPool.h source/robot/item/weapon/Sword.cpp heder/robot/item/weapon/Sword.h)
add_executable(r1 ${SOURCE_FILES1})
target_link_libraries(r1 ${catkin_LIBRARIES})
target_link_libraries(r1 ${Boost_LIBRARIES})

set(SOURCE_FILES2
        heder/robot/warior/RobotGuardian.h
        source/robot/warior/RobotGuardian.cpp
        source/robot/warior/RobotCommander.cpp
        heder/robot/warior/RobotCommander.h
        heder/message/CommandInformation.h
        source/r2_node.cpp
        source/robot/base/RobotBase.cpp heder/robot/base/RobotBase.h
        source/robot/warior/RobotArcher.cpp heder/robot/warior/RobotArcher.h
        source/robot/factory/RobotFactory.cpp
        heder/robot/factory/RobotFactory.h
        source/robot/item/Item.cpp heder/robot/item/Item.h source/robot/base/Service.cpp heder/robot/base/Service.h source/robot/item/weapon/Weapon.cpp heder/robot/item/weapon/Weapon.h source/robot/item/weapon/Bow.cpp heder/robot/item/weapon/Bow.h
        source/robot/item/projectile/Arrow.cpp heder/robot/item/projectile/Arrow.h
        source/robot/item/projectile/Projectile.cpp heder/robot/item/projectile/Projectile.h source/thread/ThreadPool.cpp heder/thread/ThreadPool.h source/robot/item/weapon/Sword.cpp heder/robot/item/weapon/Sword.h)
add_executable(r2 ${SOURCE_FILES2})
target_link_libraries(r2 ${catkin_LIBRARIES})
target_link_libraries(r2 ${Boost_LIBRARIES})


