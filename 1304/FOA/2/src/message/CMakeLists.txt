cmake_minimum_required(VERSION 2.8.3)
project(message)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	message_generation
)

add_message_files(
  FILES
  location.msg
)

generate_messages()

catkin_package()
