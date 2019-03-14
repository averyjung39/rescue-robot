cmake_minimum_required(VERSION 2.8.3)
project(objectives)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ActiveObjectives.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
