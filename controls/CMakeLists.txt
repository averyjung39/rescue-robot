cmake_minimum_required(VERSION 2.8.3)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
