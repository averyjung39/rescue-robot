cmake_minimum_required(VERSION 2.8.3)
project(objectives)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  external
  constants
  mapperception
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
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(objectives src/objectives_main.cpp src/ObjectiveManager.cpp)
target_link_libraries(objectives ${catkin_LIBRARIES})
add_dependencies(objectives ${catkin_EXPORTED_TARGETS})
