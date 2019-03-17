cmake_minimum_required(VERSION 2.8.3)
project(planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ControlCommand.msg
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

add_executable(planning_node src/planning_main.cpp src/AStarPlanner.cpp src/RobotPath.cpp)
target_link_libraries(planning_node ${catkin_LIBRARIES})
add_dependencies(planning_node ${catkin_EXPORTED_TARGETS})
