cmake_minimum_required(VERSION 2.8.3)
project(construction_check)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensors
  planning
  constants
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES construction_check
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arc_test src/arc_main.cpp)
target_link_libraries(arc_test ${catkin_LIBRARIES})
add_dependencies(arc_test ${catkin_EXPORTED_TARGETS})
