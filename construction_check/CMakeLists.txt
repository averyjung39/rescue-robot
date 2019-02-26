cmake_minimum_required(VERSION 2.8.3)
project(construction_check)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensors
  planning
  topics
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

add_executable(construction_check src/tof_main.cpp)
target_link_libraries(construction_check ${catkin_LIBRARIES})
add_dependencies(construction_check sensors planning topics)
