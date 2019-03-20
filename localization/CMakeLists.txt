cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  constants
  messages
  sensors
)

add_message_files(
  FILES
  Pose.msg
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

add_executable(simple_localization src/localization_main.cpp src/SimpleLocalizer.cpp)
target_link_libraries(simple_localization ${catkin_LIBRARIES})
add_dependencies(simple_localization ${catkin_EXPORTED_TARGETS})
