cmake_minimum_required(VERSION 2.8.3)
project(mapperception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensors
  localization
)

add_message_files(
  FILES
  MapRow.msg
  Map.msg
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

add_executable(mapping src/mapping_main.cpp src/Mapper.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES})
add_dependencies(mapping sensors localization)
