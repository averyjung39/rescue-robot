cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  constants
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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(particle_filter src/localization_main.cpp src/ParticleFilter.cpp)
target_link_libraries(particle_filter ${catkin_LIBRARIES})
add_dependencies(particle_filter ${catkin_EXPORTED_TARGETS})
