cmake_minimum_required(VERSION 2.8.3)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  planning
  external
  constants
  sensors
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller src/controls_main.cpp src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller planning_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(test_turn_with_radius src/test_turn_with_radius.cpp)
target_link_libraries(test_turn_with_radius ${catkin_LIBRARIES})
add_dependencies(test_turn_with_radius planning_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(test_turn_on_spot src/test_turn_on_spot.cpp)
target_link_libraries(test_turn_on_spot ${catkin_LIBRARIES})
add_dependencies(test_turn_on_spot planning_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
