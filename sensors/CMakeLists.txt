cmake_minimum_required(VERSION 2.8.3)
project(sensors)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  topics
)

add_message_files(
  FILES
  TimeOfFlight.msg
  Ultrasonic.msg
  Infrared.msg
  HallEffect.msg
  IMU.msg
  Encoder.msg
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
  include
)

add_executable(tof_sensor src/tof_sensor_main.cpp src/tof.cpp)
target_link_libraries(tof_sensor ${catkin_LIBRARIES})
add_dependencies(tof_sensor ${catkin_EXPORTED_TARGETS})

add_executable(ultrasonic_sensor src/ultrasonic_sensor_main.cpp src/GPIOClass.cpp)
target_link_libraries(ultrasonic_sensor ${catkin_LIBRARIES})
add_dependencies(ultrasonic_sensor ${catkin_EXPORTED_TARGETS})
