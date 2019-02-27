cmake_minimum_required(VERSION 2.8.3)
project(sensors)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
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

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
