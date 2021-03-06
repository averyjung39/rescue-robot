cmake_minimum_required(VERSION 2.8.3)
project(sensors)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  constants
  external
)

add_message_files(
  FILES
  Distance.msg
  Ultrasonic.msg
  Photodiode.msg
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
  include
)

add_executable(tof_sensor src/tof_sensor_main.cpp)
target_link_libraries(tof_sensor ${catkin_LIBRARIES})
add_dependencies(tof_sensor ${catkin_EXPORTED_TARGETS})

# add_executable(ultrasonic_sensor src/ultrasonic_sensor_main.cpp)
# target_link_libraries(ultrasonic_sensor ${catkin_LIBRARIES})
# add_dependencies(ultrasonic_sensor ${catkin_EXPORTED_TARGETS})

add_executable(photodiode_sensor src/photodiode_sensor_main.cpp)
target_link_libraries(photodiode_sensor ${catkin_LIBRARIES})
add_dependencies(photodiode_sensor ${catkin_EXPORTED_TARGETS})

add_executable(hall_effect_sensor src/hall_effect_sensor_main.cpp)
target_link_libraries(hall_effect_sensor ${catkin_LIBRARIES})
add_dependencies(hall_effect_sensor ${catkin_EXPORTED_TARGETS})

add_executable(encoder src/encoder_main.cpp src/MotorEncoder.cpp)
target_link_libraries(encoder ${catkin_LIBRARIES})
add_dependencies(encoder ${catkin_EXPORTED_TARGETS})

add_executable(imu_sensor src/imu_sensor_main.cpp)
target_link_libraries(imu_sensor ${catkin_LIBRARIES})
add_dependencies(imu_sensor ${catkin_EXPORTED_TARGETS})
