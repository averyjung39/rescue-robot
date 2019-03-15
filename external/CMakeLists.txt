cmake_minimum_required(VERSION 2.8.3)
project(external)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES wiringPi IMU tof
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(wiringPi
    src/wiringPi/wiringPi.cpp
    src/wiringPi/piHiPri.cpp
    src/wiringPi/softPwm.cpp
    src/wiringPi/softTone.cpp
    src/wiringPi/wiringPiI2C.cpp
)

target_link_libraries(wiringPi ${catkin_LIBRARIES})

install(TARGETS wiringPi
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(IMU
    src/IMU/IMUSensor.cpp
)
target_link_libraries(IMU ${catkin_LIBRARIES})
install(TARGETS IMU
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(tof
    src/tof/VL53L0X.cpp
)

target_link_libraries(tof ${catkin_LIBRARIES})

install(TARGETS tof
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
