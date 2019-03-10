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
    LIBRARIES wiringPi 
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
)

target_link_libraries(wiringPi ${catkin_LIBRARIES})

install(TARGETS wiringPi
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_library(tof
#     src/tof/core/vl53l0x_api.cpp
#     src/tof/core/vl53l0x_api_calibration.cpp
#     src/tof/core/vl53l0x_api_core.cpp
#     src/tof/core/vl53l0x_api_ranging.cpp
#     src/tof/core/vl53l0x_api_strings.cpp
#     src/tof/platform/vl53l0x_platform.cpp
#     src/tof/platform/vl53l0x_platform_log.cpp
#     src/tof/platform/vl53l0x_i2c_win_serial_comms.cpp
# )

# target_link_libraries(tof ${catkin_LIBRARIES})

# install(TARGETS tof
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
