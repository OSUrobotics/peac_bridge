cmake_minimum_required(VERSION 2.8.3)
project(peac_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Location.msg
  Device.msg
  Control.msg
  ControlActuation.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ListLocations.srv
  ListDevices.srv
  GetDeviceInfo.srv
  UpdateControl.srv
  ReportControlActuation.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

install(PROGRAMS scripts/bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
