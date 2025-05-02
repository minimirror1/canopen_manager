cmake_minimum_required(VERSION 3.0.2)
project(ros1_interface)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  std_srvs
)

## Package configuration
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    std_srvs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  src/canopen_node.py
  src/send_command.py
  src/motor_setting_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
) 