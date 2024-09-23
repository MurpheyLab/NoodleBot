cmake_minimum_required(VERSION 3.0.2)
project(noodlebot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  swimmer_command.msg
  swimmer_info.msg
  swimmer_reset.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime rospy sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/april_tag_repeater
  scripts/dummy_swimmer
  scripts/enjoy
  scripts/gui
  scripts/joint_repeater
  scripts/train
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
