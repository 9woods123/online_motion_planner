cmake_minimum_required(VERSION 3.0.2)
project(detect_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation  # Add this line to include message_generation package
)

## Declare ROS messages, services, and actions
add_message_files(
  FILES
  Detect.msg
  # Add more custom message files if needed
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime  # Add message_runtime dependency here
)

## Build your message package
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add any additional executables, libraries, or dependencies you need for your package
