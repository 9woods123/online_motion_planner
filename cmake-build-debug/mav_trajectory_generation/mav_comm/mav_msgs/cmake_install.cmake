# Install script for directory: /home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_msgs/msg" TYPE FILE FILES
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/msg/Actuators.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/msg/AttitudeThrust.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/msg/RateThrust.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/msg/RollPitchYawrateThrust.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/msg/TorqueThrust.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/msg/Status.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/msg/FilteredSensorData.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/msg/GpsWaypoint.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_msgs/cmake" TYPE FILE FILES "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/mav_trajectory_generation/mav_comm/mav_msgs/catkin_generated/installspace/mav_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/devel/include/mav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/devel/share/roseus/ros/mav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/devel/share/common-lisp/ros/mav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/devel/share/gennodejs/ros/mav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/devel/lib/python2.7/dist-packages/mav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/devel/lib/python2.7/dist-packages/mav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/mav_trajectory_generation/mav_comm/mav_msgs/catkin_generated/installspace/mav_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_msgs/cmake" TYPE FILE FILES
    "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/mav_trajectory_generation/mav_comm/mav_msgs/catkin_generated/installspace/mav_msgs-msg-extras.cmake"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/cmake/export_flags.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_msgs/cmake" TYPE FILE FILES
    "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/mav_trajectory_generation/mav_comm/mav_msgs/catkin_generated/installspace/mav_msgsConfig.cmake"
    "/home/woods/uuv/motion_planner_ws/src/cmake-build-debug/mav_trajectory_generation/mav_comm/mav_msgs/catkin_generated/installspace/mav_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_msgs" TYPE FILE FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mav_msgs" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_msgs/include/mav_msgs/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

