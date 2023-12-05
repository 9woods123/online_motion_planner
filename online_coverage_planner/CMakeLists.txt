cmake_minimum_required(VERSION 3.0.2)
project(online_coverage_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  detect_msgs
  geometry_msgs
  octomap_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## add libarary  tiling_map

cs_add_library(tiling_map 
${CMAKE_CURRENT_SOURCE_DIR}/src/tiling_map.cpp
)
target_link_libraries(tiling_map
        ${EIGEN3_LIBRARIES}
        )

## add libarary  online_coverage_planner
cs_add_library(online_coverage_planner 
${CMAKE_CURRENT_SOURCE_DIR}/src/online_coverage_planner.cpp
${CMAKE_CURRENT_SOURCE_DIR}/include/online_coverage_planner/online_coverage_planner.h
)
add_dependencies(online_coverage_planner tiling_map)
## online_coverage_planner using lib tiling_map
target_link_libraries(online_coverage_planner
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${PCL_LIBRARIES}
        tiling_map
        )


## exe online_coverage_planner_node 
cs_add_executable(online_coverage_planner_node 
${CMAKE_CURRENT_SOURCE_DIR}/src/online_coverage_planner_node.cpp
)
## exe online_coverage_planner using the lib online_coverage_planner
target_link_libraries(online_coverage_planner_node
online_coverage_planner )
add_dependencies(online_coverage_planner_node
 online_coverage_planner)

cs_install()
cs_export()

