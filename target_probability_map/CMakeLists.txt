
cmake_minimum_required(VERSION 2.8.3)
project(target_probability_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()



find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        )

set(SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/target_probability_map/target_probability_map.cpp
        #Boris Lau, Christoph Sprunk, Wolfram Burgard
        )

set(HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/target_probability_map/target_probability_map.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/target_probability_map/target_probability_voxel.h

        )


## OPEN MOTION PLANNING LIBRARY
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
#cs_add_executable(my_node ${SOURCES})

# 添加库文件

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        )

cs_install()
cs_export()
