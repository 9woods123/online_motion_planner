# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/woods/uuv/motion_planner_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/woods/uuv/motion_planner_ws/src/cmake-build-debug

# Utility rule file for clean_test_results_mav_trajectory_generation_ros.

# Include the progress variables for this target.
include mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/progress.make

mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros:
	cd /home/woods/uuv/motion_planner_ws/src/cmake-build-debug/mav_trajectory_generation/mav_trajectory_generation_ros && /usr/bin/python2 /opt/ros/melodic/share/catkin/cmake/test/remove_test_results.py /home/woods/uuv/motion_planner_ws/src/cmake-build-debug/test_results/mav_trajectory_generation_ros

clean_test_results_mav_trajectory_generation_ros: mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros
clean_test_results_mav_trajectory_generation_ros: mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/build.make

.PHONY : clean_test_results_mav_trajectory_generation_ros

# Rule to build all files generated by this target.
mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/build: clean_test_results_mav_trajectory_generation_ros

.PHONY : mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/build

mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/clean:
	cd /home/woods/uuv/motion_planner_ws/src/cmake-build-debug/mav_trajectory_generation/mav_trajectory_generation_ros && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/cmake_clean.cmake
.PHONY : mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/clean

mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/depend:
	cd /home/woods/uuv/motion_planner_ws/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/woods/uuv/motion_planner_ws/src /home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_trajectory_generation_ros /home/woods/uuv/motion_planner_ws/src/cmake-build-debug /home/woods/uuv/motion_planner_ws/src/cmake-build-debug/mav_trajectory_generation/mav_trajectory_generation_ros /home/woods/uuv/motion_planner_ws/src/cmake-build-debug/mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mav_trajectory_generation/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/depend

