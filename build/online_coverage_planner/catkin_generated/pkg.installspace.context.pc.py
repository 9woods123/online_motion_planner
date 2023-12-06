# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "catkin_simple;detect_msgs;geometry_msgs;octomap_msgs;roscpp;rospy;sensor_msgs;std_msgs;tf;visualization_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltiling_map;-lonline_coverage_planner".split(';') if "-ltiling_map;-lonline_coverage_planner" != "" else []
PROJECT_NAME = "online_coverage_planner"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
