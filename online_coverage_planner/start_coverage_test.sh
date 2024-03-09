#!/bin/bash

# 启动第一个 roslaunch
gnome-terminal --tab --title="Launch 1" --command="
bash -c 'roslaunch uuv_gazebo_worlds ocean_waves.launch; exec bash'"
sleep 1  # 等待 5 秒
# 启动第二个 roslaunch
gnome-terminal --tab --title="Launch 2" --command="
bash -c 'roslaunch online_motion_planner online_motion_planner.launch; exec bash'"
sleep 1  # 等待 5 秒

# 启动第三个 roslaunch
gnome-terminal --tab --title="Launch 3" --command="
bash -c 'roslaunch online_coverage_planner online_coverage_planner.launch; exec bash'"
