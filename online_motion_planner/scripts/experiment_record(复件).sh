#!/bin/bash

# 打开新标签页并执行指令的函数
function run_in_new_tab() {
    local cmd="$1"
    local title="$2"
    gnome-terminal --tab --title="$title" -- bash -c "$cmd; exec bash"
}

# 在同一个终端中依次打开三个标签页，并分别执行指令

# 打开第一个标签页并执行第一个指令
run_in_new_tab "roslaunch uuv_gazebo_worlds ocean_waves.launch" "ocean_waves.launch"

# 等待5秒
sleep 5

# 打开第二个标签页并执行第二个指令
run_in_new_tab "roslaunch online_motion_planner online_motion_planner.launch" "online_motion_planner.launch"

# 等待5秒
sleep 5

# 打开第三个标签页并执行第三个指令
run_in_new_tab "python3 experiment_record.py" "experiment_record.py"
