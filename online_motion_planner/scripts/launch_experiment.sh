#!/bin/bash

# 在同一个终端中打开三个标签页，并分别执行指令

# 打开第一个标签页并执行第一个指令
gnome-terminal --tab --title="ocean_waves.launch" -- bash -c "conda init;conda deactivate; roslaunch uuv_gazebo_worlds ocean_waves.launch; exec bash"

# 等待5秒
sleep 5

# 打开第二个标签页并执行第二个指令
gnome-terminal --tab --title="online_motion_planner.launch" -- bash -c "conda init;conda deactivate; roslaunch online_motion_planner online_motion_planner.launch; exec bash"

# 等待5秒
sleep 5

# 打开第三个标签页并执行第三个指令
gnome-terminal --tab --title="experiment_record.py" -- bash -c "conda init;conda deactivate; python3 experiment_record.py; exec bash"
