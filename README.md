# online_motion_planner



**online_motion_planner**  uses astar to find a feasible motion trajectory, and replan is done when collosion happened and horizon is reached.


**Prerequisites**

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. If not already done so, create a catkin workspace with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/):

```shell script
sudo apt-get install python-catkin-tools
mkdir -p ~/catkin_ws/src
git clone https://github.com/9woods123/uuv_sim.git
git clone https://github.com/ethz-asl/voxblox.git
git clone https://github.com/9woods123/online_motion_planner.git
cd ~/catkin_ws
catkin_make
```

## Run An Experiment

In order to launch a auv, firstly launch a gazebo world:

```
roslaunch uuv_gazebo_worlds ocean_waves.launch 
```

```
 roslaunch online_motion_planner eca9_a9_upload.launch
```

```
roslaunch online_motion_planner online_motion_planner.launch
```




Using  rviz 3d nav goal to give a gobal goal pose.  If you want to modify the initial height of 3d nav goal, you can look into the cpp in rviz-3d-nav-goal-tool.
![2023-06-15 14-45-49 的屏幕截图](https://github.com/9woods123/online_motion_planner/assets/78521063/d6435e44-a1cf-4d4c-b958-3584c2dd3765)

![2023-06-1![2023-06-15 14-46-36 的屏幕截图](https://github.com/9woods123/online_motion_planner/assets/78521063/52e6db95-6fff-4902-b5ee-c25feac618c7)
5 14-46-48 的屏幕截图](https://github.com/9woods123/online_motion_planner/assets/78521063/dae1cfba-0ba2-4775-a24b-543e2a04215c)

Some important log info can help you to work on this tools.

![2023-06-15 14-46-36 的屏幕截图](https://github.com/9woods123/online_motion_planner/assets/78521063/17fea9c7-86a1-46d5-9737-e2a538b0da9b)


