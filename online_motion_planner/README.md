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


Some important log info can help you to work on this tools.




sudo apt-get install libgoogle-glog-dev

sudo apt install nvidia-cuda-toolkit

pip3 install scipy
pip install scipy

catkin_make -DCMAKE_BUILD_TYPE=Release
