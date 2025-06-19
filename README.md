
# Online Motion Planner

**Online Motion Planner** is a trajectory planning framework that uses the A* algorithm to generate a 3d feasible motion trajectories for mobile robots. The planner performs online replanning whenever a potential collision is detected or the planning horizon is reached.

This method is general-purpose and can be applied to various robotic platforms. We have applied it to torpedo-shaped AUVs in a 3D underwater environment, and the method has been peer-reviewed and published at **IROS 2025**.

If you find this work helpful, please consider citing:

```bibtex
@inproceedings{nine2025omp,
  author    = {Woods Nine and Others},
  title     = {Online Motion Planning for AUVs Using 3D Multibeam Sonar in Unknown Environments},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2025}
}
```


## Prerequisites

1. **Install ROS**  
   Follow the [ROS Installation guide](http://wiki.ros.org/ROS/Installation) to install ROS (Desktop-Full recommended).

2. **Set Up Catkin Workspace**  
   Install `catkin tools` and create a workspace:

   ```bash
   sudo apt-get install python-catkin-tools
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```

3. **Clone Repositories and Install Dependencies**  
   Clone the required repositories and install necessary dependencies:

   ```bash
   git clone https://github.com/9woods123/online_motion_planner.git
   vcs import . < online_motion_planner/online_motion_planner/omp_https.rosinstall --recursive
   sudo apt-get update && sudo apt-get install -y \
      libgoogle-glog-dev \
      nvidia-cuda-toolkit \
      python3-vcstool \
      python3-catkin-tools \
      ros-$ROS_DISTRO-cmake-modules \
      protobuf-compiler \
      autoconf \
      git \
      rsync \
      ros-$ROS_DISTRO-mavros-msgs \
      ros-$ROS_DISTRO-tf2-sensor-msgs \
      ros-$ROS_DISTRO-nlopt \
      ros-$ROS_DISTRO-octomap-msgs \
      ros-$ROS_DISTRO-ompl \
      ros-$ROS_DISTRO-octomap-server \
      libompl-dev

   pip3 install scipy

   ```

4. **Build the Workspace**  
   Compile the workspace in Release mode:

   ```bash
   cd ~/catkin_ws
   catkin_make -DCMAKE_BUILD_TYPE=Release
   ```

## Run An Experiment

1. **Launch Gazebo World**  
   Start by launching a Gazebo world:

   ```bash
   roslaunch uuv_gazebo_worlds ocean_waves.launch
   ```

3. **Launch the Motion Planner**  
   Run the motion planner:

   ```bash
   roslaunch online_motion_planner online_motion_planner.launch
   ```
   
![2025-06-19 23-29-30 的屏幕截图](https://github.com/user-attachments/assets/c4a82422-feeb-492e-aa99-9b6bdfc5f075)

## Using RViz

Use the **RViz 3D Nav Goal** tool to set a global goal pose for the AUV. If you need to adjust the initial height of the 3D Nav Goal, you can modify the C++ code in the `rviz-3d-nav-goal-tool` package.


