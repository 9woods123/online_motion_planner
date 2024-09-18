
# Online Motion Planner

**Online Motion Planner** uses the A* algorithm to find feasible motion trajectories for Autonomous Underwater Vehicles (AUVs). The planner replans the trajectory when a collision is detected or the planning horizon is reached.

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
   sudo apt-get install libgoogle-glog-dev nvidia-cuda-toolkit
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

## Using RViz

Use the **RViz 3D Nav Goal** tool to set a global goal pose for the AUV. If you need to adjust the initial height of the 3D Nav Goal, you can modify the C++ code in the `rviz-3d-nav-goal-tool` package.


