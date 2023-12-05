//
// Created by woods on 2023/12/4.
//

#include "online_coverage_planner/online_coverage_planner.h"


online_coverage_planner::online_coverage_planner(const ros::NodeHandle& nh,
                                                 const ros::NodeHandle& nh_private)

    : nh_(nh), nh_private_(nh_private), rate_(10.0) {

    initParams();

    robot_pose_sub=nh_.subscribe("robot_pose_topic",10, &online_coverage_planner::poseCallback,this);

    planned_goal_pub = nh_.advertise<geometry_msgs::PoseStamped>(
                                "planned_goal_topic", 1,true);

    planner_timer=nh_.createTimer(rate_.expectedCycleTime(),
                                         &online_coverage_planner::planning_loop,this);
    

    tiling_map_ptr=std::make_shared<tiling_map>(
        online_coverage_planner::tiling_resolution,
        online_coverage_planner::bound_box_x_min,
        online_coverage_planner::bound_box_x_max,
        online_coverage_planner::bound_box_y_min,
        online_coverage_planner::bound_box_y_max);

}

void online_coverage_planner::initParams() {

    nh_private_.param<std::string>("robot_name", robot_name_, "/eca_a9");

    nh_private_.param<float>("tiling_resolution", tiling_resolution, 2);
    nh_private_.param<float>("bound_box_x_min", bound_box_x_min, 2);
    nh_private_.param<float>("bound_box_x_max", bound_box_x_max, 2);
    nh_private_.param<float>("bound_box_y_min", bound_box_y_min, 2);
    nh_private_.param<float>("bound_box_y_max", bound_box_y_max, 2);
    nh_private_.param<float>("bound_box_z_min", bound_box_z_min, 2);
    nh_private_.param<float>("bound_box_z_max", bound_box_z_max, 2);

}

void online_coverage_planner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

bool online_coverage_planner::decideNextAreaToExplore(){

    // TODO ,input: tiling_map, 

}

void online_coverage_planner::planning_loop(const ros::TimerEvent &event) {
    


}










