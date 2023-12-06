//
// Created by woods on 2023/12/4.
//

#ifndef ONLINE_COVERAGE_PLANNER_H
#define ONLINE_COVERAGE_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <online_coverage_planner/tiling_map.h>


class online_coverage_planner{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher  planned_goal_pub;

    ros::Subscriber point_cloud_sub;
    ros::Subscriber robot_pose_sub;

    ros::Subscriber goal_sub;

    ros::Timer planner_timer;
    ros::Rate rate_;

    std::shared_ptr<TilingMap::tiling_map> tiling_map_ptr;
    
//  local msgs and datas
    geometry_msgs::PoseStamped current_pose;


//  local msgs and datas

//============================parameters===================================
    std::string robot_name_;
    float tiling_resolution;
    float bound_box_x_min;
    float bound_box_x_max;
    float bound_box_y_min;
    float bound_box_y_max;
    float bound_box_z_min;
    float bound_box_z_max;
    //============================parameters===================================




    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void planning_loop(const ros::TimerEvent& event);
    bool decideNextAreaToExplore();
    void initParams();

public:
    online_coverage_planner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

};

#endif //ONLINE_COVERAGE_PLANNER_H
