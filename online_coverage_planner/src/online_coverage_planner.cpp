//
// Created by woods on 2023/12/4.
//

#include "online_coverage_planner/online_coverage_planner.h"


online_coverage_planner::online_coverage_planner(const ros::NodeHandle& nh,
                                                 const ros::NodeHandle& nh_private)

    : nh_(nh), nh_private_(nh_private), rate_(1.0) {

    initParams();

    robot_pose_sub=nh_.subscribe("robot_pose_topic",10, &online_coverage_planner::poseCallback,this);

    planned_goal_pub = nh_.advertise<geometry_msgs::PoseStamped>(
                                "planned_goal_topic", 1,true);

    planner_timer=nh_.createTimer(rate_.expectedCycleTime(),
                                         &online_coverage_planner::planning_loop,this);
    

    tiling_map_ptr=std::make_shared<TilingMap::tiling_map>(
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


    // std::shared_ptr<TilingMap::TilingMapHashmap> tilingMapHashmap=tiling_map_ptr->getTilingMapHashmap();
    std::shared_ptr<TilingMap::TilingMapHashmap> tilingMapHashmap=
    std::make_shared<TilingMap::TilingMapHashmap>(*tiling_map_ptr->getTilingMapHashmap());

    // 遍历hashmap，找到价值最高的栅格
    // std::cout <<"tilingMapHashmap size= "<< tilingMapHashmap->size()<<std::endl;

    Eigen::Vector3d robot_pose(current_pose.pose.position.x,
                                                            current_pose.pose.position.y,
                                                            current_pose.pose.position.z);

    TilingMap::tilingGrid grid_max_p;
    float max_value=-999;
    
    Eigen::Vector3i robot_index =tiling_map_ptr->PointToIndex(robot_pose);

    for (const auto& entry : *tilingMapHashmap) {
            
            if(robot_index==entry.first){continue;}

            float distance=abs(robot_index.x()-entry.first.x())+abs(robot_index.y()-entry.first.y());            // city distance
            float value=entry.second.potential/distance;

            if( value >max_value){
                max_value=value;
                grid_max_p=entry.second;
            }

    }

    // std::cout<<grid_max_p.index_x<<", "<<grid_max_p.index_y<<" p="<<grid_max_p.potential<<std::endl;

}


void online_coverage_planner::planning_loop(const ros::TimerEvent &event) {


// TODO 更新地图，此处后续应该改为更具传感器数据更新地图，这里用robotPose是因为
// 没想好传感器的具体形式，只能假设随着机器人移动，传感器数据是覆盖机器人pose周围的一种扫描。

Eigen::Vector3d robot_pose(current_pose.pose.position.x,
                                                            current_pose.pose.position.y,
                                                            current_pose.pose.position.z);
tiling_map_ptr->updateMapbyRobotPose(robot_pose);

decideNextAreaToExplore();

}










