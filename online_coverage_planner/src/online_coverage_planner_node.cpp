//
// Created by woods on 2023/12/4.
//
//
// Created by woods on 23-5-21.
//
#include "online_coverage_planner/online_coverage_planner.h"
#include <ros/ros.h>


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "online_coverage_planner");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~"); //  using private node to update parameters.

    online_coverage_planner online_coverage_planner_node(nh, nh_private);
    ros::spin();
    return 0;
}
