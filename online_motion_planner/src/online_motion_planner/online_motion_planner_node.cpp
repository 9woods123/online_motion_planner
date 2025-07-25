//
// Created by Tianyou Yu (woods) on 2023-05-21.
// Contact: tianyouyu9@163.com
//

#include <online_motion_planner/online_motion_planner.h>
#include <ros/ros.h>


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "online_motion_planner");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~"); //  using private node to update parameters.
    online_motion_planner online_motion_planner(nh, nh_private);
    ros::spin();
    return 0;
}
