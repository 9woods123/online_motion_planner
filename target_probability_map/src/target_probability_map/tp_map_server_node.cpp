//
// Created by woods on 2023/7/5.
//
#include <target_probability_map/tp_map_server.h>
#include <ros/ros.h>


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "target_probability_map");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~"); //  using private node to update parameters.
    tp_map::tp_map_server tp_map_server(nh, nh_private);
    ros::spin();
    return 0;
}