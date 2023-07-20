//
// Created by woods on 2023/7/5.
//
#include "target_detection_sim/target_detection_sim.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "target_detection_sim");
    ros::NodeHandle nh;


    //=========== Generate random target position and store in an octree for fast searching.
    static_target = generateRandomPoints(minX,maxX,minY,maxY,minZ,maxZ,targent_number);
    octree.setInputCloud(static_target);
    octree.addPointsFromInputCloud();
    //=========== Generate random target position and store in an octree for fast searching.


    pointcloud_pub=nh.advertise<sensor_msgs::PointCloud2>("sim_point_cloud", 1);
    targets_vis_pub= nh.advertise<visualization_msgs::MarkerArray>("targets/visualization_markers", 10);
    robot_pose_sub=nh.subscribe("robot_pose_topic",10, poseCallback);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0), octreeSearchCallback);
    ros::Timer fake_sensor = nh.createTimer(ros::Duration(0.1), sensorCallback);
    ros::Timer vis_timer = nh.createTimer(ros::Duration(1), visCallback);


    ros::spin();

    return 0;
}