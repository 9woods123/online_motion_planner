//
// Created by woods on 2023/7/4.
//

#ifndef TARGET_PROBABILITY_MAP_TP_MAP_SERVER_H
#define TARGET_PROBABILITY_MAP_TP_MAP_SERVER_H

#include "target_probability_map.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <detect_msgs/Detect.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>
#include <iostream>

#define SQ(x) ((x)*(x))

namespace tp_map{

    class tp_map_server
    {

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        std::shared_ptr<target_probability_map>  tp_map_;

        ros::Subscriber detect_sensor_sub_;
        ros::Subscriber sensors_input_sub;
        ros::Subscriber robot_pose_sub;
        ros::Publisher  pointcloud_pub;
        ros::Publisher marker_pub;
        ros::Timer sim_sensing_timer;
        ros::Timer mapping_timer;
        ros::Timer vis_timer_;
        ros::Rate rate_;

        geometry_msgs::PoseStamped current_pose;
        sensor_msgs::PointCloud2   sim_pointcloud_msg;
        detect_msgs::Detect detect_info;
        // Sensor msg;

        //params
        double target_occ_radius=0.5;
        double local_sensing_radius=25;
        double sensing_frequency=5; //HZ


    public:
        tp_map_server(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private,
                      const double map_resolution,
                      const double occur_threshold,
                      const double free_threshold);
        tp_map_server(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private);

        void sim_sensing_loop(const ros::TimerEvent& event);
        void visCallback(const ros::TimerEvent& event);
        void DetectCallback(const detect_msgs::Detect::ConstPtr& msg);
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void pubPointcloud(const pcl::PointCloud<pcl::PointXYZ> *pointcloud);
        bool updateProbabilityAtPosition(Eigen::Vector3d point,double observe_prob);
        bool updateProbBySensor(const detect_msgs::Detect* detect_info);
        void generateLocalSensing();
        bool getTPmap();

    };

}

#endif //TARGET_PROBABILITY_MAP_TP_MAP_SERVER_H
