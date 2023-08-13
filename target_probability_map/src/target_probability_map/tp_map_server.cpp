//
// Created by woods on 2023/7/4.
//
#include "target_probability_map/tp_map_server.h"

namespace tp_map {


    tp_map_server::tp_map_server(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                                 const double map_resolution, const double occur_threshold,
                                 const double free_threshold):
                                 nh_(nh), nh_private_(nh_private),rate_(10)
    {
        pointcloud_pub=nh_.advertise<sensor_msgs::PointCloud2>("sim_point_cloud", 1, true);
        robot_pose_sub=nh_.subscribe("robot_pose_topic",10, &tp_map_server::poseCallback,this);
        detect_sensor_sub_=nh_.subscribe("detect_info_topic",10, &tp_map_server::DetectCallback,this);
        vis_timer_ = nh_.createTimer(ros::Duration(0.25), &tp_map_server::visCallback, this);
        marker_pub=nh_.advertise<visualization_msgs::MarkerArray>("map_markers", 1);
        tp_map_.reset(new target_probability_map(map_resolution,occur_threshold,free_threshold));


        //set params;
        nh_private_.param<float>("collision_radius", collision_radius, 5);
        tp_map_->setCollisionRadius(collision_radius);
    }

    void tp_map_server::DetectCallback(const detect_msgs::Detect::ConstPtr &msg) {
        detect_info=*msg;
        updateProbBySensor(&detect_info);
    }

    std::shared_ptr<target_probability_map> tp_map_server::getmapPtr() {
        return tp_map_;
    }

    void tp_map_server::visCallback(const ros::TimerEvent &event) {

        // TODO now I maker the target by trav

        target_probability_map::OctreeType* octreePTR=tp_map_->getOctree();
        // Assuming you have a map stored in a suitable data structure like tp_hashmap

        // Create the MarkerArray message
        visualization_msgs::MarkerArray marker_array_msg;
        int point_id=0;
        for (const auto& entry : octreePTR->getInputCloud()->points) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world"; // Set the frame_id to match your map frame
            marker.header.stamp = ros::Time::now();
            marker.ns = "map_markers"; // Namespace to group your markers
            marker.id = point_id++; // Unique ID for each marker, can be index or hash value

            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = entry.x; // Set the marker position based on your map data
            marker.pose.position.y = entry.y;
            marker.pose.position.z = entry.z;
            //TODO using ros param, to set radius.
            marker.scale.x = 2*collision_radius; // Set the marker size based on your map data
            marker.scale.y = 2*collision_radius;
            marker.scale.z = 2*collision_radius;
            marker.color.r = 1.0; // Set the marker color based on your map data
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = entry.probability; // Alpha channel for transparency

            // Add the marker to the marker_array_msg
            marker_array_msg.markers.push_back(marker);
        }

        // Publish the MarkerArray message
        marker_pub.publish(marker_array_msg);

    }

    bool tp_map_server::updateProbBySensor(const detect_msgs::Detect *detect_info) {

        int info_size=detect_info->probability.size();
        for(int i =0;i<info_size;i++)
        {
            Eigen::Vector3d point(detect_info->x[i],detect_info->y[i],detect_info->z[i]);
            updateProbabilityAtPosition(point,detect_info->probability[i]);
        }
    }

    void tp_map_server::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pose = *msg;
    }

    bool tp_map_server::updateProbabilityAtPosition(Eigen::Vector3d point, double observe_prob){
        tp_map_->updateProbabilityAtPosition(point,observe_prob);
    }

    float tp_map_server::getCollisionRadius() {
        return collision_radius;
    }


}