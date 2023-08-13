#ifndef TARGET_DETECTION_SIM_H
#define TARGET_DETECTION_SIM_H

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <Eigen/Core>
#include <pcl/octree/octree_search.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/MarkerArray.h>
#include "chrono"
#include "random"
#include <detect_msgs/Detect.h>

class target_detection_sim {

public:
    target_detection_sim(ros::NodeHandle nh,ros::NodeHandle nh_private);

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void sensorCallback(const ros::TimerEvent& event);
    void sonnarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void visCallback(const ros::TimerEvent& event);
    void octreeSearchCallback(const ros::TimerEvent& event);
    void generateLocalSensing(Eigen::Vector3d target_point);
    void generateLocalSensing(std::vector<Eigen::Vector3d> target_points,
                              std::vector<float> target_dists_);
    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void pubPointcloud(const pcl::PointCloud<pcl::PointXYZ>* pointcloud);
    void pubDetectInfo();
    bool lookforTransform(std::string target_frame,std::string source_frame,
                          Eigen::Affine3d& transformEigen);
    pcl::PointCloud<pcl::PointXYZ>::Ptr generateRandomPoints(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, size_t n);

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber robot_pose_sub_;
    ros::Subscriber sonnar_sub_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher targets_vis_pub_;
    ros::Publisher detect_pub_;

    ros::Timer octree_search_timer_;
    ros::Timer sensor_timer_;
    ros::Timer vis_timer_;

    geometry_msgs::PoseStamped current_pose_;
    sensor_msgs::PointCloud2 sonnar_cloud_;

    std::string sonnar_msgs_frame_id;


    double minX = -190;
    double maxX = 180;
    double minY = -200;
    double maxY = 200;
    double minZ = -80;
    double maxZ = -99;
    size_t target_number = 400; // Number of points
    float  target_sphere_radius=4;

    float search_radius = 40.0f; // Search radius for finding points within the given distance
    float resolution = 1.0f;     // Voxel size

    bool transform_is_recived_=false;
    Eigen::Affine3d trans_;




    pcl::PointCloud<pcl::PointXYZ>::Ptr static_target_;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_;
    std::vector<Eigen::Vector3d> target_points_;
    std::vector<float> target_dists_;
    pcl::PointCloud<pcl::PointXYZ> sim_pointcloud_;
    sensor_msgs::PointCloud2 sim_pointcloud_msg_;
    detect_msgs::Detect detect_msg_;

};

#endif // TARGET_DETECTION_SIM_H
