//
// Created by woods on 2023/7/5.
//


#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <chrono>
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include <pcl/octree/octree_search.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/MarkerArray.h>

ros::Subscriber robot_pose_sub;
geometry_msgs::PoseStamped current_pose;
ros::Publisher  pointcloud_pub;
ros::Publisher targets_vis_pub;

double minX = -200;
double maxX =  180;
double minY = -200;
double maxY =  200;
double minZ = -80;
double maxZ = -91;
size_t targent_number = 300;  // Number of points

float search_radius =50.0f;  // Search radius for finding points within the given distance
float resolution = 1.0f;  // Voxel size
pcl::PointCloud<pcl::PointXYZ>::Ptr static_target;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
std::vector<Eigen::Vector3d> target_points;
pcl::PointCloud<pcl::PointXYZ> sim_pointcloud;
sensor_msgs::PointCloud2   sim_pointcloud_msg;

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // 创建一个用于发布MarkerArray的发布者

    // 创建一个visualization_msgs::MarkerArray
    visualization_msgs::MarkerArray marker_array;
    int id = 0;


    // 创建一个新的Marker，并设置类型为SPHERE_LIST
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 5; // 设置球体的大小
    // 设置球体的颜色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;
//    marker.lifetime=ros::Duration(0);

    for (const auto& point : cloud->points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }

    // 将Marker添加到MarkerArray中
    marker_array.markers.push_back(marker);
    // 发布MarkerArray
    targets_vis_pub.publish(marker_array);
}

void pubPointcloud(const pcl::PointCloud<pcl::PointXYZ> *pointcloud) {

    //转换到 sensor_msgs::PointCloud2  sim_pointcloud_msg 信息下，并用pointcloud_pub发布出去;

    pcl::toROSMsg(*pointcloud, sim_pointcloud_msg);
    // 设置 PointCloud2 的 header 信息（可选）
    sim_pointcloud_msg.header.frame_id = "eca_a9/base_link";
    sim_pointcloud_msg.header.stamp = ros::Time::now();
    sim_pointcloud_msg.is_dense = true; // 设置点云数据是否都有效
    // 发布 PointCloud2
    pointcloud_pub.publish(sim_pointcloud_msg);
}

void generateLocalSensing(Eigen::Vector3d target_point )
{
    // simulate a cylinder_cloud around the OCCUR Center
    double p_x,p_y,p_z;
    p_x=target_point.x();
    p_y=target_point.y();
    p_z=target_point.z();

    const float cylinder_radius =1.5;
    const float cylinder_height =20;
    const float pc_resolution=0.5;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Affine3d world_to_robot_transform=Eigen::Affine3d::Identity();
            world_to_robot_transform.translation() <<
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z;
    Eigen::Quaterniond quat(current_pose.pose.orientation.w,
                            current_pose.pose.orientation.x,
                            current_pose.pose.orientation.y,
                            current_pose.pose.orientation.z);
    world_to_robot_transform.rotate(quat);

    for (float theta = 0.0; theta <= 2 * M_PI; theta += pc_resolution) {
        for (float z = p_z - cylinder_height / 2.0; z <= p_z + cylinder_height / 2.0; z += pc_resolution) {
            float x = p_x + cylinder_radius * std::cos(theta);
            float y = p_y + cylinder_radius * std::sin(theta);
            pcl::PointXYZ point;
            Eigen::Vector3d transformed_point =
            world_to_robot_transform.inverse() * Eigen::Vector3d(x, y, z);

            point.x = transformed_point.x();
            point.y = transformed_point.y();
            point.z = transformed_point.z();

            // 将点添加到圆柱体的点云对象中
            cylinder_cloud->points.push_back(point);
        }
    }
    sim_pointcloud += *cylinder_cloud;

}

void sensorCallback(const ros::TimerEvent& event)
{
    auto start = std::chrono::high_resolution_clock::now();

    sim_pointcloud.clear();
    for(auto point :target_points)
    {
        generateLocalSensing(point);
    }
    pubPointcloud(&sim_pointcloud);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "sensorCallback  cost==" << elapsed.count() << " ms" << std::endl;
}

void visCallback(const ros::TimerEvent& event) {
    visualizePointCloud(static_target);
}

void octreeSearchCallback(const ros::TimerEvent& event) {
    target_points.clear();
    pcl::PointXYZ search_point;
    search_point.x = current_pose.pose.position.x;
    search_point.y = current_pose.pose.position.y;
    search_point.z = current_pose.pose.position.z;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (octree.radiusSearch(search_point, search_radius,
                            pointIdxRadiusSearch,pointRadiusSquaredDistance) > 0)
    {
        // Point(s) found within the search radius
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            pcl::PointXYZ nearby_point = static_target->points[pointIdxRadiusSearch[i]];
            target_points.emplace_back(Eigen::Vector3d
                                               (nearby_point.x,nearby_point.y,nearby_point.z));
        }

    }

}


void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose=*msg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateRandomPoints
        (double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, size_t n){

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> dist_x(min_x, max_x);
    std::uniform_real_distribution<double> dist_y(min_y, max_y);
    std::uniform_real_distribution<double> dist_z(min_z, max_z);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = n;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (int i = 0; i < n; ++i) {
        cloud->points[i].x=dist_x(gen);
        cloud->points[i].y=dist_y(gen);
        cloud->points[i].z=dist_z(gen);
    }
    return cloud;
}

