//
// Created by woods on 2023/7/4.
//
#include "target_probability_map/tp_map_server.h"

namespace tp_map {

    tp_map_server::tp_map_server(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private),rate_(10)
    {
        double defalt_resolution=1;
        double defalt_o_th=0.85;
        double defalt_f_th=0.25;
        tp_map_server(nh,nh_private,defalt_resolution,defalt_o_th,defalt_f_th);
    }

    tp_map_server::tp_map_server(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                                 const double map_resolution, const double occur_threshold,
                                 const double free_threshold):
                                 nh_(nh), nh_private_(nh_private),rate_(10)
    {
        tp_map_.reset(new target_probability_map(map_resolution,occur_threshold,free_threshold));
        pointcloud_pub=nh_.advertise<sensor_msgs::PointCloud2>("sim_point_cloud", 1, true);
        robot_pose_sub=nh_.subscribe("robot_pose_topic",10, &tp_map_server::poseCallback,this);
        //sensors_input_sub=ros::Subscriber
        sim_sensing_timer=nh_.createTimer(1/sensing_frequency,&tp_map_server::sim_sensing_loop,this);
    }

    void tp_map_server::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pose = *msg;
    }

    bool tp_map_server::updateProbabilityAtPosition(Eigen::Vector3d point, double observe_prob){
        tp_map_->updateProbabilityAtPosition(point,observe_prob);
    }

    void tp_map_server::sim_sensing_loop(const ros::TimerEvent &event) {
        // looping every 1/sensing_frequency s.
        generateLocalSensing();
    }

    void tp_map_server::pubPointcloud(const pcl::PointCloud<pcl::PointXYZ> *pointcloud) {

       //转换到 sensor_msgs::PointCloud2  sim_pointcloud_msg 信息下，并用pointcloud_pub发布出去;

        pcl::toROSMsg(*pointcloud, sim_pointcloud_msg);
        // 设置 PointCloud2 的 header 信息（可选）
        sim_pointcloud_msg.header.frame_id = "world";
        sim_pointcloud_msg.header.stamp = ros::Time::now();

        // 发布 PointCloud2
        pointcloud_pub.publish(sim_pointcloud_msg);
    }

    void tp_map_server::generateLocalSensing()
    {
        const target_probability_map::tp_hashmap* hash_map_=tp_map_->getMap();
        pcl::PointCloud<pcl::PointXYZ> sim_pointcloud;
        // TODO calculate the sim point cloud according to the OCCUR voxel
        // and the point distance to current pose of robot.
        auto start = std::chrono::high_resolution_clock::now();

        for(auto voxel:*hash_map_)
        {
            Eigen::Vector3d voxel_center_coordinate=tp_map_->IndexToVoxelCenter(voxel.first);
            Eigen::Vector3d curr_pose(current_pose.pose.position.x,
                                      current_pose.pose.position.y,
                                      current_pose.pose.position.z);

            if(voxel.second.target_search_state!=OCCUR)
            {
                continue;
            }
            if((curr_pose - voxel_center_coordinate).norm()>local_sensing_radius)
            {
                continue;
            }
            // simulate a cylinder_cloud around the OCCUR Center
            double p_x,p_y,p_z;
            p_x=voxel_center_coordinate.x();
            p_y=voxel_center_coordinate.y();
            p_z=voxel_center_coordinate.z();
            const float cylinder_radius =1.5;
            const float cylinder_height =20;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            for (float theta = 0.0; theta <= 2.0 * M_PI; theta += 0.01) {
                for (float z = p_z - cylinder_height / 2.0; z <= p_z + cylinder_height / 2.0; z += 0.01) {
                    float x = p_x + cylinder_radius * std::cos(theta);
                    float y = p_y + cylinder_radius * std::sin(theta);

                    pcl::PointXYZ point;
                    point.x = x;
                    point.y = y;
                    point.z = z;

                    // 将点添加到圆柱体的点云对象中
                    cylinder_cloud->points.push_back(point);
                }
            }
            sim_pointcloud += *cylinder_cloud;     //merge the cylinder_cloud into the sim_pointcloud
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        std::cout << "generateLocalSensing  cost==" << elapsed.count() << " ms" << std::endl;
        pubPointcloud(&sim_pointcloud);

    }
}