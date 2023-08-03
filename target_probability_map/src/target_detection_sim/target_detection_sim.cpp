//
// Created by woods on 2023/7/5.
//
#include "target_detection_sim/target_detection_sim.h"
#include <tf/transform_listener.h>

target_detection_sim::target_detection_sim(ros::NodeHandle nh)
        : nh_(nh), octree_(resolution) {

    // Initialize subscribers and publishers
    robot_pose_sub_ = nh_.subscribe("robot_pose_topic", 10, &target_detection_sim::poseCallback, this);
    sonnar_sub_ = nh_.subscribe("sonnar_topic", 1, &target_detection_sim::sonnarCallback, this);
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sim_point_cloud", 1);
    targets_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("targets/visualization_markers", 10);
    detect_pub_=nh_.advertise<detect_msgs::Detect>("detect_info", 1);
    // Generate random target position and store in an octree for fast searching
    static_target_ = generateRandomPoints(minX, maxX, minY, maxY, minZ, maxZ, target_number);
    octree_.setInputCloud(static_target_);
    octree_.addPointsFromInputCloud();

    // Set up timers
    octree_search_timer_ = nh_.createTimer(ros::Duration(1.0), &target_detection_sim::octreeSearchCallback, this);
    sensor_timer_ = nh_.createTimer(ros::Duration(0.1), &target_detection_sim::sensorCallback, this);
    vis_timer_ = nh_.createTimer(ros::Duration(0.1), &target_detection_sim::visCallback, this);

}

void target_detection_sim::pubDetectInfo() {


}

void target_detection_sim::sonnarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sonnar_cloud_=*msg;
    sonnar_msgs_frame_id=sonnar_cloud_.header.frame_id;

    if(! transform_is_recived_)
    {
        lookforTransform("eca_a9/base_link",sonnar_msgs_frame_id,trans_);
    }
}

void target_detection_sim::generateLocalSensing(std::vector<Eigen::Vector3d> target_points,
                                                std::vector<float> target_dists) {


    detect_msgs::Detect detect_msg;
    detect_msg.header.stamp = ros::Time::now(); // 设置时间戳
    detect_msg.header.frame_id="world";


    for(int i=0;i<target_points.size();i++)
    {
        // 这里应该用高斯，但是暂时用1代替
        float prob=1;
        detect_msg.x.push_back(target_points[i].x());
        detect_msg.y.push_back(target_points[i].y());
        detect_msg.z.push_back(target_points[i].z());
        detect_msg.probability.push_back(prob);
    }

    detect_pub_.publish(detect_msg);

}


void target_detection_sim::generateLocalSensing(Eigen::Vector3d target_point) {
    // Simulate a sphere around the OCCUR Center
    double p_x, p_y, p_z;
    p_x = target_point.x();
    p_y = target_point.y();
    p_z = target_point.z();

    const float sphere_radius = target_sphere_radius; // Modify the sphere radius as desired
    const float pc_resolution = 0.2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    Eigen::Affine3d world_to_robot_transform = Eigen::Affine3d::Identity();
    world_to_robot_transform.translation() << current_pose_.pose.position.x,
            current_pose_.pose.position.y,
            current_pose_.pose.position.z;
    Eigen::Quaterniond quat(current_pose_.pose.orientation.w,
                            current_pose_.pose.orientation.x,
                            current_pose_.pose.orientation.y,
                            current_pose_.pose.orientation.z);
    world_to_robot_transform.rotate(quat);
    Eigen::Affine3d world_to_sensor_transform = world_to_robot_transform * trans_;

    for (float theta = 0.0; theta <= 2 * M_PI; theta += pc_resolution) {
        for (float phi = 0.0; phi <= M_PI; phi += pc_resolution) {
            float x = p_x + sphere_radius * std::cos(theta) * std::sin(phi);
            float y = p_y + sphere_radius * std::sin(theta) * std::sin(phi);
            float z = p_z + sphere_radius * std::cos(phi);
            pcl::PointXYZ point;
            Eigen::Vector3d transformed_point =
                    world_to_sensor_transform.inverse() * Eigen::Vector3d(x, y, z);

            point.x = transformed_point.x();
            point.y = transformed_point.y();
            point.z = transformed_point.z();

            // Add the point to the sphere_cloud
            sphere_cloud->points.push_back(point);
        }
    }
    sim_pointcloud_ += *sphere_cloud;
}


void target_detection_sim::sensorCallback(const ros::TimerEvent& event) {


    generateLocalSensing(target_points_,target_dists_);

//    sim_pointcloud_.clear();
//
//    // 将订阅到的sonnar信息转换成点云，然后generateLocalSensing 将模拟的障碍物，添加到这里面。
//
//    pcl::fromROSMsg(sonnar_cloud_, sim_pointcloud_);
//
//    for (auto point : target_points_) {
//        generateLocalSensing(point);
//
//    }
//
//    pubPointcloud(&sim_pointcloud_);

}

void target_detection_sim::visCallback(const ros::TimerEvent& event) {
    visualizePointCloud(static_target_);
}

void target_detection_sim::octreeSearchCallback(const ros::TimerEvent& event) {
    auto start = std::chrono::high_resolution_clock::now();

    target_points_.clear();
    target_dists_.clear();
    pcl::PointXYZ search_point;
    search_point.x = current_pose_.pose.position.x;
    search_point.y = current_pose_.pose.position.y;
    search_point.z = current_pose_.pose.position.z;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (octree_.radiusSearch(search_point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        // Point(s) found within the search radius
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            pcl::PointXYZ nearby_point = static_target_->points[pointIdxRadiusSearch[i]];
            target_points_.emplace_back(Eigen::Vector3d(nearby_point.x, nearby_point.y, nearby_point.z));
            target_dists_.emplace_back(pointRadiusSquaredDistance[i]);
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
//    std::cout << "octreeSearchCallback  cost==" << elapsed.count() << " ms" << std::endl;
}

void target_detection_sim::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr target_detection_sim::generateRandomPoints(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, size_t n) {
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
        cloud->points[i].x = dist_x(gen);
        cloud->points[i].y = dist_y(gen);
        cloud->points[i].z = dist_z(gen);
    }
    return cloud;
}

void target_detection_sim::visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // Create a visualization_msgs::MarkerArray
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    // Create a new Marker, and set the type to SPHERE_LIST
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 2*target_sphere_radius; // Set the size of the sphere
    marker.color.r = 1.0; // Set the color to red
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3; // Set the alpha value (transparency)
    // marker.lifetime = ros::Duration(0);

    for (const auto& point : cloud->points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }

    // Add the Marker to the MarkerArray
    marker_array.markers.push_back(marker);
    // Publish the MarkerArray
    targets_vis_pub_.publish(marker_array);
}

void target_detection_sim::pubPointcloud(const pcl::PointCloud<pcl::PointXYZ>* pointcloud) {
    // Convert to sensor_msgs::PointCloud2 and publish it using pointcloud_pub_
    pcl::toROSMsg(*pointcloud, sim_pointcloud_msg_);
    // Set PointCloud2 header information (optional)
    sim_pointcloud_msg_.header.frame_id = sonnar_msgs_frame_id;
    sim_pointcloud_msg_.header.stamp = ros::Time::now();
    sim_pointcloud_msg_.is_dense = true; // Set if the point cloud data is all valid
    // Publish PointCloud2
    pointcloud_pub_.publish(sim_pointcloud_msg_);
}


bool target_detection_sim::lookforTransform(std::string target_frame, std::string source_frame,
                                            Eigen::Affine3d& transformEigen) {


    tf::TransformListener tfListener;

    ros::Rate rate(10.0);
    while (ros::ok()) {
        // 使用tf::TransformListener对象的lookupTransform函数找到从source_frame到target_frame的坐标变换
        tf::StampedTransform transform;
        try {
            tfListener.lookupTransform(target_frame, source_frame,
                                       ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // 将tf::StampedTransform转换为Eigen::Affine3d类型
        transformEigen.translation() << transform.getOrigin().x(),
                transform.getOrigin().y(),
                transform.getOrigin().z();
        transformEigen.linear() = Eigen::Quaterniond(transform.getRotation().getW(),
                                                     transform.getRotation().getX(),
                                                     transform.getRotation().getY(),
                                                     transform.getRotation().getZ()).toRotationMatrix();
        transform_is_recived_=true;
        // 输出变换矩阵
        std::cout << "Transform from " << source_frame << " to " << target_frame << ":\n" << transformEigen.matrix() << std::endl;
        break;
        rate.sleep();
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_detection_node");
    ros::NodeHandle nh;
    target_detection_sim sim(nh);
    ros::spin();
    return 0;
}






