#include<visualization/visualization.h>


visualization::visualization(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) 
: nh_(nh),nh_private_(nh_private)
{
    // 创建一个用于发布 Marker 的发布者
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("tiling_map_markers", 10);

}

void visualization::setTilingMap2vis(const TilingMap::tiling_map& tiling_map_){
tiling_map_ptr=std::make_shared<TilingMap::tiling_map>(tiling_map_);



}


void visualization::drawMapinRviz(){

std::shared_ptr<TilingMap::TilingMapHashmap> tilingMapHashmap=
std::make_shared<TilingMap::TilingMapHashmap>(*tiling_map_ptr->getTilingMapHashmap());

// 遍历hashmap，绘制栅格，高度代表了对应的潜在价值。
float grid_resolution=tiling_map_ptr->getResolution();

for (const auto& entry : *tilingMapHashmap) {

    Eigen::Vector3d gridCenterpose=tiling_map_ptr ->getGridCenter(entry.first);


    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "tiling_map";
    marker.id = marker_array.markers.size(); // 使用数组大小作为 ID
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = gridCenterpose.x();
    marker.pose.position.y = gridCenterpose.y();
    marker.pose.position.z = gridCenterpose.z();         //TODO modify to bound_box_z_min
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x =grid_resolution;
    marker.scale.y =grid_resolution;

    if (entry.second.grid_state==TilingMap::gridState::UNKNOWN)
    {
    marker.scale.z =entry.second.potential*5;
    }
    else{
            marker.scale.z =0.1;
    }

    marker.color.r = 0.5; // 设置颜色
    marker.color.g = 0.15;
    marker.color.b = 0.0;
    marker.color.a = 0.8; // 设置透明度

    marker.lifetime = ros::Duration(); // 持续时间设为0，表示一直显示

    marker_array.markers.push_back(marker);
}

    // 发布整个 MarkerArray
    marker_pub_.publish(marker_array);


}


int main(int argc, char** argv) {
    ros::init(argc, argv, "visualization_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    visualization visualization_node(nh, nh_private);

    // TODO: 主循环或其他逻辑

    ros::spin(); // 进入 ROS 主循环

    return 0;
}