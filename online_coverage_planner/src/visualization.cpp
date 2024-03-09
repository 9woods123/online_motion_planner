#include<visualization/visualization.h>


visualization::visualization(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) 
: nh_(nh),nh_private_(nh_private)
{
    // 创建一个用于发布 Marker 的发布者
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("tiling_map_markers", 10);

    world_frame_="world";

}

void visualization::setTilingMap2vis( TilingMap::tiling_map& tiling_map_){

// tiling_map_ptr=std::make_shared<TilingMap::tiling_map>(&tiling_map_);
// 上面这样写，没法动态跟踪tiling map 的修改。std::make_shared 的目的是在堆上分配内存并构造对象，
//然后返回一个 std::shared_ptr，而不是用于共享已有对象。所以，不能使用 std::make_shared 直接共享已有对象。

tiling_map_ptr.reset(&tiling_map_);
}


void visualization::drawMapinRviz(const TilingMap::tilingGrid& next_bext_grid){
    
marker_array.markers.clear();



const auto tilingMapHashmap=tiling_map_ptr->getTilingMapHashmap();

// std::cout<<"=========================debug drawMapinRviz========================="<<std::endl;

// for (const auto& tiling_grid : *tilingMapHashmap) {
//         std::cout<<"debug"<<"tiling grid index= "<<tiling_grid.second.index_x<<", "<<tiling_grid.second.index_y
//         <<"    potential=="<< tiling_grid.second.potential<<std::endl;
// }

// 遍历hashmap，绘制栅格，高度代表了对应的潜在价值。
float grid_resolution=tiling_map_ptr->getResolution();
 
for (const auto& entry : *tilingMapHashmap) {

    if (entry.second.index_x==next_bext_grid.index_x && 
    entry.second.index_y==next_bext_grid.index_y           //  the next goal grid
    )
    {
    std::cout<<"index_x,index_y"<<entry.second.index_x<<", "<<entry.second.index_y<<std::endl;
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "tiling_map";
    marker.id = marker_array.markers.size(); // 使用数组大小作为 ID
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x =grid_resolution*0.9;
    marker.scale.y =grid_resolution*0.9;

    if (entry.second.grid_state==TilingMap::gridState::UNKNOWN)
    {
    marker.scale.z =entry.second.potential*1;
    }
    else{
    marker.scale.z =0.1;
    }

    Eigen::Vector3d gridCenterpose=tiling_map_ptr ->getGridCenter(entry.first);

    marker.pose.position.x = gridCenterpose.x();
    marker.pose.position.y = gridCenterpose.y();
    marker.pose.position.z = gridCenterpose.z() + marker.scale.z / 2.0;         //TODO modify to bound_box_z_min
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.color.r = 0; // 设置颜色
    marker.color.g = 0.5;
    marker.color.b = 0.1;
    marker.color.a = 0.5; // 设置透明度

    marker.lifetime = ros::Duration(0); // 持续时间设为0，表示一直显示

    marker_array.markers.push_back(marker);
    continue;
    }

    else{

    Eigen::Vector3d gridCenterpose=tiling_map_ptr ->getGridCenter(entry.first);

    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "tiling_map";
    marker.id = marker_array.markers.size(); // 使用数组大小作为 ID
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;


    marker.scale.x =grid_resolution*0.9;
    marker.scale.y =grid_resolution*0.9;

    if (entry.second.grid_state==TilingMap::gridState::UNKNOWN)
    {
    marker.scale.z =entry.second.potential*1;

    marker.pose.position.x = gridCenterpose.x();
    marker.pose.position.y = gridCenterpose.y();
    marker.pose.position.z = gridCenterpose.z() + marker.scale.z / 2.0;         //TODO modify to bound_box_z_min
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;


    marker.color.r = 0.5; // 设置颜色
    marker.color.g = 0.15;
    marker.color.b = 0.0;
    marker.color.a = 0.5; // 设置透明度

    }
    else{
            marker.scale.z =0.1;
            marker.pose.position.x = gridCenterpose.x();
            marker.pose.position.y = gridCenterpose.y();
            marker.pose.position.z = gridCenterpose.z() + marker.scale.z / 2.0;         //TODO modify to bound_box_z_min
            
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.color.r = 0.0; // 设置颜色
            marker.color.g = 0.0;
            marker.color.b = 0.1;
            marker.color.a = 0.2; // 设置透明度
    }


    marker.lifetime = ros::Duration(0); // 持续时间设为0，表示一直显示

    marker_array.markers.push_back(marker);
}

    }

    // 发布整个 MarkerArray

marker_pub_.publish(marker_array);

}

// void visualization::drawMapinRviz(){


// marker_array.markers.clear();

// std::shared_ptr<TilingMap::TilingMapHashmap> tilingMapHashmap=
// std::make_shared<TilingMap::TilingMapHashmap>(*tiling_map_ptr->getTilingMapHashmap());

// // 遍历hashmap，绘制栅格，高度代表了对应的潜在价值。
// float grid_resolution=tiling_map_ptr->getResolution();
 
// for (const auto& entry : *tilingMapHashmap) {

//     Eigen::Vector3d gridCenterpose=tiling_map_ptr ->getGridCenter(entry.first);

//     visualization_msgs::Marker marker;
//     marker.header.frame_id = world_frame_;
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "tiling_map";
//     marker.id = marker_array.markers.size(); // 使用数组大小作为 ID
//     marker.type = visualization_msgs::Marker::CUBE;
//     marker.action = visualization_msgs::Marker::ADD;


//     marker.scale.x =grid_resolution*0.9;
//     marker.scale.y =grid_resolution*0.9;

//     if (entry.second.grid_state==TilingMap::gridState::UNKNOWN)
//     {
//     marker.scale.z =entry.second.potential*1;
//     }
//     else{
//             marker.scale.z =0.1;
//     }

//     marker.pose.position.x = gridCenterpose.x();
//     marker.pose.position.y = gridCenterpose.y();
//     marker.pose.position.z = gridCenterpose.z() + marker.scale.z / 2.0;         //TODO modify to bound_box_z_min
    

//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;


//     marker.color.r = 0.5; // 设置颜色
//     marker.color.g = 0.15;
//     marker.color.b = 0.0;
//     marker.color.a = 0.5; // 设置透明度

//     marker.lifetime = ros::Duration(); // 持续时间设为0，表示一直显示

//     marker_array.markers.push_back(marker);
// }
// // std::cout<<"    marker_array.markers.size();"<<    marker_array.markers.size()<<std::endl;

//     // 发布整个 MarkerArray
// marker_pub_.publish(marker_array);


// }


int main(int argc, char** argv) {
    ros::init(argc, argv, "visualization_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    visualization visualization_node(nh, nh_private);

    // TODO: 主循环或其他逻辑

    ros::spin(); // 进入 ROS 主循环

    return 0;
}