#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <online_coverage_planner/tiling_map.h>
#include <visualization_msgs/MarkerArray.h>



class visualization{
    
    private:
            ros::NodeHandle nh_;
            ros::NodeHandle nh_private_;

            ros::Publisher marker_pub_;

             visualization_msgs::MarkerArray marker_array;
            std::shared_ptr<TilingMap::tiling_map> tiling_map_ptr;


    public:
            visualization(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
            void setTilingMap2vis(const TilingMap::tiling_map& tiling_map_);
            void drawMapinRviz();
            void pubMap();

};


#endif //VISUALIZATION_H