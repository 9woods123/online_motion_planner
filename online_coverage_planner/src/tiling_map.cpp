
#include<online_coverage_planner/tiling_map.h>

namespace TilingMap{


tiling_map::tiling_map(float _resolution, float _bound_box_x_min, float _bound_box_x_max,float _bound_box_y_min,float _bound_box_y_max)
{


resolution=_resolution;
bound_box_x_min=_bound_box_x_min;
bound_box_x_max=_bound_box_x_max;
bound_box_y_min=_bound_box_y_min;
bound_box_y_max=_bound_box_y_max;
map_origin_<<0,0,0;

genTilingGrid();

}


/*!
 按分辨率生成栅各，并存储在hashmap中。
 遍历整个地图
 grid.potential，沿着 -y 方向递减，

 P_max              ----->                  P_min
 ——————————————————
|                                                                  | 
|                               x                                 | 
|                               |                                  | 
|          y —— （0，0）                        | 
|                                                                  | 
|                                                                  | 
 ——————————————————
*/
bool tiling_map::genTilingGrid(){

    Eigen::Vector3i maxIndex=PointToIndex(Eigen::Vector3d(bound_box_x_max,bound_box_y_max,0));
    Eigen::Vector3i minIndex=PointToIndex(Eigen::Vector3d(bound_box_x_min,bound_box_y_min,0));

    double delta_potential_between_cols=1;

    for (double x = bound_box_x_min; x < bound_box_x_max;  x += resolution) {
        for (double y = bound_box_y_min; y < bound_box_y_max;  y += resolution) {
                      
            Eigen::Vector3i gridIndex=PointToIndex(Eigen::Vector3d(x,y,0));  // 计算栅格索引

            float potential=delta_potential_between_cols+                                           
            delta_potential_between_cols *(gridIndex.y() - minIndex.y());       // 按列计算潜在价值

            tilingGrid grid(gridIndex,potential);                                                       // 声明一个 grid 

            tiling_map_hashmap[gridIndex] = grid;                                             // grid 插入 hashmap
        }
    }

//     for (const auto& tiling_grid : tiling_map_hashmap) {
//             std::cout<<"debug"<<"tiling grid index= "<<tiling_grid.second.index_x<<", "<<tiling_grid.second.index_y
//             <<"    potential=="<< tiling_grid.second.potential<<std::endl;
//     }

}


bool tiling_map::updateMapbyRobotPose(Eigen::Vector3d point)
{

double checkFreeRadius=5;

Eigen::Vector3i index=PointToIndex(point);
Eigen::Vector3d grid_center_pose=getGridCenter(index);

double distance = (grid_center_pose - point).norm();

if (distance<checkFreeRadius){

tiling_map_hashmap[index].grid_state=gridState::FREE;
tiling_map_hashmap[index].potential=0;

}

}


const TilingMapHashmap* tiling_map::getTilingMapHashmap()const{
return  &tiling_map_hashmap;    
}


Eigen::Vector3i tiling_map::PointToIndex(Eigen::Vector3d point)const
{

        Eigen::Vector3i index = ((point - map_origin_) / resolution).array().floor().cast<int>();
        index.z()=0;
        
        return index;
}


Eigen::Vector3d tiling_map::getGridCenter(Eigen::Vector3i index)const{

        Eigen::Vector3d center = map_origin_ +
                (index.cast<double>() + Eigen::Vector3d(0.5,0.5,0.5)) * resolution;

        center.z()=0;
        return center;

}

float tiling_map::getResolution()const{
                return resolution;
            }


}