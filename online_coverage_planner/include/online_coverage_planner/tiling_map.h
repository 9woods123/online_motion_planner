//
// Created by woods on 2023/12/5.
//

#ifndef TILING_MAP_H
#define TILING_MAP_H

#include <ros/ros.h>
#include <unordered_map>
#include <Eigen/Geometry>

namespace TilingMap{


    //==================================hash  function==================================
    template <typename T> struct matrix_hash : std::unary_function<T, size_t> {
        std::size_t operator()(T const& matrix) const {
            size_t seed = 0;
            for (size_t i = 0; i < matrix.size(); ++i) {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
                        (seed >> 2);
            }
            return seed;
        }
    };        
    //==================================hash  function==================================


    enum gridState{
            UNKNOWN,
            OCCUR,
            FREE
    };

    struct tilingGrid{
                int index_x;
                int index_y;
                float potential;
                gridState grid_state;

                tilingGrid() {
                    // Default constructor
                    index_x = 0;
                    index_y = 0;
                    potential = 1;
                    grid_state = gridState::UNKNOWN;
                }
                tilingGrid(int index_x_, int index_y_){
                    index_x=index_x_;
                    index_y_=index_y_;
                    potential=1;
                    grid_state=gridState::UNKNOWN;
                }
                tilingGrid( Eigen::Vector3i index){
                    index_x=index.x();
                    index_y=index.y();
                    potential=1;
                    grid_state=gridState::UNKNOWN;
                }
                tilingGrid( Eigen::Vector3i index, float p){
                    index_x=index.x();
                    index_y=index.y();
                    potential=p;
                    grid_state=gridState::UNKNOWN;
                }


    };

    typedef std::unordered_map<Eigen::Vector3i, tilingGrid, matrix_hash<Eigen::Vector3i>> TilingMapHashmap;

    class tiling_map{

        public:

            tiling_map(float _resolution, float _bound_box_x_min, float _bound_box_x_max,
                                    float _bound_box_y_min,float _bound_box_y_max);
            bool updateMapbyRobotPose(Eigen::Vector3d point);
            TilingMapHashmap* getTilingMapHashmap();
            Eigen::Vector3d getGridCenter(Eigen::Vector3i index);
            Eigen::Vector3i PointToIndex(Eigen::Vector3d point);
            float getResolution();

        private:
            float resolution;
            float bound_box_x_min;
            float bound_box_x_max;
            float bound_box_y_min;
            float bound_box_y_max;

            Eigen::Vector3d map_origin_;
            TilingMapHashmap tiling_map_hashmap;

            bool genTilingGrid();






    };
}
#endif //TILING_MAP_H