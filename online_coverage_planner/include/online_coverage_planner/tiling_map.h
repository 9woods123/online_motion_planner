//
// Created by woods on 2023/12/5.
//

#ifndef TILING_MAP_H
#define TILING_MAP_H

#include <ros/ros.h>
#include <unordered_map>
#include <Eigen/Geometry>




    //===================hash  function===================
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
    //===================hash  function===================


    enum gridState{
            UNKNOWN,
            OCCUR,
            FREE
    };

    struct tilingGrid{
                int index_x;
                int index_y;
                gridState grid_state;
    };


    class tiling_map{

        private:
            float resolution;
            float bound_box_x_min;
            float bound_box_x_max;
            float bound_box_y_min;
            float bound_box_y_max;
            typedef std::unordered_map<Eigen::Vector3i, tilingGrid, matrix_hash<Eigen::Vector3i>> TilingMapHashmap;


        public:
            tiling_map(float _resolution, float _bound_box_x_min, float _bound_box_x_max,
                                    float _bound_box_y_min,float _bound_box_y_max);
            void setMap();
            bool genTilingGrid();

    };

#endif //TILING_MAP_H