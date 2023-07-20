//
// Created by woods on 2023/7/3.
//

#ifndef TARGET_PROBABILITY_MAP_H
#define TARGET_PROBABILITY_MAP_H


#include <unordered_map>
#include <Eigen/Geometry>
#include <vector>
#include "target_probability_voxel.h"

namespace tp_map
{
    // hash  function
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
    };        // hash  function


    class target_probability_map
    {

    public:
        // maybe using octree is better
        typedef std::unordered_map<Eigen::Vector3i,voxel,matrix_hash<Eigen::Vector3i>> tp_hashmap;

        target_probability_map(double map_resolution, double occur_threshold, double free_threshold);
        Eigen::Vector3i PointToIndex(Eigen::Vector3d point);
        Eigen::Vector3d IndexToVoxelCenter(Eigen::Vector3i index);
        bool updateProbabilityAtPosition(Eigen::Vector3d point,double observe_prob);
        bool updateProbBySensor();
        void changeVoxelState(voxel* v);
        double getMapResolution();
        bool calculateFrontiers();
        tp_hashmap* getMap();

    private:

        double map_resolution_;
        double occur_threshold;
        double free_threshold;
        Eigen::Vector3d map_origin_;


        tp_hashmap map_;
        std::vector<voxel*>  changed_voxels;
        std::vector<voxel*>  frontiers;

    };

}






















#endif //TARGET_PROBABILITY_MAP_H
