//
// Created by woods on 2023/7/3.
//

#ifndef TARGET_PROBABILITY_MAP_H
#define TARGET_PROBABILITY_MAP_H

#include <ros/ros.h>
#include <unordered_map>
#include <Eigen/Geometry>
#include <vector>
#include "target_probability_voxel.h"
#include "voxel_point.h"
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h> // 如果使用了 OctreePointCloud 类
#include <pcl/octree/octree_search.h>


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
        typedef pcl::PointCloud<VoxelPoint> VoxelPoint_Cloud;
        typedef typename VoxelPoint_Cloud::Ptr VoxelPoint_CloudPTR;
        typedef pcl::octree::OctreePointCloudSearch<VoxelPoint> OctreeType;


        target_probability_map(double map_resolution, double occur_threshold, double free_threshold);
        Eigen::Vector3i PointToIndex(Eigen::Vector3d point);
        Eigen::Vector3d IndexToVoxelCenter(Eigen::Vector3i index);
        bool updateProbabilityAtPosition(Eigen::Vector3d point,double observe_prob);
        bool radiusSearch(Eigen::Vector3d search_point,float raduis,
                          std::vector<Eigen::Vector3d>& existPoints,std::vector<float>& existDists);
        bool updateProbBySensor();
        void changeVoxelState(voxel* v);
        double getMapResolution();
        bool calculateFrontiers();
        tp_hashmap* getMap();
        OctreeType* getOctree();

    private:




        double map_resolution_;
        double occur_threshold;
        double free_threshold;
        Eigen::Vector3d map_origin_;


        tp_hashmap map_;
        std::vector<voxel*>  changed_voxels;
        std::vector<voxel*>  frontiers;


        float resolution = 1.0f;     // Voxel size

//        typedef pcl::PointCloud<VoxelPoint> PointCloud;
//        typedef typename PointCloud::Ptr PointCloudPtr;
//        typedef pcl::octree::OctreePointCloudSearch<VoxelPoint> OctreeType;
//        typename OctreeType::Ptr octree;
//        PointCloudPtr cloud;


        OctreeType octree;
        VoxelPoint_CloudPTR cloud;


    };

}


#endif //TARGET_PROBABILITY_MAP_H
