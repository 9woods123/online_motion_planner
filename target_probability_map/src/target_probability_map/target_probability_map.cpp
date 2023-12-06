//
// Created by woods on 2023/7/3.
//
#include "target_probability_map/target_probability_map.h"
#include "chrono"
namespace tp_map {

    target_probability_map::target_probability_map(double map_resolution,
                                                   double occur_threshold,
                                                   double free_threshold):
            map_resolution_(map_resolution),
            occur_threshold(occur_threshold),
            free_threshold(free_threshold),
            octree(0.1f)
    {

        map_.clear();
        changed_voxels.clear();
        map_origin_<<0,0,0;


        cloud.reset(new VoxelPoint_Cloud);
        cloud->points.push_back(VoxelPoint(0,0,0,0));


        //TODO the boudning box is defined, but static.
        double minx, miny, minz, maxx, maxy, maxz;
        minx=-200;miny=-200;minz=-100;maxx=200;maxy=200;maxz=0;
        octree.defineBoundingBox(minx, miny, minz, maxx, maxy, maxz);
        // the boudning box is defined, but static.

        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();
    }

    Eigen::Vector3i target_probability_map::PointToIndex(Eigen::Vector3d point) {
        Eigen::Vector3i index = ((point - map_origin_) / map_resolution_).array().floor().cast<int>();
        return index;
    }

    void target_probability_map::setCollisionRadius(float collision_radius_) {
    collision_radius=collision_radius_;
    }

    target_probability_map::OctreeType* target_probability_map::getOctree() {
        return &octree;
    }

    Eigen::Vector3d target_probability_map::IndexToVoxelCenter(Eigen::Vector3i index) {
        Eigen::Vector3d center = map_origin_ +
                (index.cast<double>() + Eigen::Vector3d(0.5,0.5,0.5)) * map_resolution_;
        return center;
    }

    target_probability_map::tp_hashmap* target_probability_map::getMap() {
        return  &map_;
    }

    double target_probability_map::getMapResolution() {
        return map_resolution_;
    }

    bool target_probability_map::calculateFrontiers() {
        if (changed_voxels.empty()) {
            return false;
        }

        // 清空之前的边界
        frontiers.clear();

        // 遍历每个 changed_voxels 中的 voxel
        for (auto c_voxel: changed_voxels) {
            const Eigen::Vector3i &voxel_index = c_voxel->index;

            // 遍历 26-neibors
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;  // 跳过当前体素
                        }
                        Eigen::Vector3i neighbor_index = voxel_index + Eigen::Vector3i(dx, dy, dz);
                        auto neighbor_iter = map_.find(neighbor_index);

                        if (neighbor_iter == map_.end()) {
                            // if neighbor_iter is not occurs in the map_ before, it's new
                            // , and the voxel should be a frontiers.
                            frontiers.push_back(c_voxel);
                        } else if (neighbor_iter->second.target_search_state != OCCUR &&
                                   neighbor_iter->second.target_search_state == UNKNOWN) {
                            // if neighbor_iter is not OCCUR, and neighbor is unknown
                            // the voxel should be a frontiers.
                            frontiers.push_back(c_voxel);
                        }
                    }
                }
            }
            return true;
        }
    }

    void target_probability_map::changeVoxelState(voxel *voxel_) {

        if(voxel_->probability>occur_threshold)
        {
            if(voxel_->target_search_state!=OCCUR)
            {
                changed_voxels.push_back(voxel_);
            }
            voxel_->target_search_state=OCCUR;
        }
        else if( voxel_->probability< free_threshold)
        {
            if(voxel_->target_search_state!=FREE)
            {
                changed_voxels.push_back(voxel_);
            }
            voxel_->target_search_state=FREE;
        }
        else
        {
            if(voxel_->target_search_state!=UNKNOWN)
            {
                changed_voxels.push_back(voxel_);
            }
            voxel_->target_search_state=UNKNOWN;
        }
    }

    bool target_probability_map::updateProbBySensor() {
        // TODO use updateProbabilityAtPosition here to update every detected point.
    }

    bool target_probability_map::radiusSearch(Eigen::Vector3d search_point, float radius,
                                              std::vector<Eigen::Vector3d> &existPoints,
                                              std::vector<float> &existDists) {
        //  0.05ms 左右

        VoxelPoint searchPoint( search_point.x(),search_point.y(),search_point.z()); // 设置要搜索的点

        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;

        int num_neighbors = octree.radiusSearch(searchPoint, radius, k_indices, k_sqr_distances);

        for (int i = 0; i < num_neighbors; ++i) {
            int index = k_indices[i];
            float distance = std::sqrt(k_sqr_distances[i]);
            existPoints.emplace_back(Eigen::Vector3d(
                                    cloud->points[index].x,
                                    cloud->points[index].y,
                                    cloud->points[index].z));
            existDists.emplace_back(distance);
        }



    }

    bool target_probability_map::updateProbabilityAtPosition(Eigen::Vector3d point, double observe_prob) {

        Eigen::Vector3i index = PointToIndex(point);
//        VoxelPoint search(index.x(),index.y(),index.z());
        VoxelPoint search(point.x(),point.y(),point.z());

        std::vector<int> point_index;

        // test
//        std::vector<Eigen::Vector3d> existPoints;
//        std::vector<float> existDists;
//        radiusSearch(point,50,existPoints,existDists);
        // test

        octree.voxelSearch(search, point_index);
        if (!point_index.empty()) {
            // Voxel contains points, access the points using the indices
            for (const int& idx : point_index) {
//              std::cout<<"voxelSearch found"<<std::endl;
                VoxelPoint* point_in_voxel = &cloud->points[idx];
                double prior_prob = point_in_voxel->probability; // 先获取先验概率
                double posterior_prob = (observe_prob * prior_prob)
                                    / (observe_prob * prior_prob + (1 - observe_prob) * (1 - prior_prob));
                // 使用贝叶斯更新计算后验概率
                point_in_voxel->probability = posterior_prob; // 更新概率值
//                std::cout<<"cloud size=="<<cloud->size()<<std::endl;
                //changeVoxelState();
            }

        } else {
            // Voxel does not contain any points
            VoxelPoint new_point(point.x(),point.y(),point.z()); // push back the real data which has not been processed by floor();
            double prior_prob = new_point.probability; // 先获取先验概率
            double posterior_prob = (observe_prob * prior_prob)
                                    / (observe_prob * prior_prob + (1 - observe_prob) * (1 - prior_prob));
            // 使用贝叶斯更新计算后验概率
            new_point.probability = posterior_prob; // 更新概率值
            octree.addPointToCloud(new_point,cloud);

        }

        return true;
    }

}

