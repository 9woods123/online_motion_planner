//
// Created by woods on 2023/7/3.
//
#include "target_probability_map/target_probability_map.h"


namespace tp_map {

    target_probability_map::target_probability_map(double map_resolution,
                                                   double occur_threshold,
                                                   double free_threshold):
        map_resolution_(map_resolution),
        occur_threshold(occur_threshold),
        free_threshold(free_threshold) {

        map_.clear();
        changed_voxels.clear();
        map_origin_<<0,0,0;
    }

    Eigen::Vector3i target_probability_map::PointToIndex(Eigen::Vector3d point) {
        Eigen::Vector3i index = ((point - map_origin_) / map_resolution_).array().floor().cast<int>();
        return index;
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

    bool target_probability_map::updateProbabilityAtPosition(Eigen::Vector3d point, double observe_prob) {

        Eigen::Vector3i index = PointToIndex(point);
        // add a new voxel if
        voxel new_voxel(index);
        auto iterator = map_.emplace(index, new_voxel);
        if (!iterator.second) {
            // 元素已存在，可以通过迭代器访问现有的元素
            voxel& existingVoxel = iterator.first->second;
            double prior_prob = existingVoxel.probability; // 先获取先验概率
            double posterior_prob = (observe_prob * prior_prob)
                                    / (observe_prob * prior_prob + (1 - observe_prob) * (1 - prior_prob));
            // 使用贝叶斯更新计算后验概率
            existingVoxel.probability = posterior_prob; // 更新概率值
            changeVoxelState(&existingVoxel);// change voxel state according to the probability.

        } else {
            // 元素是新插入的
            // do nothing
        }
        return true;
    }

}

