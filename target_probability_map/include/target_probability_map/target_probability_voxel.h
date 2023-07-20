//
// Created by woods on 2023/7/3.
//

#ifndef TARGET_PROBABILITY_VOXEL_H
#define TARGET_PROBABILITY_VOXEL_H

#include "Eigen/Geometry"

namespace tp_map {

    enum voxel_state{
          UNKNOWN,
          OCCUR,
          FREE
    };

    struct voxel
    {
        Eigen::Vector3i index;
        double probability;
        voxel_state target_search_state;

        // init function
        voxel(int x,int y,int z)
        {
            index<<x,y,z;
            probability = 0.5;
            target_search_state=voxel_state::UNKNOWN;
        }
        voxel(Eigen::Vector3i index)
        {
            index=index;
            probability = 0.5;
            target_search_state=voxel_state::UNKNOWN;
        }
    };
};

#endif //TARGET_PROBABILITY_VOXEL_H
