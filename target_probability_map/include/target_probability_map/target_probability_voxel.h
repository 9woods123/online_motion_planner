//
// Created by woods on 2023/7/3.
//

#ifndef TARGET_PROBABILITY_VOXEL_H
#define TARGET_PROBABILITY_VOXEL_H

#include "Eigen/Geometry"
#include <pcl/point_types.h>


namespace tp_map {

    enum voxel_state{
        UNKNOWN,
        OCCUR,
        FREE
    };

    struct voxel
    {
        float x;
        float y;
        float z;

        Eigen::Vector3i index;
        double probability;
        voxel_state target_search_state;

        // init function
        voxel()
        {
            x=0;y=0,z=0;
        }

        voxel(int X,int Y,int Z)
        {

            x=X,y=Y,z=Z;
            index<<X,Y,Z;
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
