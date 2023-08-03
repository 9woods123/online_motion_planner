//
// Created by woods on 2023/7/3.
//
#define PCL_NO_PRECOMPILE

#include "Eigen/Geometry"
#include <pcl/point_types.h>


struct VoxelPoint
{
    float x;
    float y;
    float z;
    float probability;

    VoxelPoint(float _x,float _y,float _z)
    {
        x=_x;
        y=_y;
        z=_z;
        probability=0.5;
    }

    VoxelPoint()
    {

    }
    const Eigen::Vector3f getVector3fMap() const
    {
        return Eigen::Vector3f(x, y, z);
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(VoxelPoint,
                                          (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, probability, probability)
)
