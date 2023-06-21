#include "hybrid_astar/collisiondetection.h"

using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
    esdf_server_= nullptr;
}



void CollisionDetection::SetMapPtr(voxblox::EsdfServer *esdf_server) {
    esdf_server_.reset(esdf_server);
}


void CollisionDetection::getMapSize() {

    std::cout<<"block_size()==="<<
    esdf_server_->getEsdfMapPtr()->block_size()
    <<std::endl
    <<"NumberOfAllocatedBlocks()==="<<
    esdf_server_->getEsdfMapPtr()->getEsdfLayer().getNumberOfAllocatedBlocks()
    <<std::endl;

}

VoxelState CollisionDetection::checkVoxelState(Eigen::Vector3d point) {

//    std::cout<< "checkVoxelState"<<std::endl;
    double distance = 0.0;
    double c_voxel_distance=collision_distance_;

    if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(point, &distance))
    {
        // This means the voxel is observed
        if (distance < c_voxel_distance)
        {
            return VoxelState::occupied;
        }
        else
        {
            return VoxelState::free;
        }

    }
    else
    {
        return VoxelState::unknown;
    }

}



bool CollisionDetection::isTraversable(const HybridAStar::Node4D *node){

    //  this function from chatgpt is error that it provides a solution calculating
    //  a bound box using octomap->begin_leafs_bbx(min,max);
    //  the bound box axis is fixed, two point are given to determine a cuboid
    //  but the min and max point are opposite, when rotation happens, the
    //  bound box are smaller than we imagined.


    float length=bound_box_length_;
    float width =bound_box_width_;
    float height=bound_box_height_;   //  TODO,   get params from launch


    float current_x=node->getX();
    float current_y=node->getY();
    float current_z=node->getZ();
    float current_t=node->getT();



    // Convert the yaw to a rotation matrix
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = Eigen::Vector3d(current_x, current_y, current_z);
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(current_t, Eigen::Vector3d::UnitZ());
    transform.linear() = rot;

    float resolution=esdf_server_->getEsdfMapPtr()->voxel_size();

    for (float x= -length/2.0; x<= length/2.0; x+=resolution)
    {

        for(float y= -width/2.0; y<=width/2.0; y+=resolution)
        {

            for (float z= -height/2.0; z<= height/2.0; z+=resolution)
            {
             Eigen::Vector3d point_in_bound_box =transform * Eigen::Vector3d(x, y, z);
             VoxelState point_state=checkVoxelState(point_in_bound_box);

             if(point_state==VoxelState::occupied)
             {
                 return false;
             }

            }

        }
    }


    return true;

}

bool CollisionDetection::isTraversable(const HybridAStar::Node3D *node){

    //  this function from chatgpt is error that it provides a solution calculating
    //  a bound box using octomap->begin_leafs_bbx(min,max);
    //  the bound box axis is fixed, two point are given to determine a cuboid
    //  but the min and max point are opposite, when rotation happens, the
    //  bound box are smaller than we imagined.


    float length=2;
    float width =1;
    float height=1;   //  TODO,   get params from launch


    float current_x=node->getX();
    float current_y=node->getY();
    float current_z=node->getZ();



    // Convert the yaw to a rotation matrix
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = Eigen::Vector3d(current_x, current_y, current_z);
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    transform.linear() = rot;

    float resolution=0.5;
    for (float x= -length/2.0; x<= length/2.0; x+=resolution)
    {

        for(float y= -width/2.0; y<=width/2.0; y+=resolution)
        {

            for (float z= -height/2.0; z<= height/2.0; z+=resolution)
            {
                Eigen::Vector3d point_in_bound_box =transform * Eigen::Vector3d(x, y, z);
                VoxelState point_state=checkVoxelState(point_in_bound_box);

                if(point_state==VoxelState::occupied)
                {
                    return false;
                }

            }

        }
    }

    return true;

}