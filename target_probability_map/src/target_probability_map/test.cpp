
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <vector>
#include <ctime>


struct testPoint
{
    float x;
    float y;
    float z;
    float intensity;
    float p;

    testPoint(float _x,float _y,float _z)
    {
        x=_x;
        y=_y;
        z=_z;
        p=0.5;
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(testPoint,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (float, p, p)
)

int main ()
{
    srand ((unsigned int) time (NULL));


    typedef pcl::PointCloud<testPoint> PointCloud;
    PointCloud::Ptr cloud_(new PointCloud);


    typedef pcl::octree::OctreePointCloudSearch<testPoint> OctreeType;
    typename OctreeType::Ptr octree_(new OctreeType(0.1f));





    for (float x = -1.0; x <= 1.0; x += 0.1)
    {
        for (float y = -1.0; y <= 1.0; y += 0.1)
        {
            for (float z = -1.0; z <= 1.0; z += 0.1)
            {
                testPoint point(x,y,z);
                point.p = x + y + z; // 假设给p赋值
                cloud_->push_back(point);
            }
        }
    }

    std::cout<<"cloud_->size()"<<cloud_->size()<<std::endl;
    octree_->setInputCloud(cloud_);
    octree_->addPointsFromInputCloud();
    // 定义查询的范围（voxel的立方体边长）

    std::cout<<"octree_->getTreeDepth();=="<<octree_->getTreeDepth()<<std::endl;

    std::cout<<"octree_ getInputCloud()->size()=="<<octree_->getInputCloud()->points.size()<<std::endl;


    // 查询一个具体的voxel（立方体中心的位置）
    float voxel_center_x = 0.5;
    float voxel_center_y = 0.5;
    float voxel_center_z = 0.5;

    // 在octree中进行voxelSearch，找到位于指定voxel内的点的索引
    testPoint search_point(voxel_center_x,voxel_center_y,voxel_center_z);

    std::vector<int> points;
    int K = 10;

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    if(octree_->voxelSearch(search_point,points))
    {
            std::cout<<"octree_->voxelSearch"<<std::endl;
    }

    return 0;



}