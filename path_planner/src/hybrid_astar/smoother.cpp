#include "hybrid_astar/smoother.h"

using namespace HybridAStar;



void Smoother::debugPath() {

    for(auto point:fourDpath)
    {
        std::cout<<"point== ("<<point.getX()<<","<<point.getY()<<","
        <<point.getZ()<<")  "<<std::endl;
    }

}

Eigen::Vector3d Smoother::obstacleTerm(Eigen::Vector3d xi,
                             const HybridAStar::CollisionDetection *configurationSpace){

  Eigen::Vector3d obstacleGradient;
  double d_threshold=2.5;
  double distance;
  if(configurationSpace->
  getDistanceAndGradientAtPosition(xi, &distance,&obstacleGradient))
  {
      if(distance<d_threshold)
      {
          return -wObstacle*obstacleGradient;
      }

  }

      return Eigen::Vector3d(0,0,0);
}

Eigen::Vector3d Smoother::smoothnessTerm(Eigen::Vector3d xim3, Eigen::Vector3d xim2, Eigen::Vector3d xim1,
                                         Eigen::Vector3d xi, Eigen::Vector3d xip1, Eigen::Vector3d xip2,
                                         Eigen::Vector3d xip3) {

    Eigen::Vector3d gradient= wAccSmooth * (-xim3+ 6*xim2 - 15*xim1 + 20*xi - 15*xip1 + 6*xip2 -xip3)+
                              wVelSmooth * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
    return gradient;

}


void  Smoother::tracePath(const Node4D* node, int i, std::vector<Node4D> path )
{
    if (node == nullptr) {
        std::reverse(path.begin(),path.end());
        this->fourDpath =path;
        return;
    }

    i++;
    path.push_back(*node);
    tracePath(node->getPred(), i, path);
}


bool Smoother::smooth(const HybridAStar::CollisionDetection *configurationSpace) {

    auto start = std::chrono::high_resolution_clock::now();
    int iterations = 0;
    // the maximum iterations for the gd smoother
    int maxIterations = 100;
    // the lenght of the path in number of nodes
    int pathLength = fourDpath.size();

    std::vector<Node4D> newPath = fourDpath;

//    float totalWeight = wSmoothness + wCurvature + wObstacle;
    float totalWeight = wAccSmooth + wVelSmooth +  wObstacle;
    std::vector<Node4D> temp_path=newPath;

    while (iterations < maxIterations) {
        // choose the first three nodes of the path
        for (int i = 3; i < pathLength - 3; ++i) {
            // x i minus 2 x_(i+2)
            Eigen::Vector3d xim3(newPath[i - 3].getX(), newPath[i - 3].getY(), newPath[i - 3].getZ());
            Eigen::Vector3d xim2(newPath[i - 2].getX(), newPath[i - 2].getY(),newPath[i - 2].getZ());
            Eigen::Vector3d xim1(newPath[i - 1].getX(), newPath[i - 1].getY(),newPath[i - 1].getZ());
            Eigen::Vector3d xi(newPath[i].getX(), newPath[i].getY(),newPath[i].getZ());
            Eigen::Vector3d xip1(newPath[i + 1].getX(), newPath[i + 1].getY(),newPath[i + 1].getZ());
            Eigen::Vector3d xip2(newPath[i + 2].getX(), newPath[i + 2].getY(),newPath[i + 2].getZ());
            Eigen::Vector3d xip3(newPath[i + 3].getX(), newPath[i + 3].getY(), newPath[i + 3].getZ());
            Eigen::Vector3d correction;

            correction = correction + smoothnessTerm(xim3, xim2, xim1, xi, xip1, xip2, xip3);
            correction = correction + obstacleTerm(xi,configurationSpace);

            //  gradient descent
            xi = xi - alpha * correction/totalWeight;

            temp_path[i].setX(xi[0]);
            temp_path[i].setY(xi[1]);
            temp_path[i].setZ(xi[2]);
        }
        newPath=temp_path;
        iterations++;
    }

    // adjust yaw along the velocity.
    for (int i = 3; i < pathLength - 3; ++i)
    {
        Eigen::Vector3d xi(newPath[i].getX(),newPath[i].getY(),newPath[i].getZ());
        Eigen::Vector3d xip1(newPath[i+1].getX(),newPath[i+1].getY(),newPath[i+1].getZ());
        Eigen::Vector3d Dxi = xip1 - xi;
        newPath[i].setT(std::atan2(Dxi.y(), Dxi.x()));
    }


    fourDpath = newPath;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "trajectory optimer cost==" << elapsed.count() << " ms" << std::endl;


}

