#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "hybrid_astar/node4d.h"
#include "hybrid_astar/helper.h"
#include "hybrid_astar/constants.h"
#include "hybrid_astar/collisiondetection.h"

namespace HybridAStar {
/*!
   \brief This class takes a path object and smoothes the nodes of the path.

   It also uses the Voronoi diagram as well as the configuration space.
*/
class Smoother {
 public:
  Smoother() {}


    /// obstacleCost - pushes the path away from obstacles
  Eigen::Vector3d obstacleTerm(Eigen::Vector3d xi,const HybridAStar::CollisionDetection *configurationSpace);

    /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
  Eigen::Vector3d curvatureTerm();

    /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
  Eigen::Vector3d smoothnessTerm(Eigen::Vector3d xim2, Eigen::Vector3d xim1, Eigen::Vector3d xi,
                                 Eigen::Vector3d xip1, Eigen::Vector3d xip2);

  bool smooth(const HybridAStar::CollisionDetection *configurationSpace);

  void tracePath(const Node4D* node, int i = 0, std::vector<Node4D> path = std::vector<Node4D>());

  /// returns the path of the smoother object
  const std::vector<Node4D>& get4DPath() {return fourDpath;}

  void debugPath();


 private:


  std::vector<Node4D> fourDpath;


  float alpha=0.1;
  float wObstacle = 0.05;
  float wCurvature = 0.01;
  float wSmoothness = 0.2;

};
}
#endif // SMOOTHER_H
