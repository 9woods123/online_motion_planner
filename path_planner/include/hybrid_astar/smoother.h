#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "hybrid_astar/node4d.h"
#include "hybrid_astar/helper.h"
#include "hybrid_astar/constants.h"

namespace HybridAStar {
/*!
   \brief This class takes a path object and smoothes the nodes of the path.

   It also uses the Voronoi diagram as well as the configuration space.
*/
class Smoother {
 public:
  Smoother() {}

  /*!
     \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.

     During the different interations the following cost are being calculated
     obstacleCost
     curvatureCost
     smoothnessCost
     voronoiCost
  */

  /*!
     \brief Given a node pointer the path to the root node will be traced recursively
     \param node a 3D node, usually the goal node
     \param i a parameter for counting the number of nodes
  */

  void tracePath(const Node4D* node, int i = 0, std::vector<Node4D> path = std::vector<Node4D>());

  /// returns the path of the smoother object
  const std::vector<Node4D>& get4DPath() {return fourDpath;}




 private:

  /// voronoi diagram describing the topology of the map
  /// width of the map
  int width;
  /// height of the map
  int height;
  /// path to be smoothed
  std::vector<Node4D> fourDpath;

};
}
#endif // SMOOTHER_H
