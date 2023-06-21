#include "hybrid_astar/smoother.h"
using namespace HybridAStar;



void  Smoother::tracePath(const Node4D* node, int i, std::vector<Node4D> path )
{
    if (node == nullptr) {
        this->fourDpath = path;
        return;
    }

    i++;
    path.push_back(*node);
    tracePath(node->getPred(), i, path);
}



