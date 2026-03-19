#pragma once
#include "dmap_navigation/dmap.h"
#include <vector>

namespace dmap_navigation {

struct PathNode {
    Eigen::Vector2i grid_pos;
    float cost;
    // Per ricostruire il percorso
    int parent_idx = -1;
};

class Planner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Planner(const DMap* dmap) : _dmap(dmap) {}
    //trova il path usando Dijkstra/A*
    std::vector<Eigen::Vector2f> plan(const Eigen::Vector2f& start, const Eigen::Vector2f& goal);
private:
    const DMap* _dmap;
    float computeCost(int x, int y, bool diagonal);
};

}