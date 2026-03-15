#include "dmap_navigation/planner.h"
#include <queue>

namespace dmap_navigation {

float Planner::computeCost(int x, int y, bool diagonal) {
    float dist = _dmap->getDistance(Eigen::Vector2f(x * 0.1, y * 0.1)); // Assumendo res 0.1
    float obstacle_cost = 0;
    
    float robot_radius = 0.3;    
    float influence_range = 1.0;

    if (dist < robot_radius) return 1e6; //colpito ostacolo
    
    if (dist < influence_range) {
        // Costo che aumenta linearmente avvicinandosi all'ostacolo
        obstacle_cost = 10.0 * (influence_range - dist) / (influence_range - robot_radius);
    }

    float traversal_cost = diagonal ? 1.414 : 1.0;
    return traversal_cost + obstacle_cost;
}

// Qui andrebbe l'implementazione di Dijkstra che espande i nodi dal goal allo start
// o viceversa, usando una priority_queue.
}