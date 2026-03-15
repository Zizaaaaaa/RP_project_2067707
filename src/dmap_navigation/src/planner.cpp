#include "dmap_navigation/planner.h"
#include <queue>
#include <algorithm>
#include <vector>

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

struct NodeRecord {
    int index;
    float cost_so_far;
    //per ordinare dal costo più basso la prioriy queue
    bool operator>(const NodeRecord& other) const {
        return cost_so_far > other.cost_so_far;
    }
};

std::vector<Eigen::Vector2f> Planner::plan(const Eigen::Vector2f& start, const Eigen::Vector2f& goal) {
    std::vector<Eigen::Vector2f> path;
    
    //assumo mappa 1000x1000 con risoluzione 0.05
    int width = 1000; 
    int height = 1000;
    float res = 0.05;

    int start_x = std::round(start.x() / res);
    int start_y = std::round(start.y() / res);
    int goal_x = std::round(goal.x() / res);
    int goal_y = std::round(goal.y() / res);

    if (start_x < 0 || start_x >= width || goal_x < 0 || goal_x >= width) return path; // Fuori mappa

    std::vector<float> min_cost(width * height, 1e9);
    std::vector<int> parent(width * height, -1);
    std::priority_queue<NodeRecord, std::vector<NodeRecord>, std::greater<NodeRecord>> open_list;

    int start_idx = start_y * width + start_x;
    int goal_idx = goal_y * width + goal_x;

    min_cost[start_idx] = 0;
    open_list.push({start_idx, 0});

    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    bool diag[] = {false, false, false, false, true, true, true, true};

    bool found = false;

    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        if (current.index == goal_idx) {
            found = true;
            break;
        }

        if (current.cost_so_far > min_cost[current.index]) continue;

        int cx = current.index % width;
        int cy = current.index / width;

        for (int i = 0; i < 8; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                int n_idx = ny * width + nx;
                float transition_cost = computeCost(nx, ny, diag[i]);
                
                if (transition_cost >= 1e5) continue; //cella occupata

                float new_cost = current.cost_so_far + transition_cost;

                if (new_cost < min_cost[n_idx]) {
                    min_cost[n_idx] = new_cost;
                    parent[n_idx] = current.index;
                    open_list.push({n_idx, new_cost});
                }
            }
        }
    }

    if (found) {
        int curr = goal_idx;
        while (curr != -1) {
            int cx = curr % width;
            int cy = curr / width;
            path.push_back(Eigen::Vector2f(cx * res, cy * res));
            curr = parent[curr];
        }
        std::reverse(path.begin(), path.end());
    }

    return path;
}

}
 //namespace