#include "dmap_navigation/dmap.h"
#include <queue>
#include <cmath>

namespace dmap_navigation {

DMap::DMap(int width, int height, float resolution)
    : _width(width), _height(height), _resolution(resolution) {
    _cells.resize(width * height);
}

void DMap::compute(const std::vector<int8_t>& grid_data) {
    //  Inizializzo ostacoli a 0, il resto a infinito
    std::priority_queue<std::pair<float, int>, 
                        std::vector<std::pair<float, int>>, 
                        std::greater<std::pair<float, int>>> pq;
    for (int i = 0; i < (int)grid_data.size(); ++i) {
        if (grid_data[i] > 50) { // cella occupata (soglia standard ROS)
            _cells[i].distance = 0.0f;
            _cells[i].occupied = true;
            pq.push({0.0f, i});
        } else {
            _cells[i].distance = std::numeric_limits<float>::max();
            _cells[i].occupied = false;
        }
    }

    //espansione di Dijkstra (8-connettività)
    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    float weights[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};
    while (!pq.empty()) {
        float d = pq.top().first;
        int idx = pq.top().second;
        pq.pop();
        if (d > _cells[idx].distance) continue;
        int cx = idx % _width;
        int cy = idx / _width;
        for (int i = 0; i < 8; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];
            if (nx >= 0 && nx < _width && ny >= 0 && ny < _height) {
                int n_idx = ny * _width + nx;
                float new_dist = d + weights[i] * _resolution;
                if (new_dist < _cells[n_idx].distance) {
                    _cells[n_idx].distance = new_dist;
                    pq.push({new_dist, n_idx});
                }
            }
        }
    }

    // Calcolo dei Gradienti (Centrale) per Gauss-Newton
    // necessario per la localizzazione: grad = [ (d(x+1)-d(x-1))/2, (d(y+1)-d(y-1))/2 ]
    for (int y = 1; y < _height - 1; ++y) {
        for (int x = 1; x < _width - 1; ++x) {
            int idx = y * _width + x;
            float dx_val = (_cells[idx + 1].distance - _cells[idx - 1].distance) / (2.0f * _resolution);
            float dy_val = (_cells[idx + _width].distance - _cells[idx - _width].distance) / (2.0f * _resolution);
            _cells[idx].gradient = Eigen::Vector2f(dx_val, dy_val);
        }
    }
}

float DMap::getDistance(const Eigen::Vector2f& world_pos) const {
    // Conversione da metri a pixel
    int x = std::round(world_pos.x() / _resolution);
    int y = std::round(world_pos.y() / _resolution);
    if (x < 0 || x >= _width || y < 0 || y >= _height) return 100.0f; // Fuori mappa
    return _cells[y * _width + x].distance;
}

Eigen::Vector2f DMap::getGradient(const Eigen::Vector2f& world_pos) const {
    int x = std::round(world_pos.x() / _resolution);
    int y = std::round(world_pos.y() / _resolution);
    if (x < 0 || x >= _width || y < 0 || y >= _height) return Eigen::Vector2f::Zero();
    return _cells[y * _width + x].gradient;
}

} //        namespace dmap_navigation