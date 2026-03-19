#pragma once
#include <Eigen/Core>
#include <vector>
#include <limits>

namespace dmap_navigation {

struct Cell {
    float distance = std::numeric_limits<float>::max();
    Eigen::Vector2f gradient = Eigen::Vector2f::Zero();
    bool occupied = false;
};

class DMap {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DMap(int width, int height, float resolution);
    
    // Calcola la DMap partendo da una OccupancyGrid
    void compute(const std::vector<int8_t>& grid_data);
    
    //Restituisce distanza e gradiente per una posizione (x,y) nello spazio
    float getDistance(const Eigen::Vector2f& world_pos) const;
    Eigen::Vector2f getGradient(const Eigen::Vector2f& world_pos) const;

private:
    int _width, _height;
    float _resolution;
    std::vector<Cell> _cells;
};

} // namespace dmap_navigation