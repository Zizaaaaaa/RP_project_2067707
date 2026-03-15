#include "dmap_navigation/dmap.h"
#include <iostream>

int main() {
    dmap_navigation::DMap dmap(10, 10, 1.0);
    std::vector<int8_t> data(100, 0);
    data[55] = 100;
    dmap.compute(data);
    std::cout << "Distanza in (0,0): " << dmap.getDistance(Eigen::Vector2f(0,0)) << " m" << std::endl;
    std::cout << "Distanza in (5,5): " << dmap.getDistance(Eigen::Vector2f(5,5)) << " m" << std::endl;
    return 0;
}