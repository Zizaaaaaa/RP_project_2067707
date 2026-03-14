#pragma once
#include "dmap_navigation/dmap.h"
#include <Eigen/Geometry>

namespace dmap_navigation {

class Localizer {
public:
    Localizer(const DMap* dmap) : _dmap(dmap) {}

    //n iterazioni di gauss-newton per trovare la posa giusta
    Eigen::Isometry2f localize(const std::vector<Eigen::Vector2f>& scan_points, 
                               const Eigen::Isometry2f& initial_guess, 
                               int iterations = 10);

private:
    const DMap* _dmap;
};

}