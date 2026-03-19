#include "dmap_navigation/localizer.h"

namespace dmap_navigation {

Eigen::Isometry2f Localizer::localize(const std::vector<Eigen::Vector2f>& scan_points, 
                                      const Eigen::Isometry2f& initial_guess, 
                                      int iterations) {
    Eigen::Isometry2f X = initial_guess;

    for (int i = 0; i < iterations; ++i) {
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f b = Eigen::Vector3f::Zero();

        for (const auto& p_robot : scan_points) {
            Eigen::Vector2f p_map = X * p_robot;
            float d = _dmap->getDistance(p_map);
            Eigen::Vector2f grad = _dmap->getGradient(p_map);

            if (d >= 10.0f) continue;

            Eigen::Matrix<float, 1, 3> J;
            J(0, 0) = grad.x();
            J(0, 1) = grad.y();
            
            Eigen::Vector2f p_rot(-p_robot.y(), p_robot.x());
            J(0, 2) = grad.dot(X.linear() * p_rot);

            H += J.transpose() * J;
            b += J.transpose() * d;
        }

        //regolarizzazione
        H += Eigen::Matrix3f::Identity() * 1.0f;

        Eigen::Vector3f delta_X = H.ldlt().solve(-b);

        X.translation().x() += delta_X.x();
        X.translation().y() += delta_X.y();
        
        Eigen::Rotation2Df current_rot(X.linear());
        X.linear() = Eigen::Rotation2Df(current_rot.angle() + delta_X.z()).matrix();
    }
    return X;
}

}