#include "dmap_navigation/localizer.h"
#include <iostream>

namespace dmap_navigation {

Eigen::Isometry2f Localizer::localize(const std::vector<Eigen::Vector2f>& scan_points, 
                                      const Eigen::Isometry2f& initial_guess, 
                                      int iterations) {
    Eigen::Isometry2f X = initial_guess;

    for (int i = 0; i < iterations; ++i) {
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f b = Eigen::Vector3f::Zero();
        float total_error = 0;

        for (const auto& p_robot : scan_points) {
            // Trasform il punto laser nel frame della mappa
            Eigen::Vector2f p_map = X * p_robot;

            float d = _dmap->getDistance(p_map);
            Eigen::Vector2f grad = _dmap->getGradient(p_map);

            if (d >= 10.0f) continue; // Salta punti fuori mappa

            // Jacobiano della trasformazione p_map = X * p_robot
            // J = [ grad_x, grad_y ] * [ I | R * J_rot * p_robot ]
            Eigen::Matrix<float, 1, 3> J;
            J(0, 0) = grad.x(); // derivata rispetto a x
            J(0, 1) = grad.y(); // derivata rispetto a y
            
            // Derivata rispetto alla rotazione (theta)
            Eigen::Vector2f p_rot(-p_robot.y(), p_robot.x());
            J(0, 2) = grad.dot(X.linear() * p_rot);

            H += J.transpose() * J;
            b += J.transpose() * d;
            total_error += d * d;
        }

        //aggiunta piccolo termine di regolarizzazione
        H += Eigen::Matrix3f::Identity() * 1.0;

        //Sistema lineare H * delta_X = -b
        Eigen::Vector3f delta_X = H.ldlt().solve(-b);

        // Aggiornamento della posa (Manifold update)
        Eigen::Translation2f translation_update(delta_X.x(), delta_X.y());
        Eigen::Rotation2Df rotation_update(delta_X.z());
        X = X * translation_update * rotation_update;
    }
    return X;
}

}