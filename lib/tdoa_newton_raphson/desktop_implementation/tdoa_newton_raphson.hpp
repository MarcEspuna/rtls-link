#pragma once

#include <Eigen.h>
#include <random>
#include <cmath>
#include <etl/vector.h>

namespace tdoa_estimator {

    // Main Newton-Raphson function
    Eigen::VectorXd newtonRaphson(const Eigen::MatrixXd& anchorPositionsLeft,
                        const Eigen::MatrixXd& anchorPositionsRight,
                        const Eigen::VectorXd& doas,
                        Eigen::VectorXd initialPos,
                        const int numUpdates);

    struct TDoAMeasurement {
        int anchor_a;
        int anchor_b;
        double tdoa;
        uint64_t timestamp;
    };


}
