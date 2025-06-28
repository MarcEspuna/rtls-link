#pragma once

#include <Eigen.h>
#include <Eigen/QR>

#include <random>
#include <cmath>

namespace tdoa_estimator {

    static constexpr size_t kMaxCapacity = 10;

    // Eigen types:
    using PosMatrix = Eigen::Matrix<double, Eigen::Dynamic, 3, 0, kMaxCapacity, 3>;
    using DynVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxCapacity, 1>;
    using PosVector2D = Eigen::Matrix<double, 2, 1>;

    // Main Newton-Raphson function (3D)
    DynVector newtonRaphson(const PosMatrix& anchorPositionsLeft,
                        const PosMatrix& anchorPositionsRight,
                        const DynVector& doas,
                        DynVector initialPos,
                        const int numUpdates);

    // Main Newton-Raphson function (2D)
    PosVector2D newtonRaphson2D(const PosMatrix& anchorPositionsLeft,
                            const PosMatrix& anchorPositionsRight,
                            const DynVector& doas,
                            PosVector2D initialPos,
                            double fixedZ,
                            const int numUpdates);

    struct TDoAMeasurement {
        int anchor_a;
        int anchor_b;
        double tdoa;
        uint64_t timestamp;
    };


}
