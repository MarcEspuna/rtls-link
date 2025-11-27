#pragma once

#include <Eigen.h>
#include <Eigen/QR>

#include <random>
#include <cmath>

namespace tdoa_estimator {

    static constexpr size_t kMaxCapacity = 32; // Increased capacity for safety

    // Eigen types:
    using PosMatrix = Eigen::Matrix<double, Eigen::Dynamic, 3, 0, kMaxCapacity, 3>;
    using DynVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxCapacity, 1>;
    using PosVector2D = Eigen::Matrix<double, 2, 1>;

    struct SolverResult {
        DynVector position;     // 3D position (or 2D projected to 3D)
        double rmse;            // Root Mean Square Error of residuals
        int iterations;         // Number of iterations performed
        bool converged;         // True if delta < threshold
        bool valid;             // True if solution is physically plausible
    };

    struct SolverResult2D {
        PosVector2D position;
        double rmse;
        int iterations;
        bool converged;
        bool valid;
    };

    // Main Newton-Raphson function (3D)
    SolverResult newtonRaphson(const PosMatrix& anchorPositionsLeft,
                        const PosMatrix& anchorPositionsRight,
                        const DynVector& doas,
                        DynVector initialPos,
                        const int maxIterations = 20,
                        const double convergenceThreshold = 1e-4);

    // Main Newton-Raphson function (2D)
    SolverResult2D newtonRaphson2D(const PosMatrix& anchorPositionsLeft,
                            const PosMatrix& anchorPositionsRight,
                            const DynVector& doas,
                            PosVector2D initialPos,
                            double fixedZ,
                            const int maxIterations = 20,
                            const double convergenceThreshold = 1e-4);

    struct TDoAMeasurement {
        int anchor_a;
        int anchor_b;
        double tdoa;
        uint64_t timestamp;
    };


}
