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

    // Fixed-size covariance matrices (stack-allocated, no heap allocation)
    using CovMatrix3D = Eigen::Matrix<double, 3, 3>;
    using CovMatrix2D = Eigen::Matrix<double, 2, 2>;

    struct SolverResult {
        DynVector position;           // 3D position (or 2D projected to 3D)
        double rmse;                  // Root Mean Square Error of residuals
        int iterations;               // Number of iterations performed
        bool converged;               // True if delta < threshold
        bool valid;                   // True if solution is physically plausible
        CovMatrix3D positionCovariance;  // 3x3 position covariance matrix
        bool covarianceValid;            // True if covariance computation succeeded
    };

    struct SolverResult2D {
        PosVector2D position;
        double rmse;
        int iterations;
        bool converged;
        bool valid;
        CovMatrix2D positionCovariance;  // 2x2 position covariance matrix (XY only)
        bool covarianceValid;            // True if covariance computation succeeded
    };

    // Main Newton-Raphson function (3D)
    SolverResult newtonRaphson(const PosMatrix& anchorPositionsLeft,
                        const PosMatrix& anchorPositionsRight,
                        const DynVector& doas,
                        DynVector initialPos,
                        const int maxIterations = 20,
                        const double convergenceThreshold = 1e-4,
                        const double rmseThreshold = 0.8);

    // Main Newton-Raphson function (2D)
    SolverResult2D newtonRaphson2D(const PosMatrix& anchorPositionsLeft,
                            const PosMatrix& anchorPositionsRight,
                            const DynVector& doas,
                            PosVector2D initialPos,
                            double fixedZ,
                            const int maxIterations = 20,
                            const double convergenceThreshold = 1e-4,
                            const double rmseThreshold = 0.8);

    struct TDoAMeasurement {
        int anchor_a;
        int anchor_b;
        double tdoa;
        uint64_t timestamp;
    };


}
