#pragma once

#include <Eigen.h>
#include <Eigen/QR>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace tdoa_estimator {

    static constexpr size_t kMaxCapacity = 32; // Max anchor pairs per solve

    // Solver scalar type: float for ESP32 hardware FPU.
    // Stability is preserved by solving J*Δ = r via QR (not normal equations)
    // and Levenberg-Marquardt damping inside the iteration.
    using Scalar = float;

    // Eigen types (iteration in float):
    using PosMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, kMaxCapacity, 3>;
    using DynVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, kMaxCapacity, 1>;
    using PosVector2D = Eigen::Matrix<Scalar, 2, 1>;
    using PosVector3D = Eigen::Matrix<Scalar, 3, 1>;

    // 2D-Jacobian shape (Nx2) used by the 2D solver.
    using PosMatrix2 = Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, kMaxCapacity, 2>;

    // Covariance kept in double - matrix inverse is precision-sensitive.
    using CovMatrix3D = Eigen::Matrix<double, 3, 3>;
    using CovMatrix2D = Eigen::Matrix<double, 2, 2>;

    struct SolverResult {
        PosVector3D position;            // 3D position
        Scalar rmse;                     // Root Mean Square Error of residuals (m)
        int iterations;                  // Number of iterations performed
        bool converged;                  // True if step/residual delta met threshold
        bool valid;                      // True if converged, within RMSE threshold, and observable
        CovMatrix3D positionCovariance;  // 3x3 position covariance (double)
        bool covarianceValid;            // True if covariance computation succeeded
    };

    struct SolverResult2D {
        PosVector2D position;
        Scalar rmse;
        int iterations;
        bool converged;
        bool valid;                      // True if converged and within RMSE threshold
        CovMatrix2D positionCovariance;
        bool covarianceValid;
    };

    struct TDoAMeasurement {
        int anchor_a;
        int anchor_b;
        Scalar tdoa;
        uint64_t timestamp;
    };

    // Anisotropic null-space prior (Change 2). After convergence the solver blends
    // the data solution toward `position` along the *weak* eigendirections of JᵀJ
    // via a per-axis Gaussian MAP combine. Well-observed axes are left untouched
    // (the data precision dominates), so no latency is added to observable axes;
    // only the near-null axis (typically Z under near-coplanar anchors) is pinned
    // to the previous estimate instead of being driven by amplified noise.
    //
    // The shrink toward the prior along eigendirection v_i with eigenvalue d_i is
    //   s_i = rho_prior / (rho_data_i + rho_prior),
    //   rho_data_i = d_i / measurementVariance,  rho_prior = 1 / sigma_m^2.
    // Disabled by default => exact legacy behaviour. The reported covariance is
    // deliberately NOT updated by this blend (it stays data-only) so a downstream
    // filter does not double-count the prior across time.
    struct NullspacePrior {
        bool enabled = false;
        PosVector3D position = PosVector3D::Zero(); // previous estimate (x_prior)
        Scalar sigma_m = Scalar(0);                 // prior std-dev (m); <=0 disables
    };

    // Main Newton-Raphson function (3D). Levenberg-Marquardt damped Gauss-Newton
    // with QR-based step solve and warm-start. Defaults tuned for UWB noise (~5-10cm).
    SolverResult newtonRaphson(const PosMatrix& anchorPositionsLeft,
                               const PosMatrix& anchorPositionsRight,
                               const DynVector& doas,
                               PosVector3D initialPos,
                               int maxIterations = 5,
                               Scalar convergenceThreshold = 1e-3f,
                               Scalar rmseThreshold = 0.8f,
                               const NullspacePrior& prior = {});

    SolverResult newtonRaphsonWeighted(const PosMatrix& anchorPositionsLeft,
                                       const PosMatrix& anchorPositionsRight,
                                       const DynVector& doas,
                                       const DynVector& weights,
                                       PosVector3D initialPos,
                                       int maxIterations = 5,
                                       Scalar convergenceThreshold = 1e-3f,
                                       Scalar rmseThreshold = 0.8f,
                                       const NullspacePrior& prior = {});

    void computeResiduals3D(const PosMatrix& anchorPositionsLeft,
                            const PosMatrix& anchorPositionsRight,
                            const DynVector& doas,
                            const PosVector3D& position,
                            DynVector& residuals);

    // Main Newton-Raphson function (2D) - solves XY with Z fixed.
    SolverResult2D newtonRaphson2D(const PosMatrix& anchorPositionsLeft,
                                   const PosMatrix& anchorPositionsRight,
                                   const DynVector& doas,
                                   PosVector2D initialPos,
                                   Scalar fixedZ,
                                   int maxIterations = 5,
                                   Scalar convergenceThreshold = 1e-3f,
                                   Scalar rmseThreshold = 0.8f);

}
