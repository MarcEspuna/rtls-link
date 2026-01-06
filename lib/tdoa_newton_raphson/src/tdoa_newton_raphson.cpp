#include "tdoa_newton_raphson.hpp"

#include <Eigen/QR>
#include <Eigen/Cholesky>

namespace tdoa_estimator {

    // Minimum measurement variance floor (1cm std dev squared)
    static constexpr double kMinMeasurementVariance = 0.0001;

    // Regularization threshold - if min diagonal of JtJ is this fraction of trace, add regularization
    static constexpr double kRegularizationThreshold = 1e-8;

    // Regularization factor (lambda = kRegularizationFactor * trace(JtJ))
    static constexpr double kRegularizationFactor = 1e-6;

    // Helper to calculate residuals
    static DynVector calculateResiduals(const PosMatrix& anchorPositionsLeft,
                                      const PosMatrix& anchorPositionsRight,
                                      const DynVector& doas,
                                      const DynVector& currentPos) {
        const int numAnchors = anchorPositionsLeft.rows();
        DynVector distancesLeft(numAnchors);
        DynVector distancesRight(numAnchors);
        
        for(int i = 0; i < numAnchors; ++i) {
            distancesLeft(i) = (anchorPositionsLeft.row(i).transpose() - currentPos).norm();
            distancesRight(i) = (anchorPositionsRight.row(i).transpose() - currentPos).norm();
        }
        
        // Residual = (d_left - d_right) - measurement
        // Ideally this is 0.
        return (distancesLeft - distancesRight) - doas;
    }

    // Newton-Raphson step function
    static DynVector newtonRaphsonStep(const PosMatrix& anchorPositionsLeft,
                            const PosMatrix& anchorPositionsRight,
                            const DynVector& doas,
                            const DynVector& initialPos,
                            const DynVector& residuals) {
        const int numAnchors = anchorPositionsLeft.rows();
        
        // We need distances again for the Jacobian (Gradient)
        // Optimization: We could pass these in, but for clarity/safety recalculating
        // or we can reconstruct from residuals if we had d_left/d_right stored.
        // For < 10 anchors, recalculation is negligible cost compared to QR decomp.
        
        PosMatrix gradF(numAnchors, 3);
        for(int i = 0; i < numAnchors; ++i) {
             double dL = (anchorPositionsLeft.row(i).transpose() - initialPos).norm();
             double dR = (anchorPositionsRight.row(i).transpose() - initialPos).norm();

             // Avoid division by zero
             if (dL < 1e-6) dL = 1e-6;
             if (dR < 1e-6) dR = 1e-6;

            gradF.row(i) = (initialPos - anchorPositionsLeft.row(i).transpose()).transpose() / dL -
                        (initialPos - anchorPositionsRight.row(i).transpose()).transpose() / dR;
        }

        // Return step: - (J^T * J)^-1 * J^T * f  (Gauss-Newton direction)
        // solved via QR decomposition of J * step = -f
        return gradF.completeOrthogonalDecomposition().solve(residuals);
    }

    // Newton-Raphson step function (2D)
    static PosVector2D newtonRaphsonStep2D(const PosMatrix& anchorPositionsLeft,
                                const PosMatrix& anchorPositionsRight,
                                const DynVector& doas,
                                const PosVector2D& initialPosXY,
                                double fixedZ,
                                const DynVector& residuals) {
        const int numAnchors = anchorPositionsLeft.rows();
        
        // Construct 3D position from 2D estimate and fixed Z
        DynVector currentPos3D(3);
        currentPos3D << initialPosXY(0), initialPosXY(1), fixedZ;

        // Calculate Jacobian (Gradient of f w.r.t X and Y)
        Eigen::Matrix<double, Eigen::Dynamic, 2, 0, kMaxCapacity, 2> gradF_XY(numAnchors, 2);
        for(int i = 0; i < numAnchors; ++i) {
            Eigen::Vector3d diffL = currentPos3D.transpose() - anchorPositionsLeft.row(i);
            Eigen::Vector3d diffR = currentPos3D.transpose() - anchorPositionsRight.row(i);
            
            double dL = diffL.norm();
            double dR = diffR.norm();

             if (dL < 1e-6) dL = 1e-6;
             if (dR < 1e-6) dR = 1e-6;

            // Grad = (pos - anchor) / dist
            // We want gradients w.r.t X and Y only
            Eigen::Vector3d gradL3 = diffL / dL;
            Eigen::Vector3d gradR3 = diffR / dR;

            gradF_XY.row(i) = (gradL3 - gradR3).head<2>().transpose();
        }

        // Solve for the step in X and Y
        return gradF_XY.completeOrthogonalDecomposition().solve(residuals);
    }

    /**
     * @brief Computes 3D position covariance from Jacobian at converged position
     *
     * Uses the Gauss-Newton approximation: P = (J^T * J)^-1 * sigma^2
     * where sigma^2 is the measurement variance estimated from residuals.
     *
     * Tikhonov regularization is applied when J^T*J is ill-conditioned.
     *
     * @param anchorPositionsLeft  Left anchor positions for each measurement
     * @param anchorPositionsRight Right anchor positions for each measurement
     * @param finalPosition        Converged position estimate
     * @param measurementVariance  Variance of measurements (sigma^2)
     * @param outCovariance        Output 3x3 covariance matrix
     * @return true if covariance computation succeeded
     */
    static bool computePositionCovariance3D(
        const PosMatrix& anchorPositionsLeft,
        const PosMatrix& anchorPositionsRight,
        const DynVector& finalPosition,
        double measurementVariance,
        CovMatrix3D& outCovariance)
    {
        const int numMeasurements = anchorPositionsLeft.rows();

        // Recompute Jacobian at final position
        Eigen::Matrix<double, Eigen::Dynamic, 3, 0, kMaxCapacity, 3> J(numMeasurements, 3);

        for (int i = 0; i < numMeasurements; ++i) {
            double dL = (anchorPositionsLeft.row(i).transpose() - finalPosition).norm();
            double dR = (anchorPositionsRight.row(i).transpose() - finalPosition).norm();

            // Avoid division by zero
            if (dL < 1e-6) dL = 1e-6;
            if (dR < 1e-6) dR = 1e-6;

            J.row(i) = (finalPosition - anchorPositionsLeft.row(i).transpose()).transpose() / dL -
                       (finalPosition - anchorPositionsRight.row(i).transpose()).transpose() / dR;
        }

        // Compute Fisher Information Matrix approximation: J^T * J
        CovMatrix3D JtJ = J.transpose() * J;

        // Check if regularization is needed using trace and min diagonal
        double trace = JtJ.trace();
        double minDiag = JtJ.diagonal().minCoeff();

        if (trace < 1e-10) {
            // Matrix is essentially zero - cannot compute covariance
            outCovariance = CovMatrix3D::Identity() * 100.0;  // Large uncertainty
            return false;
        }

        // Apply Tikhonov regularization if ill-conditioned
        if (minDiag < kRegularizationThreshold * trace) {
            double lambda = kRegularizationFactor * trace;
            JtJ += lambda * CovMatrix3D::Identity();
        }

        // Use LDLT decomposition for symmetric positive semi-definite matrix
        Eigen::LDLT<CovMatrix3D> ldlt(JtJ);

        if (ldlt.info() != Eigen::Success) {
            outCovariance = CovMatrix3D::Identity() * 100.0;
            return false;
        }

        // Compute covariance: (J^T * J)^-1 * sigma^2
        outCovariance = ldlt.solve(CovMatrix3D::Identity()) * measurementVariance;

        // Ensure result is symmetric (numerical precision)
        outCovariance = (outCovariance + outCovariance.transpose()) / 2.0;

        return true;
    }

    /**
     * @brief Computes 2D (XY) position covariance from Jacobian
     */
    static bool computePositionCovariance2D(
        const PosMatrix& anchorPositionsLeft,
        const PosMatrix& anchorPositionsRight,
        const PosVector2D& finalPositionXY,
        double fixedZ,
        double measurementVariance,
        CovMatrix2D& outCovariance)
    {
        const int numMeasurements = anchorPositionsLeft.rows();

        // Construct 3D position
        Eigen::Vector3d finalPos3D;
        finalPos3D << finalPositionXY(0), finalPositionXY(1), fixedZ;

        // Compute 2D Jacobian (Nx2)
        Eigen::Matrix<double, Eigen::Dynamic, 2, 0, kMaxCapacity, 2> J_XY(numMeasurements, 2);

        for (int i = 0; i < numMeasurements; ++i) {
            Eigen::Vector3d diffL = finalPos3D - anchorPositionsLeft.row(i).transpose();
            Eigen::Vector3d diffR = finalPos3D - anchorPositionsRight.row(i).transpose();

            double dL = diffL.norm();
            double dR = diffR.norm();

            if (dL < 1e-6) dL = 1e-6;
            if (dR < 1e-6) dR = 1e-6;

            Eigen::Vector3d gradL3 = diffL / dL;
            Eigen::Vector3d gradR3 = diffR / dR;

            J_XY.row(i) = (gradL3 - gradR3).head<2>().transpose();
        }

        // Compute J^T * J (2x2)
        CovMatrix2D JtJ = J_XY.transpose() * J_XY;

        // Check if regularization is needed
        double trace = JtJ.trace();
        double minDiag = JtJ.diagonal().minCoeff();

        if (trace < 1e-10) {
            outCovariance = CovMatrix2D::Identity() * 100.0;
            return false;
        }

        // Apply Tikhonov regularization if ill-conditioned
        if (minDiag < kRegularizationThreshold * trace) {
            double lambda = kRegularizationFactor * trace;
            JtJ += lambda * CovMatrix2D::Identity();
        }

        // Use LDLT for 2x2 matrix
        Eigen::LDLT<CovMatrix2D> ldlt(JtJ);

        if (ldlt.info() != Eigen::Success) {
            outCovariance = CovMatrix2D::Identity() * 100.0;
            return false;
        }

        outCovariance = ldlt.solve(CovMatrix2D::Identity()) * measurementVariance;
        outCovariance = (outCovariance + outCovariance.transpose()) / 2.0;

        return true;
    }

    // Main Newton-Raphson function (3D)
    SolverResult newtonRaphson(const PosMatrix& anchorPositionsLeft,
                        const PosMatrix& anchorPositionsRight,
                        const DynVector& doas,
                        DynVector initialPos,
                        const int maxIterations,
                        const double convergenceThreshold,
                        const double rmseThreshold) {
        
        DynVector pos = initialPos;
        SolverResult result;
        result.converged = false;
        result.valid = true;
        result.iterations = 0;
        result.covarianceValid = false;
        result.positionCovariance = CovMatrix3D::Identity();

        for(int i = 0; i < maxIterations; ++i) {
            result.iterations++;
            
            DynVector residuals = calculateResiduals(anchorPositionsLeft, anchorPositionsRight, doas, pos);
            DynVector step = newtonRaphsonStep(anchorPositionsLeft, anchorPositionsRight, doas, pos, residuals);
            
            // Apply step (Newton-Raphson: pos_new = pos_old - step)
            // Note: The .solve() returns 'x' for 'Ax = b'. 
            // If we set up J * delta = f, then delta = J_pseudoInv * f.
            // We want f(pos + delta) ~ f(pos) + J*delta = 0  => J*delta = -f
            // The .solve() call above solved J * step = f (positive residuals). 
            // So we subtract the result.
            pos = pos - step;

            if (step.norm() < convergenceThreshold) {
                result.converged = true;
                break;
            }
        }

        result.position = pos;

        // Final Quality Check
        DynVector finalResiduals = calculateResiduals(anchorPositionsLeft, anchorPositionsRight, doas, pos);
        result.rmse = std::sqrt(finalResiduals.squaredNorm() / doas.size());

        // Sanity check: If RMSE exceeds threshold, result is likely unreliable
        if (result.rmse > rmseThreshold) {
            result.valid = false;
        }

        // Compute covariance if solution is valid and converged
        if (result.valid && result.converged) {
            const int numMeasurements = static_cast<int>(doas.size());
            const int stateDimension = 3;

            // Estimate measurement variance using degrees-of-freedom corrected estimator
            double measurementVariance;
            if (numMeasurements > stateDimension) {
                // Unbiased variance estimator: sigma^2 = SSE / (N - d)
                measurementVariance = finalResiduals.squaredNorm() / (numMeasurements - stateDimension);
            } else {
                // Not enough measurements for unbiased estimate, use RMSE^2
                measurementVariance = result.rmse * result.rmse;
            }

            // Apply minimum variance floor (1cm std dev)
            measurementVariance = std::max(measurementVariance, kMinMeasurementVariance);

            result.covarianceValid = computePositionCovariance3D(
                anchorPositionsLeft,
                anchorPositionsRight,
                pos,
                measurementVariance,
                result.positionCovariance
            );
        }

        return result;
    }

    // Main Newton-Raphson function (2D)
    SolverResult2D newtonRaphson2D(const PosMatrix& anchorPositionsLeft,
                            const PosMatrix& anchorPositionsRight,
                            const DynVector& doas,
                            PosVector2D initialPosXY,
                            double fixedZ,
                            const int maxIterations,
                            const double convergenceThreshold,
                            const double rmseThreshold) {
        
        PosVector2D posXY = initialPosXY;
        SolverResult2D result;
        result.converged = false;
        result.valid = true;
        result.iterations = 0;
        result.covarianceValid = false;
        result.positionCovariance = CovMatrix2D::Identity();

        for(int i = 0; i < maxIterations; ++i) {
            result.iterations++;

            // Construct 3D pos for residual calc
            DynVector currentPos3D(3);
            currentPos3D << posXY(0), posXY(1), fixedZ;

            DynVector residuals = calculateResiduals(anchorPositionsLeft, anchorPositionsRight, doas, currentPos3D);
            PosVector2D step = newtonRaphsonStep2D(anchorPositionsLeft, anchorPositionsRight, doas, posXY, fixedZ, residuals);

            posXY = posXY - step;

            if (step.norm() < convergenceThreshold) {
                result.converged = true;
                break;
            }
        }

        result.position = posXY;

        DynVector currentPos3D(3);
        currentPos3D << posXY(0), posXY(1), fixedZ;
        DynVector finalResiduals = calculateResiduals(anchorPositionsLeft, anchorPositionsRight, doas, currentPos3D);
        result.rmse = std::sqrt(finalResiduals.squaredNorm() / doas.size());

        // Sanity check: If RMSE exceeds threshold, result is likely unreliable
        if (result.rmse > rmseThreshold) {
            result.valid = false;
        }

        // Compute covariance if solution is valid and converged
        if (result.valid && result.converged) {
            const int numMeasurements = static_cast<int>(doas.size());
            const int stateDimension = 2;  // 2D mode

            // Estimate measurement variance
            double measurementVariance;
            if (numMeasurements > stateDimension) {
                measurementVariance = finalResiduals.squaredNorm() / (numMeasurements - stateDimension);
            } else {
                measurementVariance = result.rmse * result.rmse;
            }

            measurementVariance = std::max(measurementVariance, kMinMeasurementVariance);

            result.covarianceValid = computePositionCovariance2D(
                anchorPositionsLeft,
                anchorPositionsRight,
                posXY,
                fixedZ,
                measurementVariance,
                result.positionCovariance
            );
        }

        return result;
    }
}
