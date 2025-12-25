#include "tdoa_newton_raphson.hpp"

#include <Eigen/QR>

namespace tdoa_estimator {

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

    // Main Newton-Raphson function (3D)
    SolverResult newtonRaphson(const PosMatrix& anchorPositionsLeft,
                        const PosMatrix& anchorPositionsRight,
                        const DynVector& doas,
                        DynVector initialPos,
                        const int maxIterations,
                        const double convergenceThreshold) {
        
        DynVector pos = initialPos;
        SolverResult result;
        result.converged = false;
        result.valid = true;
        result.iterations = 0;

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
        // 0.3m threshold is suitable for typical indoor UWB setups
        if (result.rmse > 0.8) {    // TODO: Make it configurable
            result.valid = false; 
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
                            const double convergenceThreshold) {
        
        PosVector2D posXY = initialPosXY;
        SolverResult2D result;
        result.converged = false;
        result.valid = true;
        result.iterations = 0;

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
        // 0.3m threshold is suitable for typical indoor UWB setups
        // TODO: Make this configurable
        if (result.rmse > 0.8) {
            result.valid = false;
        }

        return result;
    }
}
