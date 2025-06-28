#include "tdoa_newton_raphson.hpp"

#include <Eigen/QR>

namespace tdoa_estimator {

    // Newton-Raphson step function
    static DynVector newtonRaphsonStep(const PosMatrix& anchorPositionsLeft,
                            const PosMatrix& anchorPositionsRight,
                            const DynVector& doas,
                            const DynVector& initialPos) {
        const int numAnchors = anchorPositionsLeft.rows();
        DynVector distancesLeft(numAnchors);
        DynVector distancesRight(numAnchors);
        
        // Calculate distances
        for(int i = 0; i < numAnchors; ++i) {
            distancesLeft(i) = (anchorPositionsLeft.row(i).transpose() - initialPos).norm();
            distancesRight(i) = (anchorPositionsRight.row(i).transpose() - initialPos).norm();
        }
        
        // Calculate f (equivalent to Python's f = distances_left - distances_right - doas)
        DynVector f = distancesLeft - distancesRight - doas;
        
        // Calculate gradient
        PosMatrix gradF(numAnchors, 3);
        for(int i = 0; i < numAnchors; ++i) {
            gradF.row(i) = (initialPos - anchorPositionsLeft.row(i).transpose()).transpose() / distancesLeft(i) -
                        (initialPos - anchorPositionsRight.row(i).transpose()).transpose() / distancesRight(i);
        }

        // Return new position estimate
        return initialPos - gradF.completeOrthogonalDecomposition().solve(f);
    }

    // Newton-Raphson step function (2D)
    static PosVector2D newtonRaphsonStep2D(const PosMatrix& anchorPositionsLeft,
                                const PosMatrix& anchorPositionsRight,
                                const DynVector& doas,
                                const PosVector2D& initialPosXY,
                                double fixedZ) {
        const int numAnchors = anchorPositionsLeft.rows();
        DynVector distancesLeft(numAnchors);
        DynVector distancesRight(numAnchors);
        // Store 3D difference vectors to avoid recalculation
        PosMatrix diffsLeft(numAnchors, 3);
        PosMatrix diffsRight(numAnchors, 3);


        // Construct 3D position from 2D estimate and fixed Z
        DynVector currentPos3D(3);
        currentPos3D << initialPosXY(0), initialPosXY(1), fixedZ;

        // Calculate distances and store difference vectors
        for(int i = 0; i < numAnchors; ++i) {
            diffsLeft.row(i) = anchorPositionsLeft.row(i) - currentPos3D.transpose();
            diffsRight.row(i) = anchorPositionsRight.row(i) - currentPos3D.transpose();
            distancesLeft(i) = diffsLeft.row(i).norm();
            distancesRight(i) = diffsRight.row(i).norm();
        }

        // Calculate f (measurement residual)
        DynVector f = distancesLeft - distancesRight - doas;

        // Calculate Jacobian (Gradient of f w.r.t X and Y) using stored differences
        Eigen::Matrix<double, Eigen::Dynamic, 2, 0, kMaxCapacity, 2> gradF_XY(numAnchors, 2);
        for(int i = 0; i < numAnchors; ++i) {
            // Reuse stored difference vectors and distances
            // Note: grad = (pos - anchor) / distance. Our stored diff is (anchor - pos). So we negate.
            DynVector gradLeft3D = -diffsLeft.row(i).transpose() / distancesLeft(i);
            DynVector gradRight3D = -diffsRight.row(i).transpose() / distancesRight(i);

            // Only take the X and Y components of the gradient difference
            gradF_XY.row(i) = (gradLeft3D - gradRight3D).head<2>().transpose();
        }

        // Solve for the step in X and Y
        // Using CompleteOrthogonalDecomposition for numerical stability
        PosVector2D deltaXY = gradF_XY.completeOrthogonalDecomposition().solve(f);

        // Return new 2D position estimate
        return initialPosXY - deltaXY;
    }

    // Main Newton-Raphson function
    DynVector newtonRaphson(const PosMatrix& anchorPositionsLeft,
                        const PosMatrix& anchorPositionsRight,
                        const DynVector& doas,
                        DynVector initialPos,
                        const int numUpdates) {
        DynVector pos = initialPos;
        for(int i = 0; i < numUpdates; ++i) {
            pos = newtonRaphsonStep(anchorPositionsLeft, anchorPositionsRight, doas, pos);
        }
        return pos;
    }

    // Main Newton-Raphson function (2D)
    PosVector2D newtonRaphson2D(const PosMatrix& anchorPositionsLeft,
                            const PosMatrix& anchorPositionsRight,
                            const DynVector& doas,
                            PosVector2D initialPosXY,
                            double fixedZ,
                            const int numUpdates) {
        PosVector2D posXY = initialPosXY;
        for(int i = 0; i < numUpdates; ++i) {
            posXY = newtonRaphsonStep2D(anchorPositionsLeft, anchorPositionsRight, doas, posXY, fixedZ);
        }
        return posXY;
    }
}
