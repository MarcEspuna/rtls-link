#include "tdoa_newton_raphson.hpp"

namespace tdoa_estimator {
    // Newton-Raphson step function
    static Eigen::VectorXd newtonRaphsonStep(const Eigen::MatrixXd& anchorPositionsLeft,
                            const Eigen::MatrixXd& anchorPositionsRight,
                            const Eigen::VectorXd& doas,
                            const Eigen::VectorXd& initialPos) {
        const int numAnchors = anchorPositionsLeft.rows();
        Eigen::VectorXd distancesLeft(numAnchors);
        Eigen::VectorXd distancesRight(numAnchors);
        
        // Calculate distances
        for(int i = 0; i < numAnchors; ++i) {
            distancesLeft(i) = (anchorPositionsLeft.row(i).transpose() - initialPos).norm();
            distancesRight(i) = (anchorPositionsRight.row(i).transpose() - initialPos).norm();
        }
        
        // Calculate f (equivalent to Python's f = distances_left - distances_right - doas)
        Eigen::VectorXd f = distancesLeft - distancesRight - doas;
        
        // Calculate gradient
        Eigen::MatrixXd gradF(numAnchors, 3);
        for(int i = 0; i < numAnchors; ++i) {
            gradF.row(i) = (initialPos - anchorPositionsLeft.row(i).transpose()).transpose() / distancesLeft(i) -
                        (initialPos - anchorPositionsRight.row(i).transpose()).transpose() / distancesRight(i);
        }
        
        // Return new position estimate 
        return initialPos - gradF.completeOrthogonalDecomposition().solve(f);
    }

    // Main Newton-Raphson function
    Eigen::VectorXd newtonRaphson(const Eigen::MatrixXd& anchorPositionsLeft,
                        const Eigen::MatrixXd& anchorPositionsRight,
                        const Eigen::VectorXd& doas,
                        Eigen::VectorXd initialPos,
                        const int numUpdates) {
        Eigen::VectorXd pos = initialPos;
        for(int i = 0; i < numUpdates; ++i) {
            pos = newtonRaphsonStep(anchorPositionsLeft, anchorPositionsRight, doas, pos);
        }
        return pos;
    }
}
