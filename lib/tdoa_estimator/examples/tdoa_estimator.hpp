#pragma once


#include <Eigen.h>
#include <vector>
#include <random>
#include <cmath>

class TDoAEstimator {
public:
    struct SolutionMetrics {
        double rmse;
        double gdop;
        double residuals_std;
        double max_residual;
        double final_error;
        double confidence_score;
    };

    TDoAEstimator(const Eigen::MatrixXd& anchor_positions)
        : anchor_positions_(anchor_positions) {}

    std::pair<Eigen::Vector3d, SolutionMetrics> calculatePosition(
        const Eigen::VectorXd& tdoa_measurements,
        const Eigen::Vector3d& initial_guess,
        const double learning_rate = 0.02,
        const double momentum = 0.95,
        const int max_iterations = 2000) const;

    // Helper functions
    static double calculateGDOP(const Eigen::Vector3d& position, const Eigen::MatrixXd& anchors);
    
    // For testing/simulation purposes
    static Eigen::VectorXd simulateTDoAMeasurements(
        const Eigen::Vector3d& position,
        const Eigen::MatrixXd& anchor_positions,
        double noise_std,
        std::default_random_engine& generator);

private:
    Eigen::MatrixXd anchor_positions_;
};