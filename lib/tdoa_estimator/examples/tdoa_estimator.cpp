#include "tdoa_estimator.hpp"

/**
 * Estimates 3D position using Time Difference of Arrival (TDoA) measurements through gradient descent.
 * 
 * Mathematical Background:
 * - TDoA Measurement: τᵢⱼ = (dᵢ - dⱼ)/c, where:
 *   - dᵢ is distance to anchor i
 *   - dⱼ is distance to reference anchor
 *   - c is speed of signal (typically light/sound)
 * 
 * - Error Function: E = Σ(τᵢⱼ_measured - τᵢⱼ_calculated)²
 * 
 * - Gradient for each anchor pair: ∇E = 2(τᵢⱼ_calc - τᵢⱼ_meas) * (uᵢ - uⱼ)
 *   where uᵢ is unit vector from position to anchor i
 */
std::pair<Eigen::Vector3d, TDoAEstimator::SolutionMetrics> TDoAEstimator::calculatePosition(
    const Eigen::VectorXd& tdoa_measurements,  // Measured time differences relative to reference anchor
    const Eigen::Vector3d& initial_guess,      // Initial position estimate
    const double learning_rate_init,           // Initial gradient descent step size
    const double momentum,                     // Momentum coefficient for faster convergence
    const int max_iterations) const {
    
    // Initialize position at the initial guess
    Eigen::Vector3d position = initial_guess;
    // Initialize velocity for momentum-based updates
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double learning_rate = learning_rate_init;
    double prev_error = std::numeric_limits<double>::infinity();
    std::vector<double> residuals;

    for (int iter = 0; iter < max_iterations; ++iter) {
        Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
        residuals.clear();

        // Calculate Euclidean distances from current position to all anchors
        // Formula: dᵢ = ||anchor_posᵢ - position||
        Eigen::VectorXd distances = (anchor_positions_.rowwise() - position.transpose()).rowwise().norm();
        
        // Distance to reference anchor (anchor 0)
        double ref_distance = distances(0);

        double total_error = 0.0;
        // Iterate through non-reference anchors
        for (int i = 1; i < anchor_positions_.rows(); ++i) {
            // Calculate expected TDoA: (dᵢ - d_ref)
            double calc_tdoa = distances(i) - ref_distance;
            
            // Difference between measured and calculated TDoA
            double residual = calc_tdoa - tdoa_measurements(i-1);
            residuals.push_back(residual);

            // Calculate direction vectors (unit vectors)
            // Direction from position to current anchor: uᵢ = (p - aᵢ)/||p - aᵢ||
            Eigen::Vector3d dir_i = (position - anchor_positions_.row(i).transpose()) / (distances(i) + 1e-10);
            // Direction from position to reference anchor: u_ref = (p - a_ref)/||p - a_ref||
            Eigen::Vector3d dir_ref = (position - anchor_positions_.row(0).transpose()) / (ref_distance + 1e-10);
            
            // Update gradient: g += residual * (uᵢ - u_ref)
            // This points in the direction that would reduce the TDoA error
            gradient += residual * (dir_i - dir_ref);

            // Accumulate squared error
            total_error += residual * residual;
        }

        // Adaptive learning rate adjustment
        // Decrease if error grew, increase if error decreased
        if (total_error > prev_error) {
            learning_rate *= 0.95;  // Slow down if overshooting
        } else {
            learning_rate *= 1.01;  // Speed up if moving in right direction
        }
        learning_rate = std::clamp(learning_rate, 1e-6, 0.05);  // Keep learning rate in reasonable bounds

        // Update position using momentum
        // v = mv - αg (m: momentum, α: learning rate, g: gradient)
        velocity = momentum * velocity - learning_rate * gradient;
        // p = p + v
        position += velocity;

        // Check for convergence
        if (total_error < 1e-8) break;
        prev_error = total_error;
    }

    // Calculate final solution metrics
    SolutionMetrics metrics;
    double sum = 0.0;
    double max_residual = 0.0;
    for (double r : residuals) {
        sum += r * r;
        max_residual = std::max(max_residual, std::abs(r));
    }
    // Root Mean Square Error of residuals
    metrics.rmse = std::sqrt(sum / residuals.size());
    // Geometric Dilution of Precision - measure of anchor geometry quality
    metrics.gdop = calculateGDOP(position, anchor_positions_);
    metrics.residuals_std = 0.0;  // Standard deviation of residuals
    metrics.max_residual = max_residual;
    metrics.final_error = prev_error;
    // Higher confidence when both RMSE and GDOP are low
    metrics.confidence_score = 1.0 / (1.0 + metrics.rmse * metrics.gdop);

    return {position, metrics};
}

double TDoAEstimator::calculateGDOP(const Eigen::Vector3d& position, const Eigen::MatrixXd& anchors) {
    int n_anchors = anchors.rows();
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_anchors - 1, 3);
    
    Eigen::VectorXd distances = (anchors.rowwise() - position.transpose()).rowwise().norm();
    double ref_distance = distances(0);

    for (int i = 1; i < n_anchors; ++i) {
        Eigen::Vector3d dir_i = (position - anchors.row(i).transpose()) / distances(i);
        Eigen::Vector3d dir_ref = (position - anchors.row(0).transpose()) / ref_distance;
        H.row(i-1) = dir_i - dir_ref;
    }

    try {
        return std::sqrt((H.transpose() * H).inverse().trace());
    } catch (...) {
        return std::numeric_limits<double>::infinity();
    }
}

Eigen::VectorXd TDoAEstimator::simulateTDoAMeasurements(
    const Eigen::Vector3d& position,
    const Eigen::MatrixXd& anchor_positions,
    double noise_std,
    std::default_random_engine& generator) {
    
    Eigen::VectorXd distances = (anchor_positions.rowwise() - position.transpose()).rowwise().norm();
    double ref_distance = distances(0);
    Eigen::VectorXd tdoa = distances.segment(1, distances.size() - 1).array() - ref_distance;

    // Add noise
    std::normal_distribution<double> distribution(0.0, noise_std);
    for (int i = 0; i < tdoa.size(); ++i) {
        tdoa(i) += distribution(generator);
    }

    return tdoa;
}