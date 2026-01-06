#include "tdoa_estimator.hpp"
#include <Eigen/LU>  // For .inverse() in calculateGDOP3D

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


double TDoAEstimator::calculateGDOP3D(const Eigen::Vector3d& position, const Eigen::MatrixXd& anchors) {
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

std::pair<Eigen::Vector2d, TDoAEstimator::SolutionMetrics> TDoAEstimator::calculatePosition2D(
    const etl::vector<TDoAMeasurement, kMaxTdoaSamples>& tdoa_measurements,
    const Eigen::Vector2d& initial_guess,
    const double learning_rate_init,
    const double momentum,
    const int max_iterations) const {
    
    if (mode_ != Mode::MODE_2D) {
        throw std::runtime_error("Estimator not initialized for 2D mode");
    }
        
    // Function to check if point is inside polygon using ray casting
    auto isInsidePolygon = [](const Eigen::Vector2d& point, 
                             const etl::vector<Eigen::Vector2d, kMaxAnchors>& vertices) -> bool {
        bool inside = false;
        int j = vertices.size() - 1;
        
        for (int i = 0; i < vertices.size(); i++) {
            if ((vertices[i].y() > point.y()) != (vertices[j].y() > point.y()) &&
                point.x() < (vertices[j].x() - vertices[i].x()) * 
                (point.y() - vertices[i].y()) / (vertices[j].y() - vertices[i].y()) + 
                vertices[i].x()) {
                inside = !inside;
            }
            j = i;
        }
        return inside;
    };
    
    // Project point onto polygon if outside
    auto projectOntoPolygon = [this](const Eigen::Vector2d& point) -> Eigen::Vector2d {
        Eigen::Vector2d projected = point;
        double min_dist = std::numeric_limits<double>::infinity();
        
        // For each edge of the polygon
        for (size_t i = 0; i < polygon_.size(); i++) {
            size_t j = (i + 1) % polygon_.size();
            Eigen::Vector2d edge = polygon_[j] - polygon_[i];
            Eigen::Vector2d point_vec = point - polygon_[i];
            
            double t = std::clamp(point_vec.dot(edge) / edge.dot(edge), 0.0, 1.0);
            Eigen::Vector2d projection = polygon_[i] + t * edge;
            
            double dist = (point - projection).norm();
            if (dist < min_dist) {
                min_dist = dist;
                projected = projection;
            }
        }
        return projected;
    };
    
    Eigen::Vector2d position = initial_guess;
    if (!isInsidePolygon(position, polygon_)) {
        position = projectOntoPolygon(position);
    }
    
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double learning_rate = learning_rate_init;
    double prev_error = std::numeric_limits<double>::infinity();
    etl::vector<double, kMaxTdoaSamples> residuals;
    residuals.reserve(tdoa_measurements.size());
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
        residuals.clear();
        
        Eigen::VectorXd distances = calculate3DDistances(position);
        double total_error = 0.0;
        
        for (const auto& measurement : tdoa_measurements) {
            double dist_a = distances(measurement.anchor_a);
            double dist_b = distances(measurement.anchor_b);
            double calc_tdoa = dist_a - dist_b;
            double residual = calc_tdoa - measurement.tdoa;
            residuals.push_back(residual);
            
            Eigen::Vector2d dir_a = (position - anchor_positions_.row(measurement.anchor_a).head(2).transpose()) / (dist_a + 1e-10);
            Eigen::Vector2d dir_b = (position - anchor_positions_.row(measurement.anchor_b).head(2).transpose()) / (dist_b + 1e-10);
            gradient += residual * (dir_a - dir_b);
            total_error += residual * residual;
        }
        
        if (total_error > prev_error) {
            learning_rate *= 0.95;
        } else {
            learning_rate *= 1.01;
        }
        learning_rate = std::clamp(learning_rate, 1e-6, 0.05);
        
        velocity = momentum * velocity - learning_rate * gradient;
        Eigen::Vector2d new_position = position + velocity;
        
        // Project back to polygon if step takes us outside
        if (!isInsidePolygon(new_position, polygon_)) {
            new_position = projectOntoPolygon(new_position);
            // Reset velocity when hitting boundary to prevent oscillation
            velocity = Eigen::Vector2d::Zero();
        }
        
        position = new_position;
        
        // Convergence check
        const double convergence_threshold = 1e-6;
        if (std::abs(total_error - prev_error) < convergence_threshold && total_error < 1e-2) {
            break;
        }
        // if (total_error < 1e-6) break;
        prev_error = total_error;
    }
    
    SolutionMetrics metrics;
    // Calculate metrics...
    
    return {position, metrics};
}

Eigen::VectorXd TDoAEstimator::calculate3DDistances(const Eigen::Vector2d& position2d, double z_height) const {
    Eigen::VectorXd distances(anchor_positions_.rows());
    for (int i = 0; i < anchor_positions_.rows(); ++i) {
        double dx = position2d.x() - anchor_positions_(i, 0);
        double dy = position2d.y() - anchor_positions_(i, 1);
        double dz = z_height - anchor_positions_(i, 2);
        distances(i) = std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    return distances;
}

Eigen::VectorXd TDoAEstimator::calculate3DDistances(const Eigen::Vector3d& position) const {
    return (anchor_positions_.rowwise() - position.transpose()).rowwise().norm();
}

etl::vector<TDoAEstimator::TDoAMeasurement, TDoAEstimator::kMaxTdoaSamples> TDoAEstimator::simulateTDoAMeasurements(
        const Eigen::Vector3d& position,
        const Eigen::MatrixXd& anchor_positions,
        const std::vector<std::pair<int, int>>& anchor_pairs,
        double noise_std,
        std::default_random_engine& generator) {
    
    etl::vector<TDoAMeasurement, kMaxTdoaSamples> measurements;
    Eigen::VectorXd distances = (anchor_positions.rowwise() - position.transpose()).rowwise().norm();
    
    std::normal_distribution<double> distribution(0.0, noise_std);
    
    for (const auto& pair : anchor_pairs) {
        TDoAMeasurement measurement;
        measurement.anchor_a = pair.first;
        measurement.anchor_b = pair.second;
        measurement.tdoa = distances(pair.first) - distances(pair.second);
            
        // Add noise
        measurement.tdoa += distribution(generator);
        measurements.push_back(measurement);
    }

    return measurements;
}

bool TDoAEstimator::isPointInsideAnchorBoundary(const Eigen::Vector2d& point) const {
    int n = anchor_positions_.rows();
    bool inside = false;
    
    for (int i = 0, j = n-1; i < n; j = i++) {
        if (((anchor_positions_(i,1) > point.y()) != (anchor_positions_(j,1) > point.y())) &&
            (point.x() < (anchor_positions_(j,0) - anchor_positions_(i,0)) * 
             (point.y() - anchor_positions_(i,1)) / (anchor_positions_(j,1) - anchor_positions_(i,1)) + 
             anchor_positions_(i,0))) {
            inside = !inside;
        }
    }
    return inside;
}